"""PC regression tests for J, without camera hardware."""
import ast
from pathlib import Path
import types
import queue
import unittest
from unittest.mock import Mock, patch

import selftest_tasks as smoke
import protocol
import screen_echo
from material_state import MotionWindow
from selftest_echo import FakeDisp, FakeImg


class MaterialStateTests(unittest.TestCase):
    def test_window_stationary_motion_and_round_trip(self):
        for xs, expected in [([100, 101, 100, 101], 1),
                             ([100, 104, 108, 112], 0),
                             ([100, 110, 110, 100], 0)]:
            window = MotionWindow(300, 3, 3, 150)
            states = [window.update(i * 100, x, 240) for i, x in enumerate(xs)]
            self.assertEqual(states[-1], expected)
            self.assertEqual(states[0], 2)

    def test_missing_frames_and_ambiguous_target_reset(self):
        window = MotionWindow(300, 3, 3, 150)
        for t in range(0, 400, 100):
            state = window.update(t, 100, 200)
        self.assertEqual(state, 1)
        self.assertEqual(window.update(600, 100, 200), 2)
        self.assertEqual(window.update(700, 100, 200, False), 2)
        self.assertEqual(window.samples, [])

    def test_wire_format(self):
        self.assertEqual(protocol.parse_command("J"), ("J", ""))
        self.assertEqual(protocol.pack_material_state([]), "MSTATE,0,0")
        payload = protocol.pack_material_state([("1", 320, 240, 1), ("3", 450, 210, 1)], True)
        self.assertEqual(payload, "MSTATE,1,2,1,320,240,1,3,450,210,1")
        frame = protocol.encode_frame(payload)
        self.assertEqual(frame[2], len(payload))
        self.assertEqual(frame[-2], protocol.crc8(frame[2:-2]))
        for records in ([], [("1", 320, 240, 0)], [("1", 320, 240, 2)]):
            with self.assertRaises(ValueError):
                protocol.pack_material_state(records, True)
        with self.assertRaises(ValueError):
            protocol.pack_material_state([("1", 320, 240, 1), ("1", 450, 210, 1)])

    def run_task(self, frames, reset_at=20000, busy_tx=False):
        clock = [0]
        remaining = iter(frames)

        def detect(img, roi, pixels):
            self.assertIsNone(roi)
            clock[0] += 100
            return next(remaining, [])

        clf = types.SimpleNamespace(detect=detect, resolve_code=lambda img, obj: obj["code"])
        link = smoke._Link([])
        if busy_tx:
            pending = queue.Queue()
            pending.put("pending frame")
            link.ser = types.SimpleNamespace(_tx=pending)
        link.poll_command = lambda: "R" if clock[0] >= reset_at else None
        with patch.object(smoke.tasks, "ticks_ms", lambda: clock[0]):
            result = smoke.tasks.task_material_state(smoke.FakeCam(1), clf, None,
                           p=smoke.P_ID, ctl=smoke.tasks.TaskCtl(link))
        self.assertFalse(any(message.startswith(("MOVE,", "MOVEY,", "MOVEXY,")) for message in link.sent))
        return result, link.sent, clock[0]

    def test_busy_tx_does_not_block_terminal_response(self):
        result, sent, _ = self.run_task([], reset_at=1000, busy_tx=True)
        self.assertEqual(result, "READY")
        self.assertEqual(sent, [])
        result, sent, _ = self.run_task([[self.item()]] * 18, busy_tx=True)
        self.assertEqual(result, "MSTATE,1,1,1,320,240,1")
        self.assertEqual(sent, [])

    @staticmethod
    def item(code="1", x=320, y=240):
        return dict(code=code, cx=x, cy=y, x=x-15, y=y-15,
                    w=30, h=30, pixels=200, src="ai")

    def test_j_waits_beyond_eight_seconds_then_returns_all_stopped(self):
        frames = [[]] * 90 + [[self.item(), self.item("3", 450)]] * 18
        result, sent, elapsed = self.run_task(frames)
        self.assertGreater(elapsed, 8000)
        self.assertEqual(result, "MSTATE,1,2,1,320,240,1,3,450,240,1")
        self.assertEqual(sent, [])

    def test_partial_material_can_be_confirmed(self):
        result, _, _ = self.run_task([[self.item(x=5)]] * 18)
        self.assertEqual(result, "MSTATE,1,1,1,5,240,1")

    def test_confirmation_uses_100ms_after_motion_window_stabilizes(self):
        frames = [[self.item()]] * 10
        result, sent, elapsed = self.run_task(frames)
        self.assertEqual(result, "MSTATE,1,1,1,320,240,1")
        self.assertEqual(sent, [])
        self.assertEqual(elapsed, 500)

    def test_moving_duplicates_and_empty_wait_for_reset(self):
        sequences = [[], [[self.item(x=200 + 5*i)] for i in range(10)],
                     [[self.item(), self.item(x=450)]] * 10]
        for frames in sequences:
            result, sent, elapsed = self.run_task(frames, reset_at=1000)
            self.assertEqual(result, "READY")
            self.assertFalse(any(message.startswith("MSTATE,1,") for message in sent))

    @staticmethod
    def load_main_functions(env, *names):
        tree = ast.parse((Path(__file__).parent / "main.py").read_text(encoding="utf-8"))
        functions = [node for node in tree.body
                     if isinstance(node, ast.FunctionDef) and node.name in names]
        exec(compile(ast.Module(body=functions, type_ignores=[]), "main.py", "exec"), env)

    def test_main_dispatch_entry(self):
        task = Mock(return_value="MSTATE,1,1,1,320,240,1")
        reply = Mock()
        env = dict(protocol=protocol, tasks=types.SimpleNamespace(task_material_state=task),
                   reply=reply, cam=object(), clf=object(), disp=None,
                   pline=object(), ctl=object(), pack_error=protocol.pack_error,
                   screen_echo=screen_echo)
        self.load_main_functions(env, "dispatch")
        env["dispatch"]("J", "")
        task.assert_called_once()
        reply.assert_called_with("MSTATE,1,1,1,320,240,1")
        task.reset_mock()
        env["dispatch"]("J", "bad")
        task.assert_not_called()
        reply.assert_called_with(protocol.pack_error("j_arg"))

    def test_j_terminal_reply_refreshes_the_last_frame(self):
        screen_echo.echo.reset()
        self.addCleanup(screen_echo.echo.reset)
        previous = "HDISC,304,314,-16,1,3,0"
        payload = "MSTATE,1,1,1,320,240,1"
        screen_echo.set_state("RUNNING")
        screen_echo.set_tx(previous)
        frame = FakeImg()
        real = FakeDisp()

        def task(cam, clf, disp, p=None, ctl=None):
            disp.show(frame)
            return payload

        def send(data):
            self.assertEqual(real.shown, 1)
            self.assertEqual(frame.texts()[-2], "TX: " + previous)

        link = types.SimpleNamespace(send_payload=Mock(side_effect=send))
        env = dict(protocol=protocol, tasks=types.SimpleNamespace(task_material_state=task),
                   cam=object(), clf=object(), disp=screen_echo.wrap(real),
                   pline=object(), ctl=types.SimpleNamespace(reset_flag=False),
                   link=link, screen_echo=screen_echo, log=Mock())
        self.load_main_functions(env, "dispatch", "reply")
        env["dispatch"]("J", "")

        link.send_payload.assert_called_once_with(payload)
        self.assertEqual(real.shown, 2)
        self.assertIs(real.last, frame)
        self.assertEqual(frame.texts()[-2], "TX: " + payload)
        self.assertIn("IDLE", frame.texts()[-1])
        self.assertEqual(screen_echo.echo.tx_count, 2)


if __name__ == "__main__":
    unittest.main()
