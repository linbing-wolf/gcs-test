"""PC regression tests for J, without camera hardware."""
import ast
from pathlib import Path
import types
import queue
import tempfile
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

    def test_stable_seed_returns_stationary_immediately(self):
        window = MotionWindow(240, 3, 3, 150)
        window.seed_stable(1000, 320, 240, step_ms=40)
        self.assertEqual(window.update(1000, 320, 240), 1)
        self.assertEqual(window.update(1100, 350, 240), 0)

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

    def run_task(self, frames, reset_at=20000, busy_tx=False, seed=None):
        smoke.tasks.clear_material_state_seed()
        if seed:
            smoke.tasks._remember_material_state_seed(seed, 0)
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

    def test_confirmation_uses_1000ms_after_motion_window_stabilizes(self):
        frames = [[self.item()]] * 14
        result, sent, elapsed = self.run_task(frames)
        self.assertEqual(result, "MSTATE,1,1,1,320,240,1")
        self.assertEqual(sent, [])
        self.assertEqual(elapsed, 1400)

    def test_stable_h_coordinates_seed_the_next_j(self):
        result, sent, elapsed = self.run_task(
            [[self.item()]] * 11,
            seed=[("1", 320, 240)],
        )
        self.assertEqual(result, "MSTATE,1,1,1,320,240,1")
        self.assertEqual(sent, [])
        self.assertEqual(elapsed, 1100)

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

    def test_reply_refreshes_the_terminal_frame_and_state(self):
        screen_echo.echo.reset()
        self.addCleanup(screen_echo.echo.reset)
        payload = "HDISC2,0,3,2,24,15,1,171,256,3,360,3"
        frame = FakeImg()
        real = FakeDisp()
        disp = screen_echo.wrap(real)
        disp.show(frame)

        link = types.SimpleNamespace(send_payload=Mock())
        env = dict(disp=disp, link=link, screen_echo=screen_echo, log=Mock(),
                   ctl=types.SimpleNamespace(reset_flag=False),
                   tasks=types.SimpleNamespace(reset_guard=Mock()), TASK=None)
        self.load_main_functions(env, "reply")
        env["reply"](payload)

        link.send_payload.assert_called_once_with(payload)
        self.assertEqual(real.shown, 2)
        self.assertIs(real.last, frame)
        self.assertEqual(frame.texts()[-2], "TX: " + payload)
        self.assertIn("IDLE", frame.texts()[-1])
        self.assertEqual(screen_echo.echo.tx_count, 1)

    def test_j_terminal_reply_refreshes_once(self):
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

    def test_reply_arms_capture_immediately_after_send(self):
        events = []
        link = types.SimpleNamespace(
            send_payload=Mock(side_effect=lambda payload: events.append("send")))
        arm = Mock(side_effect=lambda ctl, payload: events.append("arm"))
        fake_screen = types.SimpleNamespace(
            set_tx=Mock(side_effect=lambda payload: events.append("screen")),
            set_state=Mock(),
            EchoDisplay=type("EchoDisplay", (), {}),
        )
        env = dict(
            disp=None,
            link=link,
            screen_echo=fake_screen,
            log=Mock(),
            ctl=types.SimpleNamespace(reset_flag=False),
            tasks=types.SimpleNamespace(
                reset_guard=Mock(),
                arm_disc_terminal_followup=arm,
                cancel_disc_terminal_followup=Mock(),
            ),
            TASK=None,
        )
        self.load_main_functions(env, "reply")
        env["reply"]("DISC,320,240,7,1")
        self.assertEqual(events, ["send", "arm", "screen"])


class DiscFollowupCaptureTests(unittest.TestCase):
    def test_capture_is_armed_after_reply_and_serviced_when_due(self):
        ctl = smoke.tasks.TaskCtl(smoke._Link([]))
        rec = Mock()
        now = [1000]
        payload = "DISC,320,240,7,1"
        with patch.multiple(
                smoke.config,
                DISC_TERMINAL_CAPTURE_ENABLED=True,
                DISC_TERMINAL_CAPTURE_DELAYS_MS=(6000, 10000),
                DISC_TERMINAL_CAPTURE_DELAY_MS=600,
                DISC_TERMINAL_CAPTURE_DELAY_D_MS=None,
                DISC_TERMINAL_CAPTURE_DELAY_E_MS=None), \
                patch.object(smoke.tasks, "ticks_ms", lambda: now[0]), \
                patch.object(smoke.tasks, "capture_disc_terminal_followup") as capture:
            self.assertTrue(smoke.tasks.schedule_disc_terminal_followup(
                ctl, rec, (96, 0, 448, 400), "D", payload, p=smoke.P_ID))
            self.assertIsNone(ctl.disc_pending_capture["due_ms"])
            self.assertEqual(
                ctl.disc_pending_capture["delays_ms"], (6000, 10000))
            self.assertTrue(smoke.tasks.arm_disc_terminal_followup(ctl, payload))
            self.assertEqual(ctl.disc_pending_capture["due_ms"], 7000)

            now[0] = 6999
            self.assertFalse(smoke.tasks.service_disc_terminal_followup(
                smoke.FakeCam(1), ctl))
            capture.assert_not_called()

            now[0] = 7000
            self.assertTrue(smoke.tasks.service_disc_terminal_followup(
                smoke.FakeCam(1), ctl))
            capture.assert_called_once()
            self.assertIs(capture.call_args.kwargs["rec"], rec)
            self.assertEqual(capture.call_args.args[3], payload)
            self.assertEqual(capture.call_args.kwargs["target_delay_ms"], 6000)
            self.assertEqual(capture.call_args.kwargs["actual_delay_ms"], 6000)
            self.assertFalse(capture.call_args.kwargs["close_record"])
            self.assertEqual(ctl.disc_pending_capture["due_ms"], 11000)

            now[0] = 10999
            self.assertFalse(smoke.tasks.service_disc_terminal_followup(
                smoke.FakeCam(1), ctl))
            self.assertEqual(capture.call_count, 1)

            now[0] = 11000
            self.assertTrue(smoke.tasks.service_disc_terminal_followup(
                smoke.FakeCam(1), ctl))
            self.assertIsNone(ctl.disc_pending_capture)
            self.assertEqual(capture.call_count, 2)
            self.assertEqual(capture.call_args.kwargs["target_delay_ms"], 10000)
            self.assertEqual(capture.call_args.kwargs["actual_delay_ms"], 10000)
            self.assertTrue(capture.call_args.kwargs["close_record"])

    def test_cancel_closes_pending_debug_record(self):
        ctl = smoke.tasks.TaskCtl(smoke._Link([]))
        rec = Mock()
        payload = "DISC,320,240,7,1"
        with patch.object(smoke.config, "DISC_TERMINAL_CAPTURE_ENABLED", True), \
                patch.object(smoke.config, "DISC_TERMINAL_CAPTURE_DELAYS_MS", None), \
                patch.object(smoke.config, "DISC_TERMINAL_CAPTURE_DELAY_MS", 600):
            self.assertTrue(smoke.tasks.schedule_disc_terminal_followup(
                ctl, rec, (96, 0, 448, 400), "E", payload, p=smoke.P_ID))
            self.assertTrue(smoke.tasks.cancel_disc_terminal_followup(
                ctl, "next_command"))
        self.assertIsNone(ctl.disc_pending_capture)
        rec.close.assert_called_once()
        self.assertEqual(rec.close.call_args.args[0], payload)

    def test_none_terminal_uses_e_specific_delay(self):
        ctl = smoke.tasks.TaskCtl(smoke._Link([]))
        rec = Mock()
        with patch.multiple(
                smoke.config,
                DISC_TERMINAL_CAPTURE_ENABLED=True,
                DISC_TERMINAL_CAPTURE_DELAYS_MS=None,
                DISC_TERMINAL_CAPTURE_DELAY_MS=600,
                DISC_TERMINAL_CAPTURE_DELAY_D_MS=700,
                DISC_TERMINAL_CAPTURE_DELAY_E_MS=350):
            self.assertTrue(smoke.tasks.schedule_disc_terminal_followup(
                ctl, rec, (96, 0, 448, 400), "E", "NONE,E", p=smoke.P_ID))
        self.assertEqual(
            ctl.disc_pending_capture["delays_ms"], (350,))

    def test_new_terminal_closes_the_previous_pending_record(self):
        ctl = smoke.tasks.TaskCtl(smoke._Link([]))
        first = Mock()
        second = Mock()
        with patch.object(smoke.config, "DISC_TERMINAL_CAPTURE_ENABLED", True), \
                patch.object(smoke.config, "DISC_TERMINAL_CAPTURE_DELAYS_MS", (6000, 10000)), \
                patch.object(smoke.config, "DISC_TERMINAL_CAPTURE_DELAY_MS", 600):
            self.assertTrue(smoke.tasks.schedule_disc_terminal_followup(
                ctl, first, (96, 0, 448, 400), "D",
                "DISC,320,240,7,1", p=smoke.P_ID))
            self.assertTrue(smoke.tasks.schedule_disc_terminal_followup(
                ctl, second, (96, 0, 448, 400), "E",
                "DISC,320,240,-7,1", p=smoke.P_ID))
        first.close.assert_called_once()
        self.assertEqual(
            first.close.call_args.args[1]["cancelled"], "replaced")
        self.assertIs(ctl.disc_pending_capture["rec"], second)

    def test_disc_debug_start_is_idempotent(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec = smoke.tasks.disc_debug.DiscDebug(
                "D", tmp, max_frames=2, keep_tasks=0)
            self.assertTrue(rec.start())
            first_dir = rec.dir
            self.assertTrue(rec.start())
            self.assertEqual(rec.dir, first_dir)
            self.assertIsNotNone(rec.meta_fp)
            rec.close("DISC,320,240,0,1")

    def test_capture_reuses_started_record_and_closes_once(self):
        rec = Mock()
        rec.snap.return_value = "after_terminal.jpg"
        now = [100]

        def fake_sleep(delay_ms):
            now[0] += delay_ms

        with patch.object(smoke.tasks, "ticks_ms", side_effect=lambda: now[0]), \
                patch.object(smoke.tasks, "sleep_ms", side_effect=fake_sleep) as sleep, \
                patch.object(
                    smoke.tasks.vision, "find_disc_center",
                    return_value=(320, 240, 100)), \
                patch.object(smoke.tasks.cam_light, "is_on", return_value=False), \
                patch.object(smoke.tasks.cam_light, "on", return_value=True), \
                patch.object(smoke.tasks.cam_light, "off") as light_off:
            image_name = smoke.tasks.capture_disc_terminal_followup(
                smoke.FakeCam(1),
                (96, 0, 448, 400),
                "D",
                "DISC,320,240,7,1",
                p=smoke.P_ID,
                delay_ms=600,
                target_delay_ms=600,
                rec=rec,
            )
        self.assertEqual(image_name, "after_terminal.jpg")
        sleep.assert_called_once_with(600)
        rec.start.assert_not_called()
        self.assertEqual(
            rec.snap.call_args.args[2], "after_terminal_600ms")
        rec.close.assert_called_once()
        self.assertEqual(rec.close.call_args.args[1]["actual_delay_ms"], 600)
        self.assertTrue(rec.close.call_args.args[1]["detected"])
        light_off.assert_called_once()

    def test_intermediate_capture_keeps_record_open_for_second_shot(self):
        rec = Mock()
        rec.snap.return_value = "after_terminal_6000ms.jpg"
        with patch.object(smoke.tasks, "ticks_ms", return_value=6000), \
                patch.object(
                    smoke.tasks.vision, "find_disc_center",
                    return_value=(320, 240, 100)), \
                patch.object(smoke.tasks.cam_light, "is_on", return_value=True):
            image_name = smoke.tasks.capture_disc_terminal_followup(
                smoke.FakeCam(1),
                (96, 0, 448, 400),
                "D",
                "DISC,320,240,7,1",
                p=smoke.P_ID,
                delay_ms=0,
                target_delay_ms=6000,
                actual_delay_ms=6000,
                rec=rec,
                close_record=False,
            )
        self.assertEqual(image_name, "after_terminal_6000ms.jpg")
        rec.close.assert_not_called()
        self.assertEqual(
            rec.snap.call_args.args[2], "after_terminal_6000ms")


if __name__ == "__main__":
    unittest.main()
