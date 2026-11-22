"""PC checks for the disabled direction-protocol overlay."""
import sys
from pathlib import Path
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.append(str(ROOT.parents[1] / "maixcam_app_v2"))
import h_direction as hd
import selftest_tasks as smoke
import protocol


def candidate(code, x, eligible=True):
    return {"code": code, "raw": (x, 240), "pair_eligible": eligible}


class DirectionTests(unittest.TestCase):
    def test_direction_and_pair_order(self):
        for step, direction, pair in [(10, hd.CCW, ("2", "1")),
                                      (-10, hd.CW, ("1", "2"))]:
            tracker = hd.DirectionTracker()
            for i in range(4):
                points = [candidate("1", 200 + step * i), candidate("2", 400 + step * i)]
                tracker.update(points, i * 100)
            self.assertEqual(tracker.direction, direction)
            self.assertEqual(tracker.order_pair(points), pair)

    def test_stationary_jitter_edges_and_movement_exclusion(self):
        tracker = hd.DirectionTracker()
        for i in range(20):
            tracker.update([candidate("1", 200 + i % 3), candidate("2", 400 + i * 10, False)], i * 100)
        self.assertEqual(tracker.direction, hd.UNKNOWN)
        for i in range(5):
            tracker.update([candidate("1", 200 + i * 10)], 2100 + i * 100, False)
        self.assertEqual(tracker.direction, hd.UNKNOWN)
        self.assertEqual(tracker.order_pair([candidate("1", 200), candidate("2", 400)]), ())

    def test_conflicting_motion_and_large_jumps(self):
        tracker = hd.DirectionTracker()
        for i in range(4):
            tracker.update([candidate("1", 200 + i * 10), candidate("2", 400 - i * 10)], i * 100)
        self.assertEqual(tracker.direction, hd.UNKNOWN)
        tracker = hd.DirectionTracker()
        for i in range(5):
            tracker.update([candidate("1", i * 100)], i * 100)
        self.assertEqual(tracker.direction, hd.UNKNOWN)

    def test_packets(self):
        self.assertEqual(hd.pack_colors("3", "2", hd.CW), "HCOLORS,3,2,0")
        self.assertEqual(hd.pack_disc(383, 151, 63, "3", "2", hd.CCW),
                         "HDISC,383,151,63,3,2,1")
        with self.assertRaises(ValueError):
            hd.pack_colors("1", "1", hd.CCW)
        payload = hd.pack_disc(383, 151, 63, "3", "2", hd.CCW)
        frame = protocol.encode_frame(payload)
        self.assertEqual(frame[:2], b"\xaa\x55")
        self.assertEqual(frame[2], len(payload))
        self.assertEqual(frame[3:-2].decode(), payload)
        self.assertEqual(frame[-2], protocol.crc8(frame[2:-2]))
        self.assertEqual(frame[-1], 13)

    def test_fallback_and_packet_validation(self):
        for fallback in (0, 1):
            tracker = hd.DirectionTracker(fallback)
            self.assertIsNone(tracker.direction)
            self.assertEqual(tracker.effective_direction, fallback)
            self.assertEqual(hd.pack_colors("1", "2", tracker.effective_direction),
                             "HCOLORS,1,2,%d" % fallback)
        for invalid in (None, True, "1", -1, 2):
            with self.assertRaises(ValueError):
                hd.DirectionTracker(invalid)
            with self.assertRaises(ValueError):
                hd.pack_colors("1", "2", invalid)

    def test_enabled_task_both_directions_and_unknown(self):
        for step, suffix, first, second, fallback in [(10, "1", "3", "1", 0),
                                                      (-10, "0", "1", "3", 1),
                                                      (0, "1", "3", "1", 1),
                                                      (0, "0", "3", "1", 0)]:
            def item(code, x):
                return dict(code=code, cx=x, cy=240, x=x-15, y=225,
                            w=30, h=30, pixels=200, src="ai")
            frames = ([[item("1", 220 + step*i), item("3", 430 + step*i)]
                       for i in range(6)] if step else [[item(first, 320)]] * 5)
            frames += [[item(second, 320)]] * 5
            clock = [0]
            remaining = iter(frames)

            def detect(img, roi, pixels):
                clock[0] += 100
                return next(remaining, [])

            clf = types.SimpleNamespace(detect=detect, resolve_code=lambda img, obj: obj["code"])
            link = smoke._Link([])
            link.poll_command = lambda: "R" if clock[0] > 2000 else None
            with patch.multiple(smoke.config, H_DIRECTION_PACKET_ENABLED=True,
                                H_DIRECTION_FALLBACK=fallback,
                                PLATE_DEBUG_SAVE=False, PLATE_LIGHT_ON=False,
                                PICK_STOP_HOLD_MS=100, H_MATERIAL_COLOR_FRAMES=2,
                                H_MATERIAL_CENTER_FRAMES=2, H_MATERIAL_TARGET_Y=240,
                                H_MATERIAL_MAX_MOVES=1), patch.object(smoke.tasks, "ticks_ms", lambda: clock[0]):
                result = smoke.tasks.task_h_material(smoke.FakeCam(1), clf, None,
                             p=smoke.P_ID, ctl=smoke.tasks.TaskCtl(link))
            self.assertEqual(result, "HDISC,320,240,0,%s,%s,%s" % (first, second, suffix))
            self.assertIn("HCOLORS,%s,%s,%s" % (first, second, suffix), link.sent)


if __name__ == "__main__":
    unittest.main()
