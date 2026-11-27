"""A continuous recent-motion window for one material color."""
import math


class MotionWindow:
    def __init__(self, hold_ms, tolerance, min_frames, max_gap_ms):
        if hold_ms <= 0 or tolerance < 0 or min_frames < 2 or max_gap_ms <= 0:
            raise ValueError("invalid material motion settings")
        self.hold_ms = hold_ms
        self.tolerance = tolerance
        self.min_frames = min_frames
        self.max_gap_ms = max_gap_ms
        self.samples = []

    def update(self, now, x, y, trustworthy=True):
        if not trustworthy:
            self.samples.clear()
            return 2
        if self.samples and now - self.samples[-1][0] > self.max_gap_ms:
            self.samples.clear()
        self.samples.append((now, x, y))
        # Keep the closest observation at least hold_ms before this sample.
        while len(self.samples) > 1 and self.samples[1][0] <= now - self.hold_ms:
            self.samples.pop(0)
        if (len(self.samples) < self.min_frames
                or now - self.samples[0][0] < self.hold_ms):
            return 2
        xs = [sample[1] for sample in self.samples]
        ys = [sample[2] for sample in self.samples]
        span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        return 1 if span <= self.tolerance else 0

    def seed_stable(self, now, x, y, step_ms=50):
        """用上一任务刚确认的稳定位置预热窗口。

        首帧若已经移位，最新样本会立即把跨窗口位移拉大并判为运动。
        """
        if self.hold_ms <= 0:
            return
        step = max(1, min(int(step_ms), self.max_gap_ms))
        first = now - self.hold_ms
        self.samples = []
        t = first
        while t < now:
            self.samples.append((t, x, y))
            t += step
