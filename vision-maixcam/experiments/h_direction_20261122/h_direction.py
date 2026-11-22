"""Optional H direction protocol. Raw image rightward motion means CCW."""

UNKNOWN = "UNKNOWN"
CW = "CW"
CCW = "CCW"


class DirectionTracker:
    """Lock direction after three displacement windows agree, before car movement."""

    def __init__(self):
        self.direction = UNKNOWN
        self.anchors = {}
        self.vote = 0
        self.count = 0

    def reset_motion(self):
        self.anchors.clear()
        self.vote = 0
        self.count = 0

    def update(self, candidates, now, allow_motion=True):
        if not allow_motion:
            self.reset_motion()
            return self.direction
        if self.direction != UNKNOWN:
            return self.direction
        points = {c["code"]: c["raw"] for c in candidates if c["pair_eligible"]}
        self.anchors = {k: v for k, v in self.anchors.items() if k in points}
        votes = []
        for code, (x, y) in points.items():
            anchor = self.anchors.get(code)
            if anchor is None:
                self.anchors[code] = (x, y, now)
                continue
            old_x, old_y, start = anchor
            elapsed = now - start
            if elapsed < 80:
                continue
            dx = x - old_x
            self.anchors[code] = (x, y, now)
            if elapsed > 400 or abs(dx) > 80 or abs(y - old_y) > 60:
                continue
            if abs(dx) >= 6:
                votes.append(1 if dx > 0 else -1)
        # One vote per frame, even when several materials move together.
        if not votes:
            return self.direction
        if min(votes) != max(votes):
            self.vote = self.count = 0
            return self.direction
        vote = votes[0]
        self.count = self.count + 1 if self.vote == vote else 1
        self.vote = vote
        if self.count >= 3:
            self.direction = CCW if vote > 0 else CW
        return self.direction

    def order_pair(self, candidates):
        if self.direction == UNKNOWN or len(candidates) != 2:
            return ()
        ordered = sorted(candidates, key=lambda c: c["raw"][0],
                         reverse=self.direction == CCW)
        if abs(ordered[0]["raw"][0] - ordered[1]["raw"][0]) < 6:
            return ()
        return tuple(c["code"] for c in ordered)


def pack_colors(first, second, direction):
    _validate(first, second, direction)
    return "HCOLORS,%s,%s,%s" % (first, second, direction)


def pack_disc(x, y, dx, first, second, direction):
    _validate(first, second, direction)
    return "HDISC,%d,%d,%d,%s,%s,%s" % (int(x), int(y), int(dx), first, second, direction)


def _validate(first, second, direction):
    if str(first) not in ("1", "2", "3", "4") or str(second) not in ("1", "2", "3", "4"):
        raise ValueError("invalid H color")
    if str(first) == str(second) or direction not in (UNKNOWN, CW, CCW):
        raise ValueError("invalid H pair or direction")
