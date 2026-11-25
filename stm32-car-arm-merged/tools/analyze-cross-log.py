import math
import re
import sys

log = sys.argv[1]
legs = []
cur = None
prev = None
pat = re.compile(
    r"\[control\] phase=(\d+).*?yaw_cdeg=(-?\d+).*?distance_mm=(-?\d+)"
    r".*?target_mm=(\d+) travel=(\w+)")
for line in open(log, encoding="utf-8", errors="replace"):
    if line.startswith("=== LEG"):
        if cur:
            legs.append(cur)
        cur = {"name": line.strip(), "dx": 0.0, "dy": 0.0, "n": 0,
               "yaw0": None, "yaw1": None, "dist": 0}
        prev = None
        continue
    m = pat.search(line)
    if not m or cur is None:
        continue
    phase, yaw, dist, tgt, travel = m.groups()
    yaw = int(yaw)
    dist = int(dist)
    if cur["yaw0"] is None:
        cur["yaw0"] = yaw
    cur["yaw1"] = yaw
    if travel != "LEFT" or phase not in ("2",):
        prev = None
        continue
    if prev is None:
        prev = dist
        continue
    dd = dist - prev
    prev = dist
    if not dd:
        continue
    ang = math.radians(yaw / 100.0 + 90.0)
    cur["dx"] += dd * math.cos(ang)
    cur["dy"] += dd * math.sin(ang)
    cur["n"] += 1
    cur["dist"] = dist
if cur:
    legs.append(cur)

x = y = 0.0
for leg in legs:
    x += leg["dx"]
    y += leg["dy"]
    print(f"{leg['name']}: samples={leg['n']} encoder={leg['dist']} "
          f"leg_vec=({leg['dx']:.0f},{leg['dy']:.0f}) "
          f"yaw {leg['yaw0']/100:.1f}->{leg['yaw1']/100:.1f} "
          f"cum=({x:.0f},{y:.0f})")
print(f"net=({x:.0f},{y:.0f}) |net|={math.hypot(x,y):.0f}")
