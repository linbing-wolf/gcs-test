"""Synthetic photometric and arc-distribution tests; no camera or movement."""
import math
import sys
import types
import numpy as np

maix = types.ModuleType('maix')
maix.image = types.ModuleType('maix.image')
sys.modules['maix'] = maix
sys.modules['maix.image'] = maix.image
import config
import vision


class Frame:
    def __init__(self, gray): self.gray = gray
    def width(self): return self.gray.shape[1]
    def height(self): return self.gray.shape[0]
    def get_pixel(self, x, y): return [int(self.gray[y, x])]


def disk(inner=235, outer=150, middle=False, short=False):
    yy, xx = np.indices((480,640))
    rad = np.hypot(xx-310, yy+32)
    gray = np.where(rad<330,inner,outer).astype(np.uint8)
    gray[np.abs(rad-330)<2] = 80
    if middle:
        gray[220:340,275:345] = 60
    if short:
        angle = np.degrees(np.arctan2(yy+32, xx-310))
        gray[(angle<70)|(angle>105)] = 150
    return Frame(gray)


for label, frame, expected in [('clear',disk(),True),('middle-occluded',disk(middle=True),True),
                               ('dark-false-rim',disk(inner=145),False),
                               ('short-visible-rim',disk(short=True),False),
                               ('bright-flat-background',Frame(np.full((480,640),235,np.uint8)),False)]:
    meta = {}
    ratio, _, _ = vision.check_inside_bright(frame,310,-32,330,330,gray=frame.gray,meta=meta)
    passed = ratio >= config.PLATE_INSIDE_MIN_RATIO and meta.get('rim_check') != 'fail'
    assert passed == expected, (label,ratio,meta)
    print(label, 'PASS', round(ratio,3), meta)


def points(degrees):
    return [(310+330*math.cos(math.radians(t)), -32+330*math.sin(math.radians(t)))
            for t in degrees]


assert vision._arc_distribution_ok(points(list(range(15,65,2))+list(range(115,166,2))),310,-32,330,330)
assert not vision._arc_distribution_ok(points(range(85,101)),310,-32,330,330)
assert not vision._arc_distribution_ok(points(range(15,80)),310,-32,330,330)
print('two-side split arc accepted; short and one-side arcs rejected')
print('ALL PASS')
