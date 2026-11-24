"""D/E bounded Hough plans must pass content validation before early return."""
import sys
import types
from unittest.mock import patch

maix = types.ModuleType('maix')
maix.image = types.ModuleType('maix.image')
sys.modules.update({'maix': maix, 'maix.image': maix.image})
import config
import disc_template_locator
import vision


class Circle:
    def __init__(self, x, y, r=60): self.values = (x, y, r)
    def x(self): return self.values[0]
    def y(self): return self.values[1]
    def r(self): return self.values[2]
    def magnitude(self): return 100


class Frame:
    def __init__(self, batches):
        self.batches = batches
        self.calls = []
    def width(self): return 640
    def height(self): return 480
    def find_circles(self, **kwargs):
        self.calls.append(kwargs)
        x, y, w, h = kwargs['roi']
        assert 0 < w <= 340 and 0 < h <= 320, kwargs
        batch = self.batches[len(self.calls)-1]
        if isinstance(batch, Exception): raise batch
        return batch


roi = [96, 0, 448, 400]
bad, good = Circle(200, 120), Circle(420, 240)
template_calls = []


def verify(img, cx, cy, meta=None, max_shift=None):
    template_calls.append((cx, cy, max_shift))
    if cx == good.x():
        meta['tpl'] = [cx-30, cy-40, 60, 80]
        meta['tpl_scale'] = 1.0
        return True
    return False


def detect(frame, refine=False, ring=False):
    meta = {}
    result = vision.find_disc_center(frame, roi, refine=refine, ring_fallback=ring, meta=meta)
    return result, meta


with patch.object(config, 'DISC_CENTER_ON', False), \
     patch.object(config, 'DISC_BLACK_CENTER_ON', False), \
     patch.object(config, 'DISC_TEMPLATE_ON', True), \
     patch.object(vision, 'verify_target2', verify), \
     patch.object(disc_template_locator, 'locate', return_value=None) as locate:
    frame = Frame([[bad], [good]])
    result, meta = detect(frame)
    assert result == (420, 240, 60), (result, meta)
    assert len(frame.calls) == 2 and meta['hough_tries'] == 2
    assert [b['accepted'] for b in meta['hough_batches']] == [False, True]
    assert 'tpl_reject' not in meta
    print('PASS: first tile false circle, next tile true target')

    template_calls.clear()
    frame = Frame([[good]])
    result, meta = detect(frame)
    assert result == (420, 240, 60) and len(frame.calls) == len(template_calls) == 1
    print('PASS: accepted center tile keeps one-plan fast path')

    plans = len(vision._hough_rois(roi)) + 1
    template_calls.clear()
    frame = Frame([[bad]]*(plans-1) + [[good]])
    result, meta = detect(frame)
    assert result == (420, 240, 60) and len(frame.calls) == plans
    assert frame.calls[-1]['threshold'] == config.DISC_THRESHOLD
    assert frame.calls[-1]['x_stride'] == 2 and frame.calls[-1]['r_step'] == 2
    assert len(template_calls) == 2, template_calls
    print('PASS: legacy plan remains reachable; duplicate failed verification cached per frame')

    template_calls.clear()
    frame = Frame([[bad]]*plans)
    with patch.object(vision, 'refine_disc_center', lambda img, cx, cy, r, meta: (cx, cy, r)):
        result, meta = detect(frame, refine=True)
    assert result is None and len(frame.calls) == plans and len(template_calls) == 1
    print('PASS: refinement/cluster retries reuse same-frame verification; false-only scene rejected')

    template_calls.clear()
    result, meta = detect(Frame([[good]]))
    result, meta = detect(Frame([[good]]))
    assert len(template_calls) == 2
    print('PASS: template cache does not leak across frames')

    frame = Frame([MemoryError('fast buffer test'), [good]])
    result, meta = detect(frame)
    assert result == (420, 240, 60) and len(frame.calls) == 2
    assert 'fast buffer test' in meta['circles_error']
    print('PASS: bounded-plan exception preserved in metadata and next plan tried')

    with patch.object(vision, 'find_ring_center', lambda *a, **k: (420, 240, 60)):
        result, meta = detect(Frame([[]]*plans), ring=True)
    assert result == (420, 240, 60) and meta['src'] == 'ring'
    print('PASS: final ring fallback still available')

    locate.side_effect = OSError('missing template')
    result, meta = detect(Frame([[good]]))
    assert result == (420, 240, 60) and 'missing template' in meta['tpl_error']
    print('PASS: full-template exception leaves Hough fallback available')


class TemplateFrame:
    def __init__(self, matches, width=640, height=480, origin=(0, 0)):
        self.matches = matches
        self.size = (width, height)
        self.origin = origin

    def width(self):
        return self.size[0]

    def height(self):
        return self.size[1]

    def to_format(self, _format):
        return self

    def resize(self, width, height):
        return TemplateFrame(self.matches, width, height)

    def crop(self, x, y, width, height):
        return TemplateFrame(self.matches, width, height, (x, y))

    def find_template(self, _tpl, _threshold, step):
        return self.matches.get((step, *self.origin))


gray_format = types.SimpleNamespace(FMT_GRAYSCALE=0)
templates = [(1.0, TemplateFrame({}, 60, 80), TemplateFrame({}, 30, 40))]
coarse_first = (20, 20, 30, 40)
coarse_second = (32, 50, 30, 40)
matches = {
    (2, 0, 0): coarse_first,
    (2, 98, 0): coarse_second,
    (1, 254, 94): (6, 6, 60, 80),
}
with patch.object(maix.image, 'Format', gray_format, create=True), \
     patch.object(disc_template_locator, '_templates', return_value=templates):
    meta = {}
    assert disc_template_locator.locate(TemplateFrame(matches), meta) == (290, 140, 76)
    assert meta['tpl'] == [260, 100, 60, 80]
    print('PASS: rejected first coarse hit does not skip valid same-scale tile')

    matches[(1, 34, 34)] = (6, 6, 60, 80)
    assert disc_template_locator.locate(TemplateFrame(matches)) == (70, 80, 76)
    assert disc_template_locator.locate(TemplateFrame(matches), roi=roi) == (290, 140, 76)
    assert disc_template_locator.locate(
        TemplateFrame(matches), prefer=(295, 142), max_jump=50) == (290, 140, 76)
    assert disc_template_locator.locate(
        TemplateFrame(matches), prefer=(450, 440), max_jump=50) is None
    print('PASS: work-area and tracking constraints reject unrelated template hits')

print('ALL PASS')
