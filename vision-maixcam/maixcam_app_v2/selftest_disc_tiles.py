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


def verify(img, cx, cy, meta=None, max_shift=None, template_paths=None):
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
     patch.object(config, 'DISC_DASHED_HOUGH_ON', False), \
     patch.object(config, 'DISC_DASHED_RADIAL_ON', False), \
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


plans = len(vision._hough_rois(roi)) + 1
with patch.object(config, 'DISC_CENTER_ON', False), \
     patch.object(config, 'DISC_BLACK_CENTER_ON', False), \
     patch.object(config, 'DISC_TEMPLATE_ON', True), \
     patch.object(config, 'DISC_DASHED_HOUGH_ON', True), \
     patch.object(config, 'DISC_DASHED_RADIAL_ON', False), \
     patch.object(vision, 'verify_target2', verify), \
     patch.object(disc_template_locator, 'locate', return_value=None):
    frame = Frame([[bad]] * plans + [[good]])
    result, meta = detect(frame)
    assert result == (420, 240, 60), (result, meta)
    assert meta['src'] == 'dashed-hough'
    assert len(frame.calls) == plans + 1
    assert meta['dashed_hough_batches'][-1]['count'] == 1
    print('PASS: low-threshold dashed-circle Hough fallback')


class RadialFrame:
    def __init__(self, center=(320, 200), radii=(34, 49, 64, 79)):
        self.center = center
        self.radii = radii

    def width(self): return 640
    def height(self): return 480
    def to_format(self, _format): return self
    def find_circles(self, **kwargs): return []

    def get_pixel(self, x, y):
        radius = ((x - self.center[0]) ** 2 + (y - self.center[1]) ** 2) ** 0.5
        return 0 if any(abs(radius - item) <= 1.2 for item in self.radii) else 255


def verify_radial(img, cx, cy, meta=None, max_shift=None, template_paths=None):
    return abs(cx - 320) <= 2 and abs(cy - 200) <= 2


with patch.object(config, 'DISC_CENTER_ON', False), \
     patch.object(config, 'DISC_BLACK_CENTER_ON', False), \
     patch.object(config, 'DISC_TEMPLATE_ON', True), \
     patch.object(config, 'DISC_DASHED_HOUGH_ON', False), \
     patch.object(config, 'DISC_DASHED_RADIAL_ON', True), \
     patch.object(maix.image, 'Format', gray_format, create=True), \
     patch.object(vision, 'verify_target2', verify_radial), \
     patch.object(disc_template_locator, 'locate', return_value=None):
    result, meta = detect(RadialFrame())
    assert result is not None and abs(result[0] - 320) <= 6 \
        and abs(result[1] - 200) <= 6, (result, meta)
    assert meta['src'].startswith('dashed-radial')
    print('PASS: interrupted concentric-ring radial fallback')


def reject_template(*args, **kwargs):
    return False


with patch.object(config, 'DISC_CENTER_ON', True), \
     patch.object(config, 'DISC_BLACK_CENTER_ON', True), \
     patch.object(config, 'DISC_TEMPLATE_ON', True), \
     patch.object(config, 'DISC_DASHED_HOUGH_ON', False), \
     patch.object(config, 'DISC_DASHED_RADIAL_ON', False), \
     patch.object(config, 'DISC_GEOMETRIC_CONSENSUS_ON', True), \
     patch.object(vision, 'find_disc_center_inner',
                  return_value=(320, 200, 38)), \
     patch.object(vision, 'find_disc_center_black',
                  return_value=(323, 201, 61)), \
     patch.object(vision, 'verify_target2', reject_template), \
     patch.object(disc_template_locator, 'locate', return_value=None):
    result, meta = detect(Frame([[]] * 20))
    assert result is not None and abs(result[0] - 321) <= 3 \
        and abs(result[1] - 200) <= 3, (result, meta)
    assert meta['src'] == 'geometric-consensus'
    assert len(meta['geometric_consensus']['sources']) >= 2
    print('PASS: white-core and black-ring geometric consensus')


paths = vision.disc_template_paths('D')
assert len(paths) == 1 and paths[0].endswith('target2.png')
assert vision.disc_template_paths('E') == paths
print('PASS: D/E share one target2 template')

print('ALL PASS')
