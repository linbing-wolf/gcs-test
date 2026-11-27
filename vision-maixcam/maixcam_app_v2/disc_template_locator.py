"""Find the printed 2 across frame tiles, independently of circle proposals."""
import math
import time
from maix import image
import config

_cache = {}


def _template_paths(template_paths=None):
    if template_paths is None:
        paths = (getattr(config, "DISC_TEMPLATE_PATH", ""),)
    elif isinstance(template_paths, (str, bytes)):
        paths = (template_paths,)
    else:
        paths = tuple(template_paths)
    out = []
    for path in paths:
        path = str(path or "")
        if path and path not in out:
            out.append(path)
    return tuple(out)


def _templates(path):
    if path not in _cache:
        base = image.load(path).to_format(image.Format.FMT_GRAYSCALE)
        scales = [float(s) for s in config.DISC_TEMPLATE_SCALES]
        scales.sort(key=lambda s: abs(s - 1.0))
        base_w, base_h = base.width(), base.height()
        templates = []
        for scale in scales:
            width = max(4, round(base_w * scale))
            height = max(4, round(base_h * scale))
            tpl = base.resize(width, height)
            coarse_tpl = tpl.resize(max(4, round(width / 2)), max(4, round(height / 2)))
            templates.append((scale, tpl, coarse_tpl))
        _cache[path] = templates
        print('[DISC-FULL-TEMPLATE] loaded %s %dx%d; scales=%s'
              % (path, base_w, base_h, scales), flush=True)
    return _cache[path]


def locate(img, meta=None, *, roi=None, prefer=None, max_jump=0,
           template_paths=None):
    """Return a match centered in the work area, with a tracking limit when set."""
    started = time.monotonic()
    gray = img.to_format(image.Format.FMT_GRAYSCALE)
    width, height = gray.width(), gray.height()
    small = gray.resize(max(1, width // 2), max(1, height // 2))
    small_w, small_h = small.width(), small.height()
    threshold = float(config.DISC_TEMPLATE_THRESHOLD)
    roi_x, roi_y, roi_w, roi_h = roi if roi is not None else (0, 0, width, height)
    errors = []
    for template_path in _template_paths(template_paths):
        try:
            templates = _templates(template_path)
        except Exception as exc:
            errors.append("%s:%s" % (template_path, exc))
            continue
        for scale, tpl, coarse_tpl in templates:
            tpl_w, tpl_h = tpl.width(), tpl.height()
            if tpl_w > width or tpl_h > height:
                continue
            tile_w, tile_h = min(128, small_w), min(104, small_h)
            if tile_w < coarse_tpl.width() or tile_h < coarse_tpl.height():
                continue
            step_x = max(1, tile_w - coarse_tpl.width())
            step_y = max(1, tile_h - coarse_tpl.height())
            end_x, end_y = small_w - tile_w, small_h - tile_h
            xs = list(range(0, end_x + 1, step_x))
            ys = list(range(0, end_y + 1, step_y))
            if xs[-1] != end_x:
                xs.append(end_x)
            if ys[-1] != end_y:
                ys.append(end_y)
            for oy in ys:
                for ox in xs:
                    coarse = small.crop(ox, oy, tile_w, tile_h).find_template(coarse_tpl, threshold, step=2)
                    if not coarse:
                        continue
                    # Refine at pixel resolution; a rejected match leaves later tiles available.
                    x = round((ox + coarse[0]) * width / small_w)
                    y = round((oy + coarse[1]) * height / small_h)
                    rx, ry = max(0, x - 6), max(0, y - 6)
                    rw, rh = min(width - rx, tpl_w + 12), min(height - ry, tpl_h + 12)
                    if rw < tpl_w or rh < tpl_h:
                        continue
                    fine = gray.crop(rx, ry, rw, rh).find_template(tpl, threshold, step=1)
                    if not fine:
                        continue
                    x, y = rx + int(fine[0]), ry + int(fine[1])
                    w, h = int(fine[2]), int(fine[3])
                    cx, cy = round(x + w / 2), round(y + h / 2)
                    if not (roi_x <= cx < roi_x + roi_w and roi_y <= cy < roi_y + roi_h):
                        continue
                    if prefer is not None and max_jump > 0 and math.hypot(cx - prefer[0], cy - prefer[1]) > max_jump:
                        continue
                    radius = max(1, round(h * .95))
                    if meta is not None:
                        meta.update(src='template-full', center_src='tpl2-full', tpl=[x, y, w, h],
                                    tpl_scale=scale, tpl_roi=[0, 0, width, height], coarse=[cx, cy, radius],
                                    radius_src='template-size-estimate', template_path=template_path,
                                    ms=round((time.monotonic() - started) * 1000, 2))
                    return cx, cy, radius
    if meta is not None:
        meta['full_template_miss']=True
        meta['full_template_ms']=round((time.monotonic()-started)*1000,2)
        if errors:
            meta['template_errors'] = errors
    return None
