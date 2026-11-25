"""Find the printed 2 over the entire frame, independently of circle proposals."""
import time
from maix import image
import config

_cache = None


def _templates():
    global _cache
    if _cache is None:
        base = image.load(config.DISC_TEMPLATE_PATH).to_format(image.Format.FMT_GRAYSCALE)
        scales = [float(s) for s in config.DISC_TEMPLATE_SCALES]
        scales.sort(key=lambda s: abs(s-1.0))
        _cache = []
        for s in scales:
            tpl=base.resize(max(4,round(base.width()*s)),max(4,round(base.height()*s)))
            small=tpl.resize(max(4,round(tpl.width()/2)),max(4,round(tpl.height()/2)))
            _cache.append((s,tpl,small))
        print('[DISC-FULL-TEMPLATE] loaded %dx%d; full-frame search; scales=%s' % (base.width(),base.height(),scales), flush=True)
    return _cache


def locate(img, meta=None):
    started = time.monotonic()
    gray = img.to_format(image.Format.FMT_GRAYSCALE)
    small=gray.resize(max(1,gray.width()//2),max(1,gray.height()//2))
    threshold = float(config.DISC_TEMPLATE_THRESHOLD)
    for scale, tpl, coarse_tpl in _templates():
        if tpl.width()>gray.width() or tpl.height()>gray.height():
            continue
        tw,th=min(128,small.width()),min(104,small.height())
        if tw<coarse_tpl.width() or th<coarse_tpl.height():
            continue
        sx,sy=max(1,tw-coarse_tpl.width()),max(1,th-coarse_tpl.height())
        xs=list(range(0,small.width()-tw+1,sx))
        ys=list(range(0,small.height()-th+1,sy))
        if xs[-1]!=small.width()-tw:xs.append(small.width()-tw)
        if ys[-1]!=small.height()-th:ys.append(small.height()-th)
        hit=None
        for oy in ys:
            for ox in xs:
                r=small.crop(ox,oy,tw,th).find_template(coarse_tpl,threshold,step=2)
                if r:
                    hit=(round((ox+r[0])*gray.width()/small.width()),round((oy+r[1])*gray.height()/small.height()),tpl.width(),tpl.height())
                    break
            if hit:break
        if not hit:continue
        x,y,w,h=hit
        # Refine the full-image step=2 hit at pixel resolution in a small window.
        rx,ry=max(0,x-6),max(0,y-6)
        rw,rh=min(gray.width()-rx,w+12),min(gray.height()-ry,h+12)
        fine=gray.crop(rx,ry,rw,rh).find_template(tpl,threshold,step=1)
        if not fine:continue
        x,y,w,h=rx+int(fine[0]),ry+int(fine[1]),int(fine[2]),int(fine[3])
        cx,cy=int(round(x+w/2)),int(round(y+h/2))
        radius=max(1,int(round(h*.95)))
        if meta is not None:
            meta.update(src='template-full',center_src='tpl2-full',tpl=[x,y,w,h],tpl_scale=scale,
                        tpl_roi=[0,0,gray.width(),gray.height()],coarse=[cx,cy,radius],
                        radius_src='template-size-estimate',ms=round((time.monotonic()-started)*1000,2))
        return cx,cy,radius
    if meta is not None:
        meta['full_template_miss']=True
        meta['full_template_ms']=round((time.monotonic()-started)*1000,2)
    return None
