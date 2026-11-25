"""Crop real pixels and compare grayscale NCC; no generated image content."""
import json
from pathlib import Path
import cv2
import numpy as np

root = Path(__file__).resolve().parent.parent
photos = root / '_from_device/new_field_other'
out = root / '_from_device/disc_template_20261030'
out.mkdir(exist_ok=True)
manifest = json.loads((photos/'manifest.json').read_text())
new = [Path(x['local']) for x in manifest[:10]]
old = [Path(x['local']) for x in manifest[10:]]
asset = root/'maixcam_app_v2/assets/target2.png'
previous = cv2.imread(str(asset), 0)
cv2.imwrite(str(out/'target2_previous.png'), previous)
seed = cv2.imread(str(new[0]), 0)[108:188, 400:460]

def match(img, tpl, region=None):
    x,y,w,h = region or (0,0,img.shape[1],img.shape[0])
    roi = img[y:y+h,x:x+w]
    best = (-1., None)
    for scale in [.7,.8,.9,1.,1.1]:
        t = cv2.resize(tpl, (round(tpl.shape[1]*scale), round(tpl.shape[0]*scale)))
        if t.shape[0]>roi.shape[0] or t.shape[1]>roi.shape[1]:
            continue
        _, score, _, loc = cv2.minMaxLoc(cv2.matchTemplate(roi,t,cv2.TM_CCOEFF_NORMED))
        if score>best[0]:
            best=(float(score), [x+loc[0],y+loc[1],t.shape[1],t.shape[0]])
    return best

candidates=[]
for path in new:
    img=cv2.imread(str(path),0)
    _,box=match(img,seed,(280,30,280,240))
    cx,cy=round(box[0]+box[2]/2),round(box[1]+box[3]/2)
    crop=img[cy-40:cy+40,cx-30:cx+30].copy()
    scores=[]
    for p in new:
        frame=cv2.imread(str(p),0)
        pos=match(frame,crop,(280,30,280,240))[0]
        neg=max(match(frame,crop,(50,30,230,240))[0],match(frame,crop,(550,30,90,240))[0])
        scores.append((pos,neg))
    quality=min(s[0] for s in scores)-max(s[1] for s in scores)
    candidates.append((quality,path,crop,scores,(cx,cy)))
quality,source,selected,scores,center=max(candidates,key=lambda x:x[0])
cv2.imwrite(str(out/'target2_candidate.png'),selected)
report={'source':str(source),'crop_center':center,'size':[60,80],'selection_margin':quality,'new_samples':[], 'old_samples':[]}
for i,p in enumerate(new):
    frame=cv2.imread(str(p),0)
    report['new_samples'].append({'file':p.name,'old_score':match(frame,previous,(280,30,280,240))[0], 'new_score':scores[i][0], 'negative_1_3':scores[i][1]})
    score,box=match(frame,selected,(280,30,280,240))
    color=cv2.imread(str(p))
    x,y,w,h=box
    cv2.rectangle(color,(x,y),(x+w,y+h),(0,255,0),2)
    cv2.imwrite(str(out/(p.stem+'_match.jpg')),color)
for p in old:
    frame=cv2.imread(str(p),0)
    report['old_samples'].append({'file':p.name,'old_score':match(frame,previous)[0], 'new_score':match(frame,selected)[0]})
(out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
