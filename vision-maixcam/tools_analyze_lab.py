# -*- coding: utf-8 -*-
"""从数据集统计 LAB 阈值 v3

- 颜色类：用「最饱和的 top 8% 像素」代表物料，先按 (A,B) 的 MAD 剔除离群图，再取 p2/p98
- 灰度道路：统计低饱和像素占比，占比高的图视为「灰色路面」，给出 L 范围
口径：MaixPy/OpenMV  L 0~100, A/B -128~127
"""
import glob
import os
import numpy as np
import cv2

D = '/home/linbing/maixcam-migration/_from_device/dataset'
COLORS = ['red', 'yellow', 'blue', 'green']
UI_TOP, UI_BOTTOM = 60, 90


def load_lab(path):
    img = cv2.imread(path)
    if img is None:
        return None
    h = img.shape[0]
    roi = img[UI_TOP:h - UI_BOTTOM, :]
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB).astype(np.float32)
    return (lab[..., 0] * 100.0 / 255.0, lab[..., 1] - 128.0, lab[..., 2] - 128.0)


def dominant(path, top_ratio=0.08):
    r = load_lab(path)
    if r is None:
        return None
    L, A, B = r
    sat = np.hypot(A, B)
    valid = (L > 12) & (L < 92)
    if valid.sum() < 500:
        return None
    thr = max(25.0, float(np.percentile(sat[valid], 100 * (1 - top_ratio))))
    m = valid & (sat >= thr)
    if m.sum() < 200:
        return None
    return (float(np.median(L[m])), float(np.median(A[m])), float(np.median(B[m])))


def gray_stats(path):
    """返回 (低饱和像素占比, 这些像素的 L 中位数, 全图 L 中位数)"""
    r = load_lab(path)
    if r is None:
        return None
    L, A, B = r
    sat = np.hypot(A, B)
    gray = (sat < 12) & (L > 10) & (L < 92)
    frac = gray.mean()
    if gray.sum() < 1000:
        return (frac, None, float(np.median(L)))
    return (frac, float(np.median(L[gray])), float(np.median(L)))


def robust(rows, k=3.0):
    """按 (A,B) 的 MAD 剔除离群图"""
    A = np.array([r[1] for r in rows]); B = np.array([r[2] for r in rows])
    ma, mb = np.median(A), np.median(B)
    mad_a = np.median(np.abs(A - ma)) or 1.0
    mad_b = np.median(np.abs(B - mb)) or 1.0
    keep = (np.abs(A - ma) < k * 1.4826 * mad_a) & (np.abs(B - mb) < k * 1.4826 * mad_b)
    return [r for r, kp in zip(rows, keep) if kp], int((~keep).sum())


def main():
    print('%-8s %5s %4s | %-20s %-22s %-22s' %
          ('类别', 'n', 'out', 'L p2/p50/p98', 'A p2/p50/p98', 'B p2/p50/p98'))
    print('-' * 104)
    stats = {}
    for c in COLORS:
        paths = sorted(glob.glob(os.path.join(D, c, '*.jpg')))
        rows = [r for r in (dominant(p) for p in paths) if r]
        kept, drop = robust(rows)
        L = np.array([r[0] for r in kept]); A = np.array([r[1] for r in kept]); B = np.array([r[2] for r in kept])
        stats[c] = (L, A, B)
        print('%-8s %5d %4d | %5.1f /%5.1f /%5.1f  %6.1f /%6.1f /%6.1f  %6.1f /%6.1f /%6.1f'
              % (c, len(rows), drop,
                 np.percentile(L, 2), np.median(L), np.percentile(L, 98),
                 np.percentile(A, 2), np.median(A), np.percentile(A, 98),
                 np.percentile(B, 2), np.median(B), np.percentile(B, 98)))

    print('\n=== 建议阈值（p2~p98，L 留 8 余量，A/B 留 6，夹到合法范围）===')
    for c in COLORS:
        L, A, B = stats[c]
        cf = lambda v, lo, hi: int(round(max(lo, min(hi, v))))
        print('TH_%-7s = [%d, %d, %d, %d, %d, %d]' % (
            c.upper(),
            cf(np.percentile(L, 2) - 8, 0, 100), cf(np.percentile(L, 98) + 8, 0, 100),
            cf(np.percentile(A, 2) - 6, -128, 127), cf(np.percentile(A, 98) + 6, -128, 127),
            cf(np.percentile(B, 2) - 6, -128, 127), cf(np.percentile(B, 98) + 6, -128, 127)))

    print('\n=== other / background：低饱和（灰色）分析 ===')
    for c in ['other', 'background']:
        paths = sorted(glob.glob(os.path.join(D, c, '*.jpg')))
        rows = [r for r in (gray_stats(p) for p in paths) if r]
        fr = np.array([r[0] for r in rows])
        grayish = [r for r in rows if r[1] is not None and r[0] >= 0.5]
        print('%-11s n=%d  低饱和占比 p50/p90/p99 = %.2f / %.2f / %.2f  「>50%% 灰」图 %d 张'
              % (c, len(rows), np.median(fr), np.percentile(fr, 90), np.percentile(fr, 99), len(grayish)))
        if grayish:
            Lg = np.array([r[1] for r in grayish])
            print('            这些图的灰色像素 L  p5/p50/p95 = %.1f / %.1f / %.1f'
                  % (np.percentile(Lg, 5), np.median(Lg), np.percentile(Lg, 95)))


if __name__ == '__main__':
    main()
