# -*- coding: utf-8 -*-
"""在 PC 上评估：float ONNX vs INT8 axmodel 的 val 准确率对比。

预处理对齐 MaixPy Classifier 默认的 Fit.FIT_COVER / ultralytics 分类 val 流程：
  短边 resize 到 224（双线性）+ 中心裁剪 224，RGB，/255
"""
import os, sys, numpy as np
from PIL import Image

W = '/home/linbing/maixcam-migration/_from_device/dataset_split'
CLASSES = ["background", "blue", "green", "other", "red", "yellow"]
SIZE = 224


def preprocess(path):
    im = Image.open(path).convert('RGB')
    w, h = im.size
    s = SIZE / min(w, h)
    im = im.resize((max(SIZE, round(w * s)), max(SIZE, round(h * s))), Image.BILINEAR)
    w, h = im.size
    left, top = (w - SIZE) // 2, (h - SIZE) // 2
    im = im.crop((left, top, left + SIZE, top + SIZE))
    a = np.asarray(im, dtype=np.float32) / 255.0
    return a  # HWC RGB float


def load_split(split):
    items = []
    for ci, c in enumerate(CLASSES):
        d = os.path.join(W, split, c)
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            if f.lower().endswith(('.jpg', '.jpeg', '.png')):
                items.append((os.path.join(d, f), ci))
    return items


def main():
    split = sys.argv[1] if len(sys.argv) > 1 else 'val'
    items = load_split(split)
    print('[%s] %d 张' % (split, len(items)))
    x = np.stack([preprocess(p) for p, _ in items])
    y = np.array([c for _, c in items])
    # 同时导出 U8 NHWC 原始 bin（给 pulsar2 run 仿真用）
    if '--dump-bin' in sys.argv:
        d = '/home/linbing/maixcam2_convert/val_bin'
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, 'labels.txt'), 'w') as fh:
            for (p, c), arr in zip(items, x):
                name = os.path.basename(p).rsplit('.', 1)[0]
                np.ascontiguousarray((arr * 255.0).round().astype(np.uint8)).tofile(
                    os.path.join(d, name + '.bin'))
                fh.write('%s %d\n' % (name, c))
        print('  -> bin 已导出到', d)

    import onnxruntime as ort
    sess = ort.InferenceSession('/home/linbing/maixcam2_convert/export.onnx',
                                providers=['CPUExecutionProvider'])
    iname = sess.get_inputs()[0].name
    logits = np.concatenate([
        sess.run(None, {iname: x[i:i + 1].transpose(0, 3, 1, 2).astype(np.float32)})[0]
        for i in range(len(x))], axis=0)
    print('  ONNX logits shape', logits.shape)
    pred = logits.argmax(1)
    acc = (pred == y).mean()
    print('  ONNX(float) top-1 = %.2f%%  (%d/%d)' % (acc * 100, (pred == y).sum(), len(y)))
    # 混淆矩阵
    cm = np.zeros((len(CLASSES), len(CLASSES)), dtype=int)
    for t, p in zip(y, pred):
        cm[t, p] += 1
    print('       ' + ' '.join('%10s' % c for c in CLASSES))
    for i, c in enumerate(CLASSES):
        print('%10s ' % c + ' '.join('%10d' % v for v in cm[i]))
    np.save('/home/linbing/maixcam2_convert/float_logits_%s.npy' % split, logits)


if __name__ == '__main__':
    main()