# -*- coding: utf-8 -*-
"""INT8 axmodel 仿真精度评估（在 pulsar2:6.0 容器内跑）。

输入：val_bin/<name>.bin（U8 NHWC 224x224x3，由 eval_acc.py --dump-bin 生成）
      val_bin/labels.txt（<name> <class_idx>）
输出：argmax(logits) 与标签比对，给出 top-1 与混淆矩阵。
"""
import os, shutil, subprocess, sys
import numpy as np

ROOT = '/data'
CLASSES = ["background", "blue", "green", "other", "red", "yellow"]


def main():
    axmodel = os.path.abspath(sys.argv[1] if len(sys.argv) > 1
                              else 'out/cls11n_npu.axmodel')
    vb = os.path.join(ROOT, 'val_bin')
    items = []
    for line in open(os.path.join(vb, 'labels.txt')):
        name, ci = line.split()
        items.append((name, int(ci)))
    print('[i] %s / %d 张' % (os.path.basename(axmodel), len(items)))

    sim_in = os.path.join(ROOT, 'sim_inputs')
    sim_out = os.path.join(ROOT, 'sim_outputs')
    shutil.rmtree(sim_in, ignore_errors=True)
    shutil.rmtree(sim_out, ignore_errors=True)
    os.makedirs(sim_in); os.makedirs(sim_out)
    for i, (name, _) in enumerate(items):
        d = os.path.join(sim_in, str(i)); os.makedirs(d, exist_ok=True)
        shutil.copyfile(os.path.join(vb, name + '.bin'),
                        os.path.join(d, 'images.bin'))
    with open(os.path.join(ROOT, 'list.txt'), 'w') as fh:
        for i in range(len(items)):
            fh.write('%d\n' % i)

    r = subprocess.run(['pulsar2', 'run', '--model', axmodel,
                        '--input_dir', sim_in, '--output_dir', sim_out,
                        '--list', os.path.join(ROOT, 'list.txt')],
                       cwd=ROOT, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-1500:]); print(r.stderr[-3000:]); return 1
    print(r.stdout.strip().splitlines()[-1] if r.stdout.strip() else '')

    pred = np.zeros(len(items), dtype=int)
    logits_all = np.zeros((len(items), 6), dtype=np.float32)
    for i, (name, _) in enumerate(items):
        od = os.path.join(sim_out, str(i))
        bins = [x for x in sorted(os.listdir(od)) if x.endswith('.bin')]
        v = np.fromfile(os.path.join(od, bins[0]), dtype=np.float32)
        logits_all[i] = v
        pred[i] = int(np.argmax(v))

    y = np.array([c for _, c in items])
    acc = (pred == y).mean()
    print('\n=== INT8 %s : val top-1 = %.2f%% (%d/%d) ==='
          % (os.path.basename(axmodel), acc * 100, (pred == y).sum(), len(y)))
    cm = np.zeros((6, 6), dtype=int)
    for t, q in zip(y, pred):
        cm[t, q] += 1
    print('       ' + ' '.join('%10s' % c for c in CLASSES))
    for i, c in enumerate(CLASSES):
        print('%10s ' % c + ' '.join('%10d' % v for v in cm[i]))

    tag = os.path.basename(axmodel).replace('.axmodel', '')
    np.save(os.path.join(ROOT, 'int8_logits_%s.npy' % tag), logits_all)
    # 与 float logits 的一致性（cosine / argmax 一致率）
    f = os.path.join(ROOT, 'float_logits_val.npy')
    if os.path.exists(f):
        fl = np.load(f)
        cos = (fl * logits_all).sum(1) / (np.linalg.norm(fl, axis=1) *
                                          np.linalg.norm(logits_all, axis=1) + 1e-9)
        print('与 float logits: cosine 均值 %.4f (min %.4f)，argmax 一致率 %.2f%%'
              % (cos.mean(), cos.min(),
                 (fl.argmax(1) == logits_all.argmax(1)).mean() * 100))
    return 0


if __name__ == '__main__':
    sys.exit(main())