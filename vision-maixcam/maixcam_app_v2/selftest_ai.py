# -*- coding: utf-8 -*-
"""selftest_ai.py —— AI 判色融合逻辑的本地自检（PC 上跑，不进发布包）

覆盖：
  1) 纯 CV（ai=None）行为不变
  2) AI 判色覆盖阈值（这就是"抗色差"要的效果）
  3) AI 不可用 / 低置信度 / 抛异常 → 自动退回阈值
  4) AI.classify_top1 正确取 top1（修掉「把 list 当索引」的 bug）

用法（PC，无需真机）：
    python selftest_ai.py
"""

import sys
import types

# ----------------------------------------------------------------------
# 造假 maix 包（只做导入与最小行为，不依赖真机）
# ----------------------------------------------------------------------
maix = types.ModuleType("maix")
image = types.ModuleType("maix.image")


class _Format:
    FMT_RGB888 = 0
    FMT_BGR888 = 1


class _Fit:
    FIT_FILL, FIT_CONTAIN, FIT_COVER = 0, 1, 2


image.Format = _Format
image.Fit = _Fit
image.COLOR_RED = 0xFF0000
image.COLOR_GREEN = 0x00FF00
image.COLOR_BLUE = 0x0000FF
image.COLOR_YELLOW = 0xFFFF00
image.Image = object
maix.image = image
sys.modules["maix"] = maix
sys.modules["maix.image"] = image


class FakeBlob:
    def __init__(self, code, x, y, w, h, pixels):
        self._c, self._x, self._y, self._w, self._h, self._p = code, x, y, w, h, pixels

    def code(self):
        return self._c

    def x(self):
        return self._x

    def y(self):
        return self._y

    def w(self):
        return self._w

    def h(self):
        return self._h

    def pixels(self):
        return self._p

    def cxf(self):
        return self._x + self._w / 2.0

    def cyf(self):
        return self._y + self._h / 2.0

    def roundness(self):
        return 0.9


class FakeImage:
    """只实现 ColorClassifier 用到的东西：find_blobs / width / height / crop"""

    def __init__(self, blobs):
        self._blobs = blobs

    def find_blobs(self, *a, **kw):
        return self._blobs

    def width(self):
        return 640

    def height(self):
        return 480

    def crop(self, x, y, w, h):
        return FakeImage([])


import config                       # noqa: E402
import vision                       # noqa: E402
import ai_model                     # noqa: E402

# 阈值位序：codes = ["1","2","3","4"] → bit0=red bit1=yellow bit2=blue bit3=green
CODE_RED, CODE_YELLOW, CODE_BLUE, CODE_GREEN = 1, 2, 4, 8

FAILED = []


def check(name, cond, extra=""):
    print("  [%s] %s %s" % ("OK" if cond else "!!", name, extra))
    if not cond:
        FAILED.append(name)


class FakeAI:
    """假 AI：ready / classify_label 行为可编排

    code  : 期望判出的颜色码（"1"~"4"）；None = 低置信度（prob 0.30）
    label : 直接指定判出的类别（用于测 other/background 否决）
    """

    def __init__(self, code=None, ready=True, boom=False, prob=0.93, label=None):
        self.code = code
        self._ready = ready
        self.boom = boom
        self.prob = prob
        self.label = label
        self.calls = 0

    def ready(self):
        return self._ready

    def classify_label(self, img, roi=None):
        self.calls += 1
        if self.boom:
            raise RuntimeError("boom")
        if self.label is not None:
            lab = self.label
            idx = config.AI_LABELS.index(lab) if lab in config.AI_LABELS else 0
            return lab, self.prob, idx
        if self.code is None:
            return "red", 0.30, 4          # 低置信度 → 应退回阈值
        lab = [k for k, v in config.AI_CLASS_TO_COLOR.items() if v == self.code][0]
        return lab, self.prob, config.AI_LABELS.index(lab)


def main():
    # ------------------------------------------------------------------
    print("[1] 纯 CV（ai=None）行为不变")
    img = FakeImage([FakeBlob(CODE_YELLOW, 10, 10, 40, 40, 900),
                     FakeBlob(CODE_RED, 100, 100, 30, 30, 400)])
    clf = vision.ColorClassifier(ai=None)
    it = clf.find_color(img, vision.COLOR_RED)
    check("找到红色块", it is not None and it["cx"] == 115, str(it and it["cx"]))
    check("src=cv", it and it.get("src") == "cv")
    check("找不到紫色之外的颜色码→None", clf.find_color(img, "9") is None)

    # ------------------------------------------------------------------
    print("[2] AI 判色覆盖阈值（抗色差）")
    # 阈值把红块误判成黄（bit1）；AI 说是红 → 应当能按红色找到
    img2 = FakeImage([FakeBlob(CODE_YELLOW, 100, 100, 30, 30, 400)])
    ai = FakeAI(code=vision.COLOR_RED)
    clf2 = vision.ColorClassifier(ai=ai)
    it = clf2.find_color(img2, vision.COLOR_RED)
    check("AI 纠正后能找到红色", it is not None)
    check("item.code 被 AI 改写", it and it["code"] == vision.COLOR_RED)
    check("item.ai_code 记录 AI 结果", it and it["ai_code"] == vision.COLOR_RED)
    check("src=ai", it and it["src"] == "ai")
    calls_after_first = ai.calls
    check("AI 只判了 1 次（命中即返回）", calls_after_first == 1, "calls=%d" % calls_after_first)
    check("按旧阈值色(黄)已找不到", clf2.find_color(img2, vision.COLOR_YELLOW) is None)

    # ------------------------------------------------------------------
    print("[3] AI 退化时自动回退阈值")
    cases = [
        ("模型没加载成功(ready=False)", FakeAI(code=vision.COLOR_RED, ready=False)),
        ("低置信度/判成背景(返回 None)", FakeAI(code=None)),
        ("AI 抛异常", FakeAI(boom=True)),
    ]
    for name, fake in cases:
        c = vision.ColorClassifier(ai=fake)
        it = c.find_color(img, vision.COLOR_RED)
        check("回退后仍能按阈值找到红色: " + name, it is not None)
        check("src=cv: " + name, it and it["src"] == "cv")

    # ------------------------------------------------------------------
    print("[4] AI.classify_top1 取 top1（修「把 list 当索引」的 bug）")
    nn_mod = types.ModuleType("maix.nn")

    class FakeClassifier:
        labels = ["background", "blue", "green", "other", "red", "yellow"]

        def __init__(self, model="", dual_buff=True):
            self.model = model

        def classify(self, img, softmax=True, fit=None):
            # 官方返回：[(idx, prob), ...] 按概率降序
            return [(5, 0.93), (2, 0.04), (0, 0.03)]

    nn_mod.Classifier = FakeClassifier
    nn_mod.YOLO11 = object
    maix.nn = nn_mod
    sys.modules["maix.nn"] = nn_mod

    old_use, old_model = config.USE_AI, config.AI_MODEL
    config.USE_AI, config.AI_MODEL = True, "/fake/cls11n.mud"
    try:
        a = ai_model.AI()
        check("AI 加载成功", a.ready())
        check("labels 来自模型", a.labels == FakeClassifier.labels, str(a.labels))
        idx, prob = a.classify_top1(FakeImage([]))
        check("classify_top1 == (5, 0.93)", idx == 5 and abs(prob - 0.93) < 1e-6,
              "got %s" % ((idx, prob),))
        check("classify() 返回 int（不是 list）", isinstance(a.classify(FakeImage([])), int))
        check("color_code_of → 黄(2)", a.color_code_of(FakeImage([])) == vision.COLOR_YELLOW)
        check("min_prob 过滤生效",
              a.color_code_of(FakeImage([]), min_prob=0.99) is None)
    finally:
        config.USE_AI, config.AI_MODEL = old_use, old_model

    # ------------------------------------------------------------------
    print("[5] AI_PREFER_CV=True 时以 CV 为准（A/B 排查用）")
    old = config.AI_PREFER_CV
    config.AI_PREFER_CV = True
    try:
        ai = FakeAI(code=vision.COLOR_RED)
        c = vision.ColorClassifier(ai=ai)
        it = c.find_color(img, vision.COLOR_YELLOW)
        check("按阈值色(黄)能找到", it is not None)
        check("src=cv", it and it["src"] == "cv")
        check("AI 一次都没被调用", ai.calls == 0)
    finally:
        config.AI_PREFER_CV = old

    # ------------------------------------------------------------------
    print("[6] AI 判成 other/background：严格丢弃 vs 宽松回退（AI_VETO_OTHER）")
    img3 = FakeImage([FakeBlob(CODE_RED, 100, 100, 30, 30, 400)])   # 阈值说是红
    old_veto = config.AI_VETO_OTHER
    try:
        config.AI_VETO_OTHER = True
        c = vision.ColorClassifier(ai=FakeAI(label="other", prob=0.95))
        it = c.find_color(img3, vision.COLOR_RED)
        check("严格模式：AI 说 other → 红色也找不到（丢弃候选）", it is None)

        config.AI_VETO_OTHER = False
        c = vision.ColorClassifier(ai=FakeAI(label="other", prob=0.95))
        it = c.find_color(img3, vision.COLOR_RED)
        check("宽松模式：AI 说 other → 退回阈值仍能命中红色", it is not None)
        check("来源标记为 cv", it and it["src"] == "cv")
    finally:
        config.AI_VETO_OTHER = old_veto

    print()
    if FAILED:
        print("[FAIL] %d 项未通过: %s" % (len(FAILED), FAILED))
        return 1
    print("[ALL PASS] AI 判色融合自检通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())