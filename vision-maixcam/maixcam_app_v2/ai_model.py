# -*- coding: utf-8 -*-
"""
ai_model.py —— 可选的 AI 层（分类），加载失败自动退回经典 CV

用法（main.py 已经接好）：
  config.USE_AI = False（默认）→ 完全不加载模型，行为与纯 CV 版本一模一样
  config.USE_AI = True         → 创建 AI()，交给 vision.ColorClassifier

融合策略（`vision.ColorClassifier.find_color` 里实现）：
  经典 CV 负责「定位」：LAB 阈值找出候选色块，拿到位置/大小/中心
  AI   负责「判色」：把色块抠图（外扩 AI_LOCATE_MARGIN 像素）丢给分类模型
  以下情况自动退回阈值判色，绝不比纯 CV 更差：
    · 模型没加载成功 / 不是 classify 模式
    · AI 置信度 < config.AI_MIN_PROB
    · AI 判成 background / other（不是四色之一）
    · 抠图或推理抛异常
  config.AI_PREFER_CV = True 可以强制以 CV 为准（AI 只记录，便于对比排查）

注意 MaixPy 的返回格式（踩过的坑）：
  nn.Classifier.classify(img) → [(idx, prob), ...]，**按概率降序**的列表，
  官方示例写法是 `max_idx, max_prob = res[0]`。
  不能把整个 list 当索引用。
"""

import config

# 分类结果取第几名（0 = top1）
_TOPK = 0


class AI:
    def __init__(self):
        self.kind = None
        self.net = None
        self.labels = list(getattr(config, "AI_LABELS", []))
        self._ready = False
        if not getattr(config, "USE_AI", False):
            return
        try:
            from maix import nn
            self.kind = getattr(config, "AI_KIND", "classify")
            if self.kind == "detect":
                self.net = nn.YOLO11(model=config.AI_MODEL, dual_buff=True)
                net_labels = list(getattr(self.net, "labels", []) or [])
                self.labels = net_labels or self.labels
            else:
                # dual_buff=False：保证每次 classify 拿到的都是本帧结果。
                # dual_buff=True 时「未就绪」会返回 prob=0，会被误判成低置信度。
                self.net = nn.Classifier(model=config.AI_MODEL, dual_buff=False)
                net_labels = list(getattr(self.net, "labels", []) or [])
                if net_labels:
                    self.labels = net_labels
            self._ready = True
            print("[AI] 已加载: %s (kind=%s, labels=%s)"
                  % (config.AI_MODEL, self.kind, self.labels))
        except Exception as e:
            print("[AI] 加载失败，退回经典 CV: %s" % e)
            print("[AI] 需要这三个文件在同一目录，且 .mud 里的文件名与之一致：")
            print("     %s" % getattr(config, "AI_MODEL", "?"))
            print("     %s_npu.axmodel / %s_vnpu.axmodel" % (
                str(getattr(config, "AI_MODEL", "")).rsplit(".", 1)[0],
                str(getattr(config, "AI_MODEL", "")).rsplit(".", 1)[0]))
            print("     上传： scp cls11n*.axmodel cls11n.mud root@<板子IP>:/root/models/")
            self.net = None
            self._ready = False

    def ready(self):
        return self._ready and self.net is not None

    # ==================== 检测（保留原接口）====================
    def detect(self, img):
        if self.kind != "detect" or self.net is None:
            return []
        try:
            return self.net.detect(img,
                                   conf_th=getattr(config, "AI_CONF", 0.5),
                                   iou_th=getattr(config, "AI_IOU", 0.45))
        except Exception as e:
            print("[AI] detect 失败: %s" % e)
            return []

    # ==================== 分类 ====================
    def _crop(self, img, roi):
        """ROI [x,y,w,h] → 裁剪后的图；越界自动夹紧；非法返回 None"""
        if not roi:
            return img
        try:
            x, y, w, h = [int(v) for v in roi]
            iw, ih = img.width(), img.height()
            x = max(0, min(x, iw - 1))
            y = max(0, min(y, ih - 1))
            w = max(1, min(w, iw - x))
            h = max(1, min(h, ih - y))
            return img.crop(x, y, w, h)
        except Exception as e:
            print("[AI] crop 失败: %s" % e)
            return None

    def classify_top1(self, img, roi=None):
        """返回 (idx, prob)；失败返回 (None, 0.0)"""
        if self.kind != "classify" or self.net is None:
            return None, 0.0
        try:
            sub = self._crop(img, roi)
            if sub is None:
                return None, 0.0
            res = self.net.classify(sub)          # [(idx, prob), ...] 降序
            if not res:
                return None, 0.0
            idx, prob = res[_TOPK]
            return int(idx), float(prob)
        except Exception as e:
            print("[AI] classify 失败: %s" % e)
            return None, 0.0

    def classify(self, img, roi=None):
        """兼容旧接口：返回 top1 类别索引（int）或 None"""
        idx, _ = self.classify_top1(img, roi)
        return idx

    def classify_label(self, img, roi=None):
        """返回 (label, prob, idx)；失败 (None, 0.0, None)

        比 color_code_of 多给出「判成了什么」，上层据此区分：
          · 判成合法颜色 → 用它
          · 判成 background/other → 可丢弃该候选（AI_VETO_OTHER）
          · 置信度不足      → 退回阈值判色
        """
        idx, prob = self.classify_top1(img, roi)
        if idx is None:
            return None, 0.0, None
        label = self.labels[idx] if 0 <= idx < len(self.labels) else None
        return label, prob, idx

    def color_code_of(self, img, roi=None, min_prob=None):
        """AI 判色：返回颜色码 "1"/"2"/"3"/"4"；
        background / other / 置信度不足 / 异常 → None（调用方兜底 CV）
        """
        idx, prob = self.classify_top1(img, roi)
        if idx is None:
            return None
        if min_prob is None:
            min_prob = getattr(config, "AI_MIN_PROB", 0.0)
        if prob < min_prob:
            return None
        return color_code(idx, self.labels)


def color_code(idx, labels=None):
    """分类索引 → 颜色码（红1 黄2 蓝3 绿4）；background/other 或非法 → None"""
    if idx is None:
        return None
    try:
        if labels is None:
            labels = getattr(config, "AI_LABELS", [])
        m = getattr(config, "AI_CLASS_TO_COLOR", {})
        i = int(idx)
        if 0 <= i < len(labels):
            return m.get(labels[i])
    except Exception:
        pass
    return None