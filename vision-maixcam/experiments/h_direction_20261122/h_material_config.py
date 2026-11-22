"""Persistent H overrides shared by MaixCAM and the desktop tuning tool."""

import ast
import json
import math

PATH = "/root/yyb_mover_data/h_material.json"
BOOLEANS = ("mode", "invert_move")
RANGES = {
    "target_y": (0, 959, False),  # 允许画面外的虚拟目标，用于 one-shot 距离偏移
    "tol_y": (1, 120, False),
    "gain": (0.01, 2, False),
    "step_max": (0, 120, True),
    "max_moves": (0, 20, True),
    "settle_ms": (0, 10000, True),
    "timeout_ms": (500, 20000, True),
    "color_frames": (1, 30, True),
    "center_frames": (1, 30, True),
}
KEYS = (*BOOLEANS, *RANGES)


def validate(values):
    if not isinstance(values, dict) or set(values) != set(KEYS):
        raise ValueError("H 配置字段有遗漏或未知字段")
    for key in BOOLEANS:
        if type(values[key]) is not bool:
            raise ValueError("%s 必须是 true 或 false" % key)
    for key, (low, high, integer) in RANGES.items():
        value = values[key]
        if (type(value) not in (int, float) or not low <= value <= high
                or not math.isfinite(value) or (integer and type(value) is not int)):
            raise ValueError("%s 必须是 %s 至 %s 的%s" %
                             (key, low, high, "整数" if integer else "数字"))
    return {key: values[key] for key in KEYS}


def defaults(config):
    return validate({key: getattr(config, "H_MATERIAL_" + key.upper()) for key in KEYS})


def defaults_from_source(source):
    """Extract only the expected assignments from config.py without importing it."""
    names = {"H_MATERIAL_" + key.upper(): key for key in KEYS}
    height_node = None
    nodes = {}
    for node in ast.parse(source).body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    if target.id == "H":
                        height_node = node.value
                    elif target.id in names:
                        nodes[names[target.id]] = node.value
                elif isinstance(target, ast.Tuple) and isinstance(node.value, ast.Tuple):
                    for item, value in zip(target.elts, node.value.elts):
                        if isinstance(item, ast.Name) and item.id == "H":
                            height_node = value
    values = {}
    for key, node in nodes.items():
        if (key == "target_y" and isinstance(node, ast.BinOp)
                and isinstance(node.op, ast.FloorDiv) and isinstance(node.left, ast.Name)
                and node.left.id == "H" and isinstance(node.right, ast.Constant)
                and node.right.value == 2 and height_node is not None):
            values[key] = ast.literal_eval(height_node) // 2
        else:
            values[key] = ast.literal_eval(node)
    return validate(values)


def read(base, path=PATH):
    try:
        with open(path, encoding="utf-8-sig") as stream:
            data = json.load(stream)
    except FileNotFoundError:
        return base
    if not isinstance(data, dict):
        raise ValueError("H 配置必须是 JSON 对象")
    return validate({**base, **data})
