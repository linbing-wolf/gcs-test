"""Extended H terminal packet with arrival state and visible material centers."""


def _color(value):
    text = str(value)
    if text not in ("1", "2", "3", "4"):
        raise ValueError("invalid H color")
    return text


def pack_h_disc2(arrival_stopped, positions):
    """Pack arrival state plus one coordinate tuple per visible color."""
    if type(arrival_stopped) is not int or arrival_stopped not in (0, 1):
        raise ValueError("arrival_stopped must be 0 or 1")
    if not isinstance(positions, (list, tuple)) or len(positions) > 4:
        raise ValueError("H positions must contain at most four colors")

    normalized = []
    seen = set()
    for item in positions:
        if not isinstance(item, (list, tuple)) or len(item) != 3:
            raise ValueError("H position must be color,x,y")
        code = _color(item[0])
        if code in seen:
            raise ValueError("duplicate H position color")
        seen.add(code)
        px, py = int(item[1]), int(item[2])
        normalized.append((px, code, py))
    normalized.sort(key=lambda item: (item[0], item[1]))

    fields = [
        "HDISC2",
        str(arrival_stopped),
        str(len(normalized)),
    ]
    for px, code, py in normalized:
        fields.extend((code, str(px), str(py)))
    return ",".join(fields)
