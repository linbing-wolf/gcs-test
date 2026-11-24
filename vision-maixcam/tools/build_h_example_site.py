"""Build a standalone H replay page and contact sheet from captured MaixCAM logs."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
import sys

from PIL import Image, ImageDraw, ImageFont


COLOR_NAMES = {"1": "红", "2": "黄", "3": "蓝", "4": "绿"}
COLOR_HEX = {
    "1": "#ef4d5b",
    "2": "#f3cf3e",
    "3": "#3978ef",
    "4": "#36a866",
}
SOURCE_SIZE = (640, 480)


def load_jsonl(path: Path) -> list[dict]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def parse_capture_time(folder: str) -> str:
    try:
        stamp = datetime.strptime(folder[:15], "%Y%m%d-%H%M%S")
    except ValueError:
        return folder[:15]
    return stamp.strftime("%Y-%m-%d %H:%M:%S")


def color_text(codes: list[str]) -> str:
    return " → ".join(COLOR_NAMES.get(code, code) for code in codes)


def simplify_candidate(candidate: dict) -> dict:
    return {
        "code": str(candidate["code"]),
        "x": candidate["x"],
        "y": candidate["y"],
        "pixels": candidate["pixels"],
        "raw": candidate.get("raw"),
        "pairEligible": bool(candidate.get("pair_eligible")),
    }


def simplify_frame(row: dict) -> dict:
    return {
        "t": row["t"],
        "phase": row.get("phase"),
        "img": row.get("img"),
        "candidates": [simplify_candidate(item) for item in row.get("candidates", [])],
        "center": row.get("center"),
        "dx": row.get("dx"),
        "dy": row.get("dy"),
        "distanceError": row.get("distance_error"),
        "colors": [str(code) for code in row.get("colors", [])],
        "stoppedColors": [str(code) for code in row.get("stopped_colors", [])],
        "movingPair": bool(row.get("moving_pair")),
        "moves": row.get("moves"),
        "direction": row.get("direction"),
        "directionAssumed": row.get("direction_assumed"),
        "sent": row.get("sent"),
        "reason": row.get("reason"),
    }


def load_session(folder: Path) -> dict:
    rows = load_jsonl(folder / "meta.jsonl")
    summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    frame_rows = [row for row in rows if row.get("type") == "frame"]
    settings = next(
        (row for row in frame_rows if row.get("phase") == "settings"),
        {},
    )
    keyframes = [simplify_frame(row) for row in frame_rows if row.get("img")]
    trace = [
        simplify_frame(row)
        for row in frame_rows
        if row.get("phase") != "settings" and row.get("candidates")
    ]
    moves = [
        simplify_frame(row)
        for row in frame_rows
        if row.get("phase") in ("move", "final_budget")
    ]
    terminal = next(
        (simplify_frame(row) for row in reversed(frame_rows) if row.get("center")),
        None,
    )
    result_parts = summary["result"].split(",")
    direction_events = []
    previous_direction = object()
    for row in frame_rows:
        if row.get("direction") not in (0, 1) or not isinstance(
            row.get("direction_assumed"), bool
        ):
            continue
        state = (row.get("direction"), row.get("direction_assumed"))
        if state == previous_direction:
            continue
        previous_direction = state
        direction_events.append(
            {
                "t": row["t"],
                "direction": row.get("direction"),
                "assumed": row.get("direction_assumed"),
            }
        )
    color_events = []
    previous_colors: list[str] = []
    for row in frame_rows:
        colors = [str(code) for code in row.get("colors", [])]
        if colors != previous_colors:
            color_events.append({"t": row["t"], "colors": colors})
            previous_colors = colors

    colors = [str(code) for code in summary.get("colors", [])]
    return {
        "id": folder.name,
        "capturedAt": parse_capture_time(folder.name),
        "durationMs": summary.get("duration_ms", 0),
        "result": summary["result"],
        "x": int(result_parts[1]),
        "y": int(result_parts[2]),
        "dx": int(result_parts[3]),
        "colors": colors,
        "colorsLabel": color_text(colors),
        "direction": int(result_parts[-1]),
        "moves": summary.get("moves", 0),
        "framesSaved": summary.get("frames_saved", len(keyframes)),
        "settings": {
            "targetX": settings.get("target_x"),
            "targetY": settings.get("target_y"),
            "roi": settings.get("roi"),
            "tolerance": settings.get("tolerance"),
            "gain": settings.get("gain"),
            "maxStep": settings.get("max_step"),
        },
        "keyframes": keyframes,
        "trace": trace,
        "movesDetail": moves,
        "terminal": terminal,
        "directionEvents": direction_events,
        "colorEvents": color_events,
    }


def load_font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    candidates = [
        Path("C:/Windows/Fonts/msyhbd.ttc" if bold else "C:/Windows/Fonts/msyh.ttc"),
        Path("C:/Windows/Fonts/simhei.ttf"),
    ]
    for path in candidates:
        if path.exists():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def draw_dashed_line(
    draw: ImageDraw.ImageDraw,
    points: tuple[int, int, int, int],
    fill: str,
    width: int = 2,
    dash: int = 8,
) -> None:
    x1, y1, x2, y2 = points
    if x1 == x2:
        for y in range(y1, y2, dash * 2):
            draw.line((x1, y, x2, min(y + dash, y2)), fill=fill, width=width)
    else:
        for x in range(x1, x2, dash * 2):
            draw.line((x, y1, min(x + dash, x2), y2), fill=fill, width=width)


def annotate_image(image_path: Path, frame: dict, settings: dict) -> Image.Image:
    source = Image.open(image_path).convert("RGB")
    if source.size != SOURCE_SIZE:
        raise ValueError(f"Unexpected image size {source.size} for {image_path}")
    draw = ImageDraw.Draw(source, "RGBA")
    roi = settings.get("roi")
    if roi:
        x, y, width, height = roi
        draw.rectangle((x, y, x + width, y + height), outline="#ffffffbb", width=2)
    target_x = settings.get("targetX") or 320
    target_y = settings.get("targetY") or 318
    draw_dashed_line(draw, (target_x, 0, target_x, 480), "#53f2c6cc")
    draw_dashed_line(draw, (0, target_y, 640, target_y), "#53f2c6cc")
    draw.ellipse(
        (target_x - 8, target_y - 8, target_x + 8, target_y + 8),
        outline="#53f2c6",
        width=2,
    )
    for candidate in frame["candidates"]:
        code = candidate["code"]
        x = candidate["x"]
        y = candidate["y"]
        color = COLOR_HEX.get(code, "#ffffff")
        radius = 14 if candidate["pairEligible"] else 10
        draw.ellipse(
            (x - radius, y - radius, x + radius, y + radius),
            outline=color,
            width=4 if candidate["pairEligible"] else 2,
        )
        draw.line((x - 22, y, x + 22, y), fill=color, width=2)
        draw.line((x, y - 22, x, y + 22), fill=color, width=2)
        label = f"{COLOR_NAMES.get(code, code)} {x},{y}"
        label_x = min(max(6, x + 14), 510)
        label_y = min(max(6, y - 30), 448)
        draw.rounded_rectangle(
            (label_x, label_y, label_x + 112, label_y + 24),
            radius=5,
            fill="#101820d9",
            outline=color,
            width=2,
        )
        draw.text((label_x + 7, label_y + 3), label, fill="#ffffff", font=load_font(14, True))
    if frame.get("center"):
        x, y = frame["center"]
        draw.rectangle((x - 18, y - 18, x + 18, y + 18), outline="#ffffff", width=3)
    return source


def build_contact_sheet(
    sessions: list[dict],
    source_root: Path,
    output_path: Path,
) -> None:
    tile_size = (360, 270)
    left = 180
    header_height = 62
    tile_gap = 12
    row_height = header_height + tile_size[1] + 20
    width = left + 3 * tile_size[0] + 2 * tile_gap + 20
    height = 76 + row_height * len(sessions) + 28
    sheet = Image.new("RGB", (width, height), "#0e1418")
    draw = ImageDraw.Draw(sheet)
    title_font = load_font(28, True)
    body_font = load_font(17)
    small_font = load_font(14)
    draw.text((20, 18), "MaixCAM · 最近 4 次 H 识别图", fill="#f4f7f5", font=title_font)
    draw.text(
        (20, 51),
        "真实原始帧，叠加当前识别中心；虚线为目标 X/Y。",
        fill="#8fa0a5",
        font=small_font,
    )
    for row_index, session in enumerate(sessions):
        y0 = 76 + row_index * row_height
        result = session["result"]
        draw.text(
            (20, y0 + 9),
            session["capturedAt"][11:],
            fill="#f4f7f5",
            font=body_font,
        )
        draw.text(
            (20, y0 + 33),
            session["colorsLabel"] or "未识别双色",
            fill="#53f2c6",
            font=small_font,
        )
        draw.text(
            (20, y0 + 53),
            f"{session['durationMs']} ms · {session['moves']} move",
            fill="#8fa0a5",
            font=small_font,
        )
        for frame_index, frame in enumerate(session["keyframes"]):
            image_path = source_root / session["id"] / frame["img"]
            annotated = annotate_image(image_path, frame, session["settings"])
            annotated.thumbnail(tile_size, Image.Resampling.LANCZOS)
            x0 = left + frame_index * (tile_size[0] + tile_gap)
            y1 = y0 + header_height
            sheet.paste(annotated, (x0, y1))
            draw.rectangle(
                (x0, y1, x0 + tile_size[0] - 1, y1 + tile_size[1] - 1),
                outline="#2c393f",
                width=1,
            )
            draw.rounded_rectangle(
                (x0 + 8, y1 + 8, x0 + 138, y1 + 34),
                radius=5,
                fill="#0e1418d9",
            )
            draw.text(
                (x0 + 14, y1 + 12),
                f"{frame_index + 1:02d} · {frame['t']} ms",
                fill="#f4f7f5",
                font=small_font,
            )
            state = "自动" if frame.get("directionAssumed") is False else "兜底"
            direction = "CW" if frame.get("direction") == 0 else "CCW"
            draw.text(
                (x0 + 155, y1 + tile_size[1] - 28),
                f"{direction} · {state}",
                fill="#f4f7f5",
                font=small_font,
            )
        draw.text(
            (left, y0 + header_height + tile_size[1] + 2),
            result,
            fill="#b9c6c9",
            font=small_font,
        )
    draw.text(
        (20, height - 20),
        "源码：/root/yyb_mover_data/plate_debug/*-H-material",
        fill="#66777d",
        font=small_font,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(output_path, quality=94, subsampling=0)


def copy_images(sessions: list[dict], source_root: Path, output_root: Path) -> None:
    for session in sessions:
        target = output_root / "assets" / "frames" / session["id"]
        target.mkdir(parents=True, exist_ok=True)
        for frame in session["keyframes"]:
            shutil.copy2(source_root / session["id"] / frame["img"], target / frame["img"])


def write_data(sessions: list[dict], output_root: Path) -> None:
    data = {
        "generatedAt": datetime.now().isoformat(timespec="seconds"),
        "source": "/root/yyb_mover_data/plate_debug/*-H-material",
        "board": {
            "targetX": 320,
            "targetY": 318,
            "cameraWidth": 640,
            "cameraHeight": 480,
            "format": "HDISC,x,y,dx,first,second,direction",
        },
        "sessions": sessions,
    }
    text = "window.H_EXAMPLE_DATA = " + json.dumps(data, ensure_ascii=False, indent=2) + ";\n"
    (output_root / "data.js").write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        type=Path,
        default=Path("vision-maixcam/_from_device/recent_h_images_20261123"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("vision-maixcam/h_current_example"),
    )
    args = parser.parse_args()
    source_root = args.source.resolve()
    output_root = args.output.resolve()
    folders = sorted(
        (
            item
            for item in source_root.iterdir()
            if item.is_dir() and item.name.endswith("-H-material")
        ),
        reverse=True,
    )
    if not folders:
        raise RuntimeError("No captured H sessions were found")
    sessions = [load_session(folder) for folder in folders]
    output_root.mkdir(parents=True, exist_ok=True)
    copy_images(sessions, source_root, output_root)
    write_data(sessions, output_root)
    build_contact_sheet(sessions, source_root, output_root / "contact-sheet.jpg")
    print(f"Built {len(sessions)} sessions in {output_root}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
