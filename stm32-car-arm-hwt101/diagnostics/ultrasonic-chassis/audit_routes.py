"""只读选路审计：不修改固件、参数、Keil工程或烧录设备。"""
from datetime import datetime, timezone
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "diagnostics/ultrasonic-chassis/build" / ("route-audit-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
OUT.mkdir()
relative_out = OUT.relative_to(ROOT)
gcc = Path("C:/msys64/ucrt64/bin/gcc.exe")
env = dict(os.environ, PATH=str(gcc.parent) + os.pathsep + os.environ["PATH"])
names = "arm_control station_job vision_link hmi_status motion navigation road_map heading_sensor heading_control".split()
command = [str(gcc), "-std=c99", "-O2", "-DARM_HOST_TEST", "-DCHASSIS_OBSTACLE_TEST", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function", "-Isrc", "-Ilookup", "diagnostics/ultrasonic-chassis/audit_routes.c", *[f"src/{name}.c" for name in names], "lookup/route_table.c", "-lm", "-o", str(relative_out / "audit_routes.exe")]
result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True)
(OUT / "build.log").write_bytes(result.stdout + result.stderr)
if result.returncode:
    raise RuntimeError((result.stdout + result.stderr).decode(errors="replace"))
print("Audit output:", OUT, flush=True)
with (OUT / "results.json").open("w") as output, (OUT / "run.log").open("w") as errors:
    result = subprocess.run([str(relative_out / "audit_routes.exe")], cwd=ROOT, env=env, stdout=output, stderr=errors)
if result.returncode:
    raise RuntimeError((OUT / "run.log").read_text())
report = json.loads((OUT / "results.json").read_text())
assert report["states"] == 531441 and report["queries"] == 43046721
assert len(report["rounds"]) == 79 * 2 * 3
with (OUT / "stage_routes.csv").open("w", newline="", encoding="utf-8-sig") as stream:
    fields = ["start", "blocked", "policy", "completedJobs", "job", "from", "to", "route", "distanceMm", "turnDeg", "unknownRoads", "passedBefore"]
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    for run in report["rounds"]:
        for stage in run["stages"]:
            writer.writerow({**{k: run[k] for k in fields[:4]}, **stage})
inputs = ["diagnostics/ultrasonic-chassis/audit_routes.c", "diagnostics/ultrasonic-chassis/audit_routes.py", "src/main_table_nav.c", "src/car_config.h", "src/structure.h", "src/road_map.c", "src/full_route.h", "tests/verify_entry.c"]
(OUT / "sources.json").write_text(json.dumps({p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in inputs}, indent=2), encoding="utf-8")
print("PASS", report["states"], "states;", report["queries"], "queries;", len(report["rounds"]), "round scenarios", flush=True)
print(json.dumps(report["roughToStorageExamples"], ensure_ascii=False), flush=True)
print("Motion model ms:", report["motionModelMs"], flush=True)
