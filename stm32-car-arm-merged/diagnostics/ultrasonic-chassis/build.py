"""从正式 Keil 项目派生底盘避障工程，运行主机模型并独立构建；不烧录。"""
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
ARM = Path("D:/keil5/ARM/ARMCC/bin")
GCC = Path("C:/msys64/ucrt64/bin/gcc.exe")
OUT = HERE / "build" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
OUT.mkdir(parents=True)


def invoke(args):
    cmd = [os.path.relpath(x, HERE) if isinstance(x, Path) and x.is_relative_to(ROOT) else str(x)
           for x in args]
    cmd[0] = str((HERE / cmd[0]).resolve())
    env = dict(os.environ, PATH=str(GCC.parent) + os.pathsep + os.environ["PATH"])
    result = subprocess.run(cmd, cwd=HERE, env=env, capture_output=True)
    output = (result.stdout + result.stderr).decode(errors="replace")
    with (OUT / "build.log").open("a", encoding="utf-8") as log:
        log.write(f"{subprocess.list2cmdline(cmd)}\n{output}\nexitCode={result.returncode}\n")
    if result.returncode:
        raise RuntimeError(output)
    return output


def relative(path):
    return os.path.relpath(path, HERE)


base = ROOT / "keil/TableNavigation.uvprojx"
project = ET.parse(base)
target = project.find("./Targets/Target")
options = target.find("TargetOption")
target.find("TargetName").text = "UltrasonicChassis"
options.find("TargetCommonOption/OutputName").text = "ultrasonic_chassis"
controls = options.find("TargetArmAds/Cads/VariousControls")
controls.find("Define").text += ",CHASSIS_OBSTACLE_TEST"
options.find("TargetArmAds/Cads/Optim").text = "3"  # Keil ARMCC level 2
options.find("TargetArmAds/Cads/SplitLS").text = "1"
options.find("TargetArmAds/Cads/uC99").text = "1"
includes = [(base.parent / p).resolve() for p in controls.findtext("IncludePath").split(";")]
controls.find("IncludePath").text = ";".join(map(relative, includes))
scatter = (base.parent / options.findtext("TargetArmAds/LDads/ScatterFile")).resolve()
options.find("TargetArmAds/LDads/ScatterFile").text = relative(scatter)
sources = []
for files in target.findall("./Groups/Group/Files"):
    for entry in list(files):
        source = (base.parent / entry.findtext("FilePath")).resolve()
        if source.name.lower() == "pid.c":  # TIM4 belongs to the software UART.
            files.remove(entry)
            continue
        entry.find("FilePath").text = relative(source)
        if entry.findtext("FileType") in ("1", "2"):
            sources.append(source)
radio = HERE.parent / "ultrasonic-dual-wireless/main.c"
group = ET.SubElement(target.find("Groups"), "Group")
ET.SubElement(group, "GroupName").text = "Dual ultrasonic and software UART"
entry = ET.SubElement(ET.SubElement(group, "Files"), "File")
ET.SubElement(entry, "FileName").text = "dual_ultrasonic_main.c"
ET.SubElement(entry, "FileType").text = "1"
ET.SubElement(entry, "FilePath").text = relative(radio)
sources.append(radio)
ET.indent(project, space="  ")
project_path = HERE / "Ultrasonic_Chassis.uvprojx"
project_bytes = ET.tostring(project.getroot(), encoding="utf-8", xml_declaration=True) + b"\n"
if not project_path.exists():
    project_path.write_bytes(project_bytes)
elif project_path.read_bytes() != project_bytes:
    raise RuntimeError("Keil base changed. Review the differences before regenerating the chassis project.")

host = OUT / "verify_chassis.exe"
host_names = "arm_control station_job vision_link hmi_status motion navigation road_map heading_sensor heading_control".split()
invoke([GCC, "-std=c99", "-O2", "-DARM_HOST_TEST", "-DCHASSIS_OBSTACLE_TEST", "-Wall", "-Wextra",
        "-Werror", "-Wno-unused-function", "-I../../src", "-I../../lookup", HERE / "verify_chassis.c",
        *[ROOT / "src" / (name + ".c") for name in host_names], ROOT / "lookup/route_table.c", "-lm", "-o", host])
host_output = invoke([host]).strip()
print(host_output, flush=True)
objects = []
for i, source in enumerate(sources):
    obj = OUT / f"{i}_{source.stem}.o"
    if source.suffix.lower() == ".s":
        invoke([ARM / "armasm.exe", "--cpu", "Cortex-M4.fp.sp", "--apcs=interwork", "-g", source, "-o", obj])
    else:
        invoke([ARM / "armcc.exe", "--cpu", "Cortex-M4.fp.sp", "--apcs=interwork", "--c99", "--split_sections", "-g", "-O2",
                "-DSTM32F40_41xxx", "-DUSE_STDPERIPH_DRIVER", "-DCHASSIS_OBSTACLE_TEST",
                *["-I" + relative(p) for p in includes], "-c", source, "-o", obj])
    objects.append(obj)
axf, hexfile = OUT / "ultrasonic_chassis.axf", OUT / "ultrasonic_chassis.hex"
invoke([ARM / "armlink.exe", "--cpu", "Cortex-M4.fp.sp", "--scatter", scatter, "--map", "--callgraph",
        "--list", OUT / "ultrasonic_chassis.map", "--output", axf, *objects])
invoke([ARM / "fromelf.exe", "--i32combined", "--output", hexfile, axf])
spec = importlib.util.spec_from_file_location("firmware_verify", ROOT / "tools/verify-firmware.py")
verify = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verify)
load = verify.hex_load_bytes(hexfile)
assert verify.elf_load_bytes(axf) == load
raw = axf.read_bytes()
header = struct.unpack_from("<16sHHIIIIIHHHHHH", raw)
sections = [struct.unpack_from("<10I", raw, header[6] + i * header[11]) for i in range(header[12])]
symbols = {}
for section in sections:
    if section[1] == 2:
        strings = sections[section[6]]
        table = raw[strings[4]:strings[4] + strings[5]]
        for i in range(section[4], section[4] + section[5], section[9]):
            name, value, _, _, _, index = struct.unpack_from("<IIIBBH", raw, i)
            if index:
                symbols[table[name:table.find(b"\0", name)].decode(errors="replace")] = value
vectors = {}
for name, irq in (("EXTI1",7),("TIM3",29),("TIM4",30),("USART2",38),("EXTI15_10",40),("TIM5",50),("CAN1_RX0",20)):
    start = 0x08000000 + (16 + irq) * 4
    address = int.from_bytes(bytes(load[start + i] for i in range(4)), "little")
    assert address == (symbols[name + "_IRQHandler"] | 1), name
    vectors[name] = hex(address)
for name in ("arm_init", "PWM_Init", "station_init", "vision_init", "hmi_init", "start_scan_init", "uart_init", "scanner_init"):
    assert name not in symbols, f"unexpected initialization in test build: {name}"
report = {"passed": True, "units": len(sources), "hostOutput": host_output, "hardwareAccessed": False,
          "axfHexLoadBytesEqual": True, "interruptVectors": vectors,
          "hexSHA256": hashlib.sha256(hexfile.read_bytes()).hexdigest(),
          "sourceHashes": {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in [*sources, base, project_path, HERE / "obstacle_io.h", HERE / "verify_chassis.c"]}}
(OUT / "result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print(f"PASS ARM build ({len(sources)} units); HEX = AXF; {OUT}")
