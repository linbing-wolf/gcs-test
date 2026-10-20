"""Verify the single-D session change and record exact local validation; no hardware access."""
from pathlib import Path
import difflib
import hashlib
import importlib.util
import json
import re

ROOT = Path(__file__).resolve().parents[1]

def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))

def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def strings(value):
    if isinstance(value, str):
        yield value
    elif isinstance(value, dict):
        for item in value.values():
            yield from strings(item)
    elif isinstance(value, list):
        for item in value:
            yield from strings(item)

context = read(ROOT / 'evidence/vision-session-r18-context.json')
BEFORE = ROOT / context['before']
BUILD = ROOT / context['build']
reports = {}
patterns = {
    'hostHeading': r'PASS heading/feedback: (\d+) assertions',
    'hostArm': r'PASS arm integration: (\d+) assertions',
    'hostContinuous': r'PASS nonstop/obstacle: (\d+) assertions',
    'hostDrive': r'PASS drive dynamics: (\d+) assertions',
    'hostStationVision': r'PASS station/vision integration: (\d+) assertions',
}
for key, relative in context['reports'].items():
    data = read(ROOT / relative)
    assert data['passed'] and data.get('exitCode', 0) == 0
    for variant in data.get('variants', data.get('results', [])):
        assert variant.get('exitCode', 0) == 0
    for name, expected in data['sourceHashes'].items():
        assert sha(ROOT / name) == expected, (relative, name)
    output = '\n'.join(strings(data.get('output', data.get('variants', data.get('results', [])))))
    assert not re.search(r'^FAIL|Assertion.*failed|error:|Unhandled Exception', output, re.M)
    assertions = [int(x) for x in re.findall(patterns[key], output)]
    assert assertions
    reports[key] = dict(passed=True, report=relative, scope=data['scope'], assertions=assertions,
                        hardwareAccuracyVerified=False)
    if key == 'hostStationVision':
        assert len(data['variants']) == 7
        for variant in data['variants']:
            text = '\n'.join(strings(variant['output']))
            assert 'PASS single D session:' in text and 'PASS station vision:' in text

for name in ['src/arm_control.c', 'src/arm_config.h']:
    assert (ROOT / name).read_bytes() == (BEFORE / name).read_bytes(), name
for name in ['evidence/current-validation.json', 'package-manifest.json']:
    assert (ROOT / name).read_bytes() == (BEFORE / name).read_bytes(), 'Concurrent receipt change: ' + name
log = (BUILD / 'build.log').read_text(encoding='utf-8-sig')
exits = re.findall(r'^exitCode=(-?\d+)', log, re.M)
assert len(exits) == 72 and set(exits) == {'0'} and 'Error:' not in log
assert log.count('Warning:') == 30
html = (BUILD / 'table_navigation.htm').read_text(encoding='utf-8-sig')
stack = int(re.search(r'Maximum Stack Usage\s*=\s*(\d+)', html).group(1))
spec = importlib.util.spec_from_file_location('firmware_check', ROOT / 'tools/verify-firmware.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
load = module.elf_load_bytes(BUILD / 'table_navigation.axf')
assert load == module.hex_load_bytes(BUILD / 'table_navigation.hex')
assert b'20261019-merge2-r18' in (BUILD / 'table_navigation.axf').read_bytes()
assert '20261019-merge2-r18' in (ROOT / 'README.md').read_text(encoding='utf-8-sig')
assert (BUILD.relative_to(ROOT) / 'table_navigation.hex').as_posix() in (ROOT / 'README.md').read_text(encoding='utf-8-sig')
# Firmware is delivered from the isolated build. Do not replace independent Keil output.
preserved = {f'keil/OBJ/table_navigation.{ext}': sha(ROOT / f'keil/OBJ/table_navigation.{ext}')
             for ext in ['axf', 'hex', 'map']}
v = read(BEFORE / 'evidence/current-validation.json')
scope = '粗加工和暂存每个对准步骤只发送一次D，连续接收MOVE纠偏，成功DISC结束任务；删除三次独立D确认，不自动发送R、不等待READY。移动中的偏差不排队执行，终包保留至车停稳。抓放参数、125度观察角及3000ms保护保持。'
v.update(version='20261019-merge2-r18', date='2026-10-19', scope=scope,
         preChangeSnapshot=context['before'], previousValidation=context['before'] + '/evidence/current-validation.json')
v.update(reports)
for name in v['sourceHashes']:
    v['sourceHashes'][name] = sha(ROOT / name)
for relative in context['reports'].values():
    v['sourceHashes'].update(read(ROOT / relative)['sourceHashes'])
v['build'] = dict(passed=True, compiler='ARM Compiler 5.06 update 5 build 528', units=70,
    successfulToolExits=72, errors=0, warnings=30, warningsScope='30 inherited vendor warnings; no application warnings',
    log=(BUILD / 'build.log').relative_to(ROOT).as_posix(), verification='evidence/vision-session-r18-verification.json')
v['firmware'] = dict(passed=True, loadBytes=len(load), start=hex(min(load)), endExclusive=hex(max(load)+1),
    artifacts={(BUILD / f'table_navigation.{ext}').relative_to(ROOT).as_posix(): sha(BUILD / f'table_navigation.{ext}') for ext in ['axf', 'hex', 'map']})
v['concurrentKeilOutputsPreserved'] = dict(artifacts=preserved, scope='Keil default output untouched; current verified firmware is in the isolated build directory')
v['staticStack']['reportedMaximumBytes'] = stack
sv = v['stationVision']
for name in ['independentConfirmationRequests', 'oneReplyConsumedPerRequest', 'operatorConfirmedSelfStopping', 'selfStoppingUpperSoftwarePhysicallyVerified']:
    sv.pop(name, None)
sv.update(verification='evidence/vision-session-r18-verification.json',
    protocol=['one D per alignment -> continuous MOVE,dx -> final DISC,x,y,dx,1',
              'MOVE -> move3mm -> stopped/relocked -> consume later MOVE, no new D'],
    requestsPerAlignmentStep=1, successfulDiscEndsSession=True, moveKeepsSessionWaiting=True,
    latestMoveConsumedOnce=True, rejectMovesBeforeStopPlus100Ms=True, terminalDiscWaitsForStoppedChassis=True,
    sessionTimeoutMs=25000, runningVisionSoftwareVerified=False,
    protocolEvidence='evidence/vision-source-20261019/maixcam_app_v2/tasks.py: task_disc sends MOVE repeatedly and returns final DISC',
    automaticResetFrames=False, requiresReadyHandshake=False,
    lateReplyLimitation='No request ID; queued old bytes are cleared but in-flight cross-session late replies cannot be distinguished')
v['roughVision']['verification'] = 'evidence/vision-session-r18-verification.json'
v['documentOperation'].update(verification='evidence/vision-session-r18-verification.json',
    currentOverride='r18 restores one D for a continuous MOVE session until final DISC; no automatic R')
v['remainingLimitations'] = [x for x in v['remainingLimitations'] if not x.startswith('r17 relies')]
v['remainingLimitations'].append('r18 follows the retained task_disc source, but the running vision firmware was not accessed. Terminal DISC received during a3mm correction is held until stopped; physical alignment remains unverified. No request IDs distinguish in-flight cross-session late replies.')
write(ROOT / 'evidence/current-validation.json', v)
proof = dict(passed=True, version=v['version'], scope=scope, hostReports=reports,
    exactReportSourceHashesMatch=True, build=v['build'], firmware=v['firmware'], stack=v['staticStack'],
    stationVision=sv, mechanicalParametersUnchanged=True, defaultKeilOutputsPreserved=preserved,
    preChangeSnapshot=context['before'], sourceDiff='evidence/vision-session-r18-source.diff',
    buildEnvironment='Sandbox ARM5 DLL relocation failed; authorized outside-sandbox isolated build completed successfully',
    hardwareAccessed=False, flashed=False, physicalMotionVerified=False)
write(ROOT / 'evidence/vision-session-r18-verification.json', proof)
m = read(ROOT / 'package-manifest.json')
m.update(version='merged2-vision-session-20261019-r18', updatedDate='2026-10-19', scheme=scope)
extra = ['evidence/vision-session-r18-verification.json', 'evidence/vision-session-r18-source.diff',
         'evidence/vision-session-r18-context.json', 'evidence/finalize-vision-session-r18.py',
         *context['reports'].values(), (BUILD / 'build.log').relative_to(ROOT).as_posix(),
         (BUILD / 'table_navigation.htm').relative_to(ROOT).as_posix(), *v['firmware']['artifacts'].keys()]
for name in list(m['files']) + extra:
    assert not Path(name).name.lower().startswith('.env') and Path(name).name.lower() != 'auth.json'
    m['files'][name] = sha(ROOT / name)
write(ROOT / 'package-manifest.json', m)
print(json.dumps(dict(passed=True, version=v['version'], assertions={key:val['assertions'] for key,val in reports.items()},
                     build=v['build'], firmware=v['firmware'], stackBytes=stack, manifestFiles=len(m['files'])), indent=2))
