"""Small stdlib-only project harness. No installs, flights or arbitrary commands."""
import argparse
import ast
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
STATES = {'not_started', 'in_progress', 'blocked', 'passing'}
REQUIRED = [
    'AGENTS.md', 'init.sh', 'PROGRESS.md', 'session-handoff.md',
    'feature_list.json', 'docs/harness/README.md',
    'docs/harness/architecture.md', 'docs/harness/runbook.md',
    'docs/hardware/dm_fc01.md', 'docs/hardware/data_capture.md',
    'config/hardware/dm_fc01.json',
    'config/logging/dm_fc01_logger_topics.example.txt',
    'data/templates/impact_trial.json', 'tools/harness.py', 'tests/test_harness.py',
]
RECIPES = {
    'H01': [
        ['tools/harness.py', 'check', '--skip-evidence'],
        ['-m', 'unittest', 'discover', '-s', 'tests', '-p', 'test_harness.py'],
    ],
    'A01': [['tools/harness.py', 'assets']],
    'S01': [['scripts/view_user_aircraft.py', '--headless']],
    'S02': [['tools/check_user_aircraft.py', '--ctl_mode', 'rate',
             '--num_envs', '4', '--headless', '--seed', '64']],
    'S03': [
        ['tools/check_armor_impact.py', '--ctl_mode', 'rate',
         '--num_envs', '4', '--headless', '--seed', '42'],
        ['tools/check_armor_impact.py', '--ctl_mode', 'rate',
         '--num_envs', '4', '--headless', '--seed', '42', '--zero-restitution'],
    ],
}


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def write_json(path, value):
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def local_path(root, relative):
    """Require a real local path; no absolute, parent or symlink escape."""
    path = Path(relative)
    if path.is_absolute() or '..' in path.parts:
        raise ValueError('Expected an in-project relative path: ' + str(relative))
    resolved = (root / path).resolve()
    if root.resolve() not in resolved.parents:
        raise ValueError('Path escapes project: ' + str(relative))
    return resolved


def feature_spec(feature):
    return {k: v for k, v in feature.items() if k not in {'status', 'evidence', 'notes'}}


def inputs_for(root, feature):
    identifier = feature['id']
    paths = {'tools/harness.py'}
    if identifier == 'H01':
        paths.update(REQUIRED)
        paths.difference_update({'feature_list.json', 'PROGRESS.md', 'session-handoff.md'})
    else:
        asset = root / 'airgym/assets/robots/user160'
        paths.update(str(p.relative_to(root)) for p in asset.rglob('*') if p.is_file())
        paths.add('docs/user_aircraft_assembly_step_validation.json')
        if identifier.startswith('S'):
            for prefix in ('airgym', 'lib', 'scripts'):
                paths.update(str(p.relative_to(root)) for p in (root / prefix).rglob('*')
                             if p.is_file() and p.suffix in
                             {'.py', '.yaml', '.yml', '.json', '.urdf', '.obj', '.stl', '.dae', '.mtl'})
            paths.update({'tools/check_user_aircraft.py', 'tools/check_armor_impact.py'})
    result = {p: sha(local_path(root, p)) for p in sorted(paths)}
    payload = json.dumps(feature_spec(feature), sort_keys=True, ensure_ascii=False).encode()
    result['@feature_spec'] = hashlib.sha256(payload).hexdigest()
    return result


def validate_features(root, data, check_evidence=True):
    errors = []
    features = data.get('features', [])
    if data.get('schema_version') != 1 or not features:
        errors.append('Invalid or empty feature list')
    identifiers = [f.get('id') for f in features]
    if len(set(identifiers)) != len(identifiers):
        errors.append('Duplicate feature id')
    if sum(f.get('status') == 'in_progress' for f in features) > 1:
        errors.append('More than one feature is in_progress')
    for feature in features:
        identifier = feature.get('id', '?')
        if feature.get('status') not in STATES:
            errors.append(identifier + ': unknown status')
        for field in ('title', 'area', 'user_visible_behavior', 'verification'):
            if not feature.get(field):
                errors.append(identifier + ': missing ' + field)
        if not isinstance(feature.get('priority'), int):
            errors.append(identifier + ': priority must be an integer')
        if feature.get('status') != 'passing' or not check_evidence:
            continue
        try:
            if identifier not in RECIPES:
                raise ValueError('no executable acceptance recipe; cannot be passing')
            evidence = local_path(root, feature.get('evidence') or '')
            record = read_json(evidence)
            if record.get('feature_id') != identifier or record.get('passed') is not True:
                raise ValueError('evidence does not record a successful matching verification')
            expected = [[record['python']] + step for step in RECIPES[identifier]]
            commands = record.get('commands', [])
            if len(commands) != len(expected):
                raise ValueError('incomplete verification commands')
            for command, argv in zip(commands, expected):
                if command.get('argv') != argv or command.get('returncode') != 0:
                    raise ValueError('verification command failed or differs from recipe')
                logfile = local_path(root, command['log'])
                if not logfile.is_file() or sha(logfile) != command.get('log_sha256'):
                    raise ValueError('missing or changed execution log')
            if record.get('inputs') != inputs_for(root, feature):
                raise ValueError('stale input hashes; re-run verification')
        except (OSError, ValueError, KeyError, TypeError) as exc:
            errors.append(identifier + ': ' + str(exc))
    return errors


def check_assets(root):
    errors = []
    try:
        folder = root / 'airgym/assets/robots/user160'
        urdf = folder / 'model.urdf'
        robot = ET.parse(urdf).getroot()
        joints = robot.findall('joint')
        expected = {'base_link2prop_' + str(i) for i in range(1, 5)}
        if {j.get('name') for j in joints} != expected or any(j.get('type') != 'fixed' for j in joints):
            errors.append('Training URDF must retain the four fixed motor joints')
        mass = sum(float(m.get('value')) for m in robot.findall('./link/inertial/mass'))
        if not math.isclose(mass, 0.160, abs_tol=1e-7):
            errors.append('Aircraft URDF mass is not 0.160 kg')
        for mesh in robot.findall('.//mesh'):
            if not local_path(folder, mesh.get('filename')).is_file():
                errors.append('Missing mesh: ' + str(mesh.get('filename')))
        report = read_json(root / 'docs/user_aircraft_assembly_step_validation.json')
        if report.get('passed') is not True or report.get('profile') != 'user160_cad_v1':
            errors.append('Historical assembly STEP report did not pass for current profile')
        if report.get('urdf_sha256') != sha(urdf):
            errors.append('STEP report refers to another URDF; do not claim current CAD verification')
        if report.get('assembled_visual_sha256') != sha(folder / 'assembled_visual.stl'):
            errors.append('STEP report assembled visual hash differs')
    except (OSError, ValueError, TypeError, KeyError, ET.ParseError) as exc:
        errors.append('Aircraft artifact check: ' + str(exc))
    return errors


def check(root, evidence=True):
    errors = []
    for name in REQUIRED:
        if not (root / name).is_file():
            errors.append('Missing required file: ' + name)
    if errors:
        return errors
    try:
        errors.extend(validate_features(root, read_json(root / 'feature_list.json'), evidence))
        hardware = read_json(root / 'config/hardware/dm_fc01.json')
        if hardware.get('board') != 'DM-FC01':
            errors.append('Hardware manifest board differs')
        template = read_json(root / 'data/templates/impact_trial.json')
        if template.get('flight_controller', {}).get('firmware_version') is not None:
            errors.append('Trial template must not invent the installed firmware version')
        for name in ('tools/harness.py', 'tests/test_harness.py',
                     'airgym/envs/task/armor_impact.py', 'airgym/envs/task/armor_impact_config.py',
                     'scripts/train_armor_impact.py', 'scripts/eval_armor_impact.py'):
            ast.parse((root / name).read_text(encoding='utf-8'), filename=name)
        for line in (root / 'config/logging/dm_fc01_logger_topics.example.txt').read_text().splitlines():
            if len(line.encode()) >= 79:
                errors.append('Logger example exceeds inspected firmware line buffer')
            if line and not line.startswith('#') and not re.fullmatch(r'[a-z_]+ \d+(?: \d+)?', line):
                errors.append('Invalid candidate logger topic line: ' + line)
        errors.extend(check_assets(root))
    except (OSError, ValueError, SyntaxError, TypeError) as exc:
        errors.append(str(exc))
    return errors


def print_errors(errors, success):
    for error in errors:
        print('FAIL ' + error)
    if not errors:
        print('PASS ' + success)
    return int(bool(errors))


def require_current_simulation_mass(root):
    folder = root / 'airgym/assets/robots/user160'
    parameters = read_json(folder / 'parameters.json')
    build = read_json(folder / 'build_report.json')
    measured = parameters['measured']['takeoff_mass_kg']
    modeled = build['mass_kg']
    if not math.isclose(measured, modeled, abs_tol=1e-7):
        raise ValueError('Simulation mass %.3f kg differs from measured %.3f kg; '
                         'update the mass distribution/model before current-aircraft GPU validation'
                         % (modeled, measured))


def verify(root, identifier):
    data = read_json(root / 'feature_list.json')
    feature = next(f for f in data['features'] if f['id'] == identifier)
    if identifier.startswith('S'):
        require_current_simulation_mass(root)
    initial_inputs = inputs_for(root, feature)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]
    directory = root / 'runs/harness' / (stamp + '-' + identifier)
    directory.mkdir(parents=True, exist_ok=False)
    record = {'feature_id': identifier, 'created_at_utc': stamp.split('-')[0],
              'python': sys.executable, 'inputs': initial_inputs, 'commands': [], 'passed': False}
    for index, step in enumerate(RECIPES[identifier]):
        argv = [sys.executable] + step
        logfile = directory / ('step_' + str(index + 1) + '.log')
        print('RUN ' + ' '.join(argv), flush=True)
        with logfile.open('w', encoding='utf-8') as stream:
            try:
                result = subprocess.run(argv, cwd=str(root), stdout=stream,
                                        stderr=subprocess.STDOUT, timeout=240)
                code = result.returncode
            except subprocess.TimeoutExpired:
                stream.write('\nHARNESS TIMEOUT after 240 seconds\n')
                code = 124
        record['commands'].append({'argv': argv, 'returncode': code,
                                   'log': str(logfile.relative_to(root)), 'log_sha256': sha(logfile)})
        print('\n'.join(logfile.read_text(errors='replace').splitlines()[-8:]), flush=True)
        if code:
            break
    record['passed'] = (len(record['commands']) == len(RECIPES[identifier]) and
                        all(c['returncode'] == 0 for c in record['commands']) and
                        initial_inputs == inputs_for(root, feature))
    evidence = directory / 'evidence.json'
    write_json(evidence, record)
    feature['status'] = 'passing' if record['passed'] else 'blocked'
    feature['evidence'] = str(evidence.relative_to(root))
    write_json(root / 'feature_list.json', data)
    print(('PASS ' if record['passed'] else 'FAIL ') + identifier + ': ' + feature['evidence'])
    return 0 if record['passed'] else 1


def new_trial(root, identifier, destination=None):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', identifier):
        raise ValueError('Trial id must be 1-64 ASCII letters/numbers/_/- without path separators')
    folder = destination or root / 'data/captures' / identifier
    folder.mkdir(parents=True, exist_ok=False)
    data = read_json(root / 'data/templates/impact_trial.json')
    data['trial_id'] = identifier
    write_json(folder / 'trial.json', data)
    (folder / 'README.txt').write_text(
        'Fill trial.json with measured values; keep unknown fields null.\n'
        'Place original files here. No fake ULog/video files are generated.\n'
        'See docs/hardware/data_capture.md.\n', encoding='utf-8')
    return folder


def positive(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value) and value > 0


def check_trial(folder):
    errors = []
    try:
        data = read_json(folder / 'trial.json')
        for field in ('trial_id', 'kind', 'recorded_at'):
            if not data.get(field):
                errors.append('Missing metadata: ' + field)
        if data.get('kind') not in {'static', 'hover', 'impact'}:
            errors.append('kind must be static, hover or impact')
        aircraft = data.get('aircraft', {})
        for field in ('takeoff_mass_kg', 'battery_cells', 'battery_start_voltage_v'):
            if not positive(aircraft.get(field)):
                errors.append('Missing positive aircraft.' + field)
        fc = data.get('flight_controller', {})
        for field in ('firmware_family', 'firmware_version', 'selected_gyro_device_id', 'selected_accel_device_id'):
            if not fc.get(field):
                errors.append('Missing flight_controller.' + field)
        for field in ('world_frame', 'body_frame', 'board_to_body_rotation'):
            if data.get('coordinates', {}).get(field) is None:
                errors.append('Missing coordinates.' + field)
        keys = ['ulog', 'firmware_info', 'parameters']
        if data.get('kind') == 'impact':
            keys += ['video', 'synchronization']
            camera = data.get('camera', {})
            if not positive(camera.get('capture_fps')) or not positive(camera.get('scale_reference_m')):
                errors.append('Impact requires capture fps and scale reference')
            sync = data.get('synchronization', {})
            if not sync.get('method') or sync.get('video_time_equals_px4_time_plus_offset_s') is None:
                errors.append('Impact requires time synchronization and offset')
            uncertainty = sync.get('uncertainty_s')
            if not isinstance(uncertainty, (int, float)) or isinstance(uncertainty, bool) or not math.isfinite(uncertainty) or uncertainty < 0:
                errors.append('Impact requires nonnegative synchronization uncertainty')
            for field in ('plate_mount', 'plate_thickness_m', 'contact_location_description', 'outcome'):
                if not data.get('impact', {}).get(field):
                    errors.append('Missing impact.' + field)
        for key in keys:
            relative = data.get('files', {}).get(key)
            if not relative:
                errors.append('Missing file entry: ' + key)
                continue
            path = local_path(folder, relative)
            if not path.is_file() or path.stat().st_size == 0:
                errors.append('Missing or empty original file: ' + relative)
            elif key == 'ulog':
                with path.open('rb') as stream:
                    if stream.read(7) != b'ULog\x01\x12\x35':
                        errors.append('Invalid ULog signature: ' + relative)
    except (OSError, ValueError, TypeError) as exc:
        errors.append(str(exc))
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('doctor')
    sub.add_parser('status')
    sub.add_parser('assets')
    p = sub.add_parser('check')
    p.add_argument('--skip-evidence', action='store_true', help='Internal re-verification structural check only')
    p = sub.add_parser('verify')
    p.add_argument('feature', choices=sorted(RECIPES))
    p = sub.add_parser('new-trial')
    p.add_argument('--id', required=True)
    p.add_argument('--output', type=Path)
    p = sub.add_parser('trial-check')
    p.add_argument('folder', type=Path)
    args = parser.parse_args()
    if args.command == 'verify':
        return verify(ROOT, args.feature)
    if args.command == 'new-trial':
        print(new_trial(ROOT, args.id, args.output))
        return 0
    if args.command == 'trial-check':
        return print_errors(check_trial(args.folder.resolve()),
                            'Trial metadata/files present; signal quality and calibration NOT validated')
    if args.command == 'assets':
        return print_errors(check_assets(ROOT), 'Training URDF and historical STEP evidence integrity')
    if args.command in {'doctor', 'status'}:
        data = read_json(ROOT / 'feature_list.json')
        print('Project: ' + str(ROOT))
        print('States: ' + str(dict(Counter(f['status'] for f in data['features']))))
        for f in sorted(data['features'], key=lambda f: f['priority']):
            print('{id:4} {status:12} {title}'.format(**f))
        print('Next: obtain DM-FC01 ver all, parameter export and first real ULog.')
        print('GPU work: source /home/hzq/RM/AirGym_RL/activate_airgym.sh')
        parameters = read_json(ROOT / 'airgym/assets/robots/user160/parameters.json')
        model = read_json(ROOT / 'airgym/assets/robots/user160/build_report.json')
        measured = parameters['measured']['takeoff_mass_kg']
        if not math.isclose(measured, model['mass_kg'], abs_tol=1e-7):
            print('PENDING: measured mass %.3f kg; existing model %.3f kg. '
                  'Current-aircraft GPU validation is blocked until model update.'
                  % (measured, model['mass_kg']))
        print('No packages installed; no GUI, training or hardware commands started.')
    if args.command in {'check', 'doctor', 'status'}:
        return print_errors(check(ROOT, not getattr(args, 'skip_evidence', False)), 'Harness consistency')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, StopIteration) as exc:
        print('FAIL ' + str(exc), file=sys.stderr)
        sys.exit(1)
