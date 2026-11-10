"""Behavioral checks for evidence freshness and incomplete trial rejection.

All fixtures are temporary synthetic files, never real flight evidence.
"""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('project_harness', Path(__file__).resolve().parents[1] / 'tools/harness.py')
h = importlib.util.module_from_spec(spec)
spec.loader.exec_module(h)


class HarnessTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        (self.root / 'tools').mkdir()
        (self.root / 'tools/harness.py').write_text('# synthetic fixture\n')
        self.assets = self.root / 'airgym/assets/robots/user160'
        self.assets.mkdir(parents=True)
        self.write_robot()

    def tearDown(self):
        self.temp.cleanup()

    def write_robot(self, mass=0.160, joint_type='fixed'):
        text = '<robot name="fixture"><link name="base_link"><inertial><mass value="%s"/></inertial><visual><geometry><mesh filename="mesh.obj"/></geometry></visual></link>' % mass
        for i in range(1, 5):
            text += '<joint name="base_link2prop_%d" type="%s"/>' % (i, joint_type)
        text += '</robot>'
        (self.assets / 'model.urdf').write_text(text)
        (self.assets / 'mesh.obj').write_text('v 0 0 0\n')
        (self.assets / 'assembled_visual.stl').write_text('synthetic fixture')
        (self.root / 'docs').mkdir(exist_ok=True)
        h.write_json(self.root / 'docs/user_aircraft_assembly_step_validation.json', {
            'passed': True, 'profile': 'user160_cad_v1',
            'urdf_sha256': h.sha(self.assets / 'model.urdf'),
            'assembled_visual_sha256': h.sha(self.assets / 'assembled_visual.stl'),
        })

    def feature(self):
        return {'id': 'A01', 'priority': 1, 'area': 'model', 'title': 'fixture',
                'user_visible_behavior': 'fixture only', 'verification': ['fixture'],
                'status': 'passing', 'evidence': 'runs/harness/fixture/evidence.json'}

    def add_evidence_fixture(self, feature):
        directory = self.root / 'runs/harness/fixture'
        directory.mkdir(parents=True)
        log = directory / 'step.log'
        log.write_text('Synthetic unit-test fixture, not actual verification\n')
        record = {'feature_id': feature['id'], 'passed': True, 'python': sys.executable,
                  'inputs': h.inputs_for(self.root, feature), 'commands': [{
                      'argv': [sys.executable] + h.RECIPES['A01'][0], 'returncode': 0,
                      'log': str(log.relative_to(self.root)), 'log_sha256': h.sha(log)}]}
        h.write_json(directory / 'evidence.json', record)
        return record, log

    def test_training_model_rejects_rotating_display_joints(self):
        self.assertEqual(h.check_assets(self.root), [])
        self.write_robot(joint_type='continuous')
        self.assertTrue(any('fixed' in x for x in h.check_assets(self.root)))

    def test_rejects_wrong_mass_and_missing_mesh(self):
        self.write_robot(mass=0.17)
        self.assertTrue(any('0.160' in x for x in h.check_assets(self.root)))
        (self.assets / 'mesh.obj').unlink()
        self.assertTrue(any('Missing mesh' in x for x in h.check_assets(self.root)))

    def test_old_simulation_mass_cannot_validate_corrected_aircraft(self):
        h.write_json(self.assets / 'parameters.json', {'measured': {'takeoff_mass_kg': .205}})
        h.write_json(self.assets / 'build_report.json', {'mass_kg': .160})
        with self.assertRaisesRegex(ValueError, 'differs from measured'):
            h.require_current_simulation_mass(self.root)
        h.write_json(self.assets / 'build_report.json', {'mass_kg': .205})
        h.require_current_simulation_mass(self.root)

    def test_cannot_mark_passing_without_execution_evidence(self):
        errors = h.validate_features(self.root, {'schema_version': 1, 'features': [self.feature()]})
        self.assertTrue(errors)

    def test_changed_input_or_log_invalidates_evidence(self):
        feature = self.feature()
        record, log = self.add_evidence_fixture(feature)
        data = {'schema_version': 1, 'features': [feature]}
        self.assertEqual(h.validate_features(self.root, data), [])
        log.write_text('Changed')
        self.assertTrue(any('log' in x for x in h.validate_features(self.root, data)))
        log.write_text('Synthetic unit-test fixture, not actual verification\n')
        (self.assets / 'mesh.obj').write_text('Changed mesh')
        self.assertTrue(any('stale' in x for x in h.validate_features(self.root, data)))

    def test_failed_or_different_command_cannot_pass(self):
        feature = self.feature()
        record, _ = self.add_evidence_fixture(feature)
        record['commands'][0]['returncode'] = 1
        h.write_json(self.root / feature['evidence'], record)
        self.assertTrue(h.validate_features(self.root, {'schema_version': 1, 'features': [feature]}))

    def test_rejects_multiple_active_features(self):
        a = self.feature()
        a['status'] = 'in_progress'
        b = dict(a, id='OTHER')
        errors = h.validate_features(self.root, {'schema_version': 1, 'features': [a, b]})
        self.assertTrue(any('More than one' in x for x in errors))

    def test_rejects_path_and_symlink_escape(self):
        for name in ('../outside', '/tmp/outside'):
            with self.assertRaises(ValueError):
                h.local_path(self.root, name)
        (self.root / 'escape').symlink_to('/tmp', target_is_directory=True)
        with self.assertRaises(ValueError):
            h.local_path(self.root, 'escape/outside')

    def test_blank_trial_is_rejected_and_existing_capture_is_preserved(self):
        folder = self.root / 'data/templates'
        folder.mkdir(parents=True)
        template = Path(__file__).resolve().parents[1] / 'data/templates/impact_trial.json'
        (folder / 'impact_trial.json').write_bytes(template.read_bytes())
        capture = h.new_trial(self.root, 'hover_001')
        self.assertTrue(h.check_trial(capture))
        self.assertFalse((capture / 'flight.ulg').exists())
        with self.assertRaises(FileExistsError):
            h.new_trial(self.root, 'hover_001')
        with self.assertRaises(ValueError):
            h.new_trial(self.root, '../escape')


if __name__ == '__main__':
    unittest.main()
