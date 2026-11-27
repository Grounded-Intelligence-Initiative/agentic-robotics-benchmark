"""Reference provenance and pinned-source checks; no performance rollout."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import reference


@unittest.skipUnless(os.geteuid() == 0, 'requires disposable root test container')
class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.trajectory = self.root / 'trajectory.json'
        self.enterContext(patch.object(reference, 'TRAJECTORY_PATH', self.trajectory))
        self.enterContext(patch.dict(os.environ, {'ALE_TRAJECTORY_PATH': str(self.trajectory)}))

    def provenance(self, name='oracle'):
        self.trajectory.write_text(json.dumps({'schema_version': 'ATIF-v1.8', 'agent': {'name': name}}))

    def test_missing_provenance_and_submission_flags_do_not_authorize(self):
        with patch.dict(os.environ, {'YAM_REFERENCE': '1', 'ALE_ORACLE': '1'}):
            self.assertFalse(reference.is_reference_episode())
        with self.assertRaisesRegex(RuntimeError, 'provenance'):
            reference.TrustedReferenceSolver(submission=str(self.root))

    def test_only_engine_oracle_provenance_authorizes(self):
        self.provenance('codex-cli')
        self.assertFalse(reference.is_reference_episode())
        self.provenance()
        self.assertTrue(reference.is_reference_episode())

    def test_alternate_trajectory_path_denied(self):
        self.provenance()
        with patch.dict(os.environ, {'ALE_TRAJECTORY_PATH': str(self.root / 'fake.json')}):
            with self.assertRaisesRegex(RuntimeError, 'trajectory path'):
                reference.is_reference_episode()

    def test_malformed_or_writable_provenance_fails_closed(self):
        for value in ('garbage', '[]', '{}', '{"schema_version":"ATIF-v1.7","agent":{"name":"oracle"}}'):
            self.trajectory.write_text(value)
            with self.assertRaises(RuntimeError):
                reference.is_reference_episode()
        self.provenance()
        self.trajectory.chmod(0o666)
        with self.assertRaises(RuntimeError):
            reference.is_reference_episode()
        self.trajectory.chmod(0o644)
        self.root.chmod(0o777)
        with self.assertRaises(RuntimeError):
            reference.is_reference_episode()

    def test_provenance_symlink_denied(self):
        self.provenance()
        target = self.root / 'actual.json'
        self.trajectory.rename(target)
        self.trajectory.symlink_to(target)
        with self.assertRaises(RuntimeError):
            reference.is_reference_episode()

    def test_pinned_sources_and_tampering(self):
        source = Path(__file__).resolve().parent.parent / 'solution'
        submission = self.root / 'submission'
        submission.mkdir()
        for name in reference.RUNTIME_FILES:
            (submission / name).write_bytes((source / name).read_bytes())
        originals = reference._verified_sources(submission)
        self.assertEqual(set(originals), set(reference.RUNTIME_FILES))
        (submission / 'agent.py').write_text('raise SystemExit("tampered")')
        with self.assertRaisesRegex(RuntimeError, 'digest'):
            reference._verified_sources(submission)
        (submission / 'agent.py').unlink()
        (submission / 'agent.py').symlink_to(source / 'agent.py')
        with self.assertRaises(RuntimeError):
            reference._verified_sources(submission)

    def test_cleanup_failure_prevents_start(self):
        self.provenance()
        with patch.object(reference, 'clear_agent_account', return_value={'cleared': False}), \
                patch.object(reference.TrustedReferenceSolver, '_start') as start:
            with self.assertRaisesRegex(RuntimeError, 'account'):
                reference.TrustedReferenceSolver()
            start.assert_not_called()

    def test_pinned_controller_stdio_and_cleanup(self):
        if not Path('/opt/physics/bin/python').exists():
            self.skipTest('requires task image private physics runtime')
        self.provenance()
        source = Path(__file__).resolve().parent.parent / 'solution'
        with patch.object(reference, 'clear_agent_account', return_value={'cleared': True}):
            with reference.TrustedReferenceSolver(submission=str(source), step_timeout=15) as solver:
                copied = Path(solver.submission)
                self.assertNotEqual(copied, source)
                self.assertEqual(copied.stat().st_mode & 0o777, 0o700)
                solver.reset()
                self.assertEqual(solver.act(0, {'items': [], 'previous_item': None}), {'finish': True})
            self.assertIsNotNone(solver.proc.poll())
            self.assertFalse(copied.exists())
            solver.proc.stdout.close()


if __name__ == '__main__':
    unittest.main()
