"""Exercise the real request ledger with a stub world, without simulator/assets."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

TASK = Path(__file__).resolve().parents[2]
SERVICES = (TASK/'environment/setup/runtime/service.py', TASK/'tests/runtime/service.py')


class StubWorld:
    def __init__(self, seed):
        self.phase = 0
        self.done = False
        self.actions = []

    def metrics(self):
        return {}

    def act(self, action):
        if len(action) != (1 if self.phase == 0 else 2):
            raise ValueError('invalid action length')
        self.actions.append(action[:])
        return {'phase': self.phase}


def load_service(path):
    stub = types.ModuleType('world')
    stub.World = StubWorld
    spec = importlib.util.spec_from_file_location('service_logging_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {'world': stub}):
        spec.loader.exec_module(module)
    return module


class ServiceLoggingTests(unittest.TestCase):
    def test_scout_and_both_carrier_channels_reach_disk(self):
        for source in SERVICES:
            with self.subTest(source=source), tempfile.TemporaryDirectory() as directory:
                runtime = load_service(source).Runtime(directory, 'commissioning', seed=1001)
                scout, carrier = [2/3], [.25, -.75]
                runtime.call({'method': 'act', 'params': {'action': scout}})
                runtime.world.phase = 1
                runtime.call({'method': 'act', 'params': {'action': carrier}})
                self.assertEqual(runtime.world.actions, [scout, carrier])
                self.assertEqual([e['a'] for e in runtime.events], [scout, carrier])
                carrier[1] = 1.
                self.assertEqual(runtime.events[-1]['a'], [.25, -.75])
                ledger = json.loads((Path(directory)/'state.json').read_text())
                self.assertEqual(ledger['events'], [
                    {'m': 'act', 'a': [2/3]}, {'m': 'act', 'a': [.25, -.75]}])

    def test_invalid_action_logging_remains_bounded(self):
        for source in SERVICES:
            with self.subTest(source=source), tempfile.TemporaryDirectory() as directory:
                runtime = load_service(source).Runtime(directory, 'commissioning', seed=1001)
                runtime.world.phase = 1
                with self.assertRaisesRegex(ValueError, 'invalid action length'):
                    runtime.call({'method': 'act', 'params': {'action': [.25, -.75, .9]}})
                ledger = json.loads((Path(directory)/'state.json').read_text())
                self.assertEqual(ledger['events'][0]['a'], [.25, -.75])
                self.assertEqual(ledger['events'][0]['error'], 'invalid action length')
                self.assertEqual(runtime.world.actions, [])


if __name__ == '__main__':
    unittest.main()
