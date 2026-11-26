"""Linux integration regressions using real live and reaped worker processes."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cpu_meter import CpuMeter

WORKER = """
import json, resource, sys, time
start = time.process_time()
while time.process_time() - start < 0.25:
    pass
print(json.dumps({'worker_cpu': time.process_time() - start}), flush=True)
if '--stay' in sys.argv:
    input()
"""
CONTROLLER = """
import json, resource, subprocess, sys
worker = json.loads(sys.argv[1])
print('READY', flush=True)
for line in sys.stdin:
    command = line.strip()
    if command == 'live':
        child = subprocess.Popen([sys.executable, '-c', worker, '--stay'],
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        print(child.stdout.readline().strip(), flush=True)
    elif command == 'reap':
        child.communicate('exit\\n', timeout=5)
        print('REAPED', flush=True)
    else:
        code = worker
        if command == 'nested':
            code = 'import subprocess,sys; subprocess.run([sys.executable,"-c",' + repr(worker) + '],check=True)'
        subprocess.run([sys.executable, '-c', code], check=True, stdout=subprocess.DEVNULL)
        print('REAPED', flush=True)
"""


@unittest.skipUnless(sys.platform == "linux", "requires Linux /proc accounting")
class CpuAccountingTests(unittest.TestCase):
    def setUp(self):
        self.controller = subprocess.Popen(
            [sys.executable, "-c", CONTROLLER, json.dumps(WORKER)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
            start_new_session=True,
        )
        self.assertEqual(self.controller.stdout.readline().strip(), "READY")
        self.meter = CpuMeter(self.controller.pid)

    def tearDown(self):
        os.killpg(self.controller.pid, signal.SIGKILL)
        self.controller.wait(timeout=5)
        self.controller.stdin.close()
        self.controller.stdout.close()

    def ask(self, command):
        self.controller.stdin.write(command + "\n")
        self.controller.stdin.flush()
        return self.controller.stdout.readline().strip()

    def assert_reaped_cpu(self, command):
        before = self.meter.seconds()
        self.assertEqual(self.ask(command), "REAPED")
        measured = self.meter.seconds() - before
        self.assertGreaterEqual(measured, 0.23)
        self.assertLess(measured, 0.5)
        print(json.dumps({"case": command, "measured_cpu_seconds": measured}), flush=True)

    def test_worker_born_and_reaped_between_samples(self):
        self.assert_reaped_cpu("worker")

    def test_waited_grandchild_time_is_retained(self):
        self.assert_reaped_cpu("nested")

    def test_new_live_worker_is_counted_without_waiting_40_samples(self):
        before = self.meter.seconds()
        self.assertGreaterEqual(json.loads(self.ask("live"))["worker_cpu"], 0.25)
        live = self.meter.seconds()
        self.assertGreaterEqual(live - before, 0.23)
        self.assertEqual(self.ask("reap"), "REAPED")
        after = self.meter.seconds()
        self.assertGreaterEqual(after, live)
        self.assertLess(after - live, 0.08, "reaping must not count the worker twice")
        print(json.dumps({"case": "live_then_reaped", "live_cpu_seconds": live - before,
                          "post_reap_extra_seconds": after - live}), flush=True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
