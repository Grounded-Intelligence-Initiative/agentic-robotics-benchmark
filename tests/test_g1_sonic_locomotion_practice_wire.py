"""Public practice must reject the same malformed solver replies as the formal host."""
import importlib.util
from pathlib import Path
import sys

import pytest


PAYLOAD = (Path(__file__).resolve().parents[1] / "tasks" / "humanoid" / "g1_sonic_locomotion"
           / "environment" / "setup" / "payload")
spec = importlib.util.spec_from_file_location("locomotion_practice_wire", PAYLOAD / "practice_wire.py")
wire = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wire)


@pytest.mark.parametrize("mode,error", [
    ("ok", None),
    ("bad_reset", "reset requires"),
    ("extra_field", "act requires"),
    ("oversize", "65,536 characters"),
])
def test_practice_reply_contract(tmp_path, mode, error):
    controller = tmp_path / "controller.py"
    controller.write_text(f"""
import json, sys
mode = {mode!r}
print("READY", flush=True)
for line in sys.stdin:
    kind = json.loads(line)["type"]
    msg = {{"ok": True}}
    if kind == "reset" and mode == "bad_reset":
        msg = {{"ok": 1}}
    elif kind == "act":
        msg = {{"action": {{"rows": [{{"codes": [16] * 64}}]}}}}
        if mode == "extra_field":
            msg["extra"] = 1
    reply = json.dumps(msg)
    if kind == "act" and mode in ("ok", "oversize"):
        reply = reply.ljust(65535 if mode == "ok" else 65536)
    print(reply, flush=True)
    if kind == "close":
        break
""")
    (tmp_path / "run.sh").write_text(f'exec "{sys.executable}" -u controller.py\n')

    def exercise():
        with wire.solver(tmp_path) as process:
            reply = process.request({"type": "act", "t": 0, "obs": {"horizon": 1250}})
            assert reply["action"]["rows"][0]["codes"] == [16] * 64

    if error:
        with pytest.raises(wire.WireError, match=error):
            exercise()
    else:
        exercise()
