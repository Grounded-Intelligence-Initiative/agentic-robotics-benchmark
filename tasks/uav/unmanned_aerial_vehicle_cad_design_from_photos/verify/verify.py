"""Freeze the submission, extract native CAD as nobody, then evaluate geometry in the hidden stage."""
import json
import hashlib
import os
import tempfile
from pathlib import Path

import lockdown_runtime as runtime


def main():
    freezer = runtime.verify_freeze.AgentFreezer.for_user(runtime.AGENT_USER)
    early = runtime._early_lockdown(freezer)
    try:
        if early["agent_processes_left"]:
            raise RuntimeError("Agent processes survived verify-start isolation")
        # Heavy geometry imports happen only after locking the hidden stage and freezing agent processes.
        from grade import grade
        from robotics_grader import read_anchor, ratio_rewards
        stage = Path(runtime.STAGE_DIR)
        config = json.loads((stage / "grader_config.json").read_text())
        stats = {k:v for k,v in early.items() if isinstance(v,float)}
        runtime.tool_usage(stats)
        root = runtime.SNAPSHOT if early["submission_snapshot"] else runtime.SUBMISSION
        doc, note = runtime.find_document(root)
        extraction = tempfile.mkdtemp(prefix="uav_empty_")
        if doc and not early["submission_root_rejected"] and not early["submission_late_writes"]:
            extraction, tail = runtime.extract(doc, config["extract_timeout_s"], stats)
            if stats.get("extract_exit") != 0:
                raise RuntimeError("Native CAD extractor failed: " + tail[-800:])
        report = grade(extraction, stage / "reference", config)
        blocked = set(json.loads((stage / "public_blocklist.json").read_text())["files"].values())
        reused = []
        if Path(root).is_dir() and not early["submission_root_rejected"]:
            for path in Path(root).rglob("*"):
                if path.is_symlink() or not path.is_file():
                    continue
                digest = hashlib.sha256()
                with path.open("rb") as handle:
                    for block in iter(lambda: handle.read(1 << 20), b""):
                        digest.update(block)
                if digest.hexdigest() in blocked:
                    reused.append(str(path.relative_to(root)))
        report["checks"]["public_model_reuse"] = float(bool(doc) and not reused)
        if reused:
            report["design_score"] = 0.
            report["diagnostics"].append("Unchanged upstream CAD export in submission: "+", ".join(reused))
        if note:
            report["diagnostics"].append(note)
        stats.update(report["statistics"])
        stats.update(report["criteria"])
        stats.update(report["checks"])
        anchor, meta = read_anchor()
        verdict = ratio_rewards(report["design_score"], anchor, metric_name=meta["metric"],
                                higher_is_better=True, full_at=meta["full_at"], extra=stats)
        if os.environ.get("ALE_VERIFICATION_PATH"):
            from ale_verify import Verification, CheckResult
            rec = Verification()
            diagnostic = json.dumps({"criteria":report["criteria"],"diagnostics":report["diagnostics"]})
            rec.check("reward", CheckResult(verdict["rewards"]["reward"], diagnostic[:2000]))
            for k,v in verdict["metrics"].items():
                rec.stat(k,float(v))
            rec.write()
        else:
            path = os.environ.get("ALE_VERDICT_PATH", str(stage / "verdict.local.json"))
            Path(path).write_text(json.dumps(dict(verdict,details=report),indent=2)+"\n")
    finally:
        runtime._restore(early)
        freezer.stop()


if __name__ == "__main__":
    main()
