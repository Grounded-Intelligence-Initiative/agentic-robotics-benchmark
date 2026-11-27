"""Verify a static JSON map patch without executing any submitted code."""

import json
import os
from collections import Counter
from pathlib import Path

from independent import build_episode
from score import score_case


def evaluate(patches, episode):
    results = {}
    passed = Counter()
    totals = Counter()
    for case_id, case in episode.items():
        patch = patches.get(case_id)
        result = score_case(case["prior"], case["truth"], patch)
        variant = case["truth"]["variant"]
        group = "unchanged" if variant == "shoulder_work" else "changed"
        if group == "unchanged" and patch != {"close": [], "add": []}:
            result["passed"] = False
            if "diagnosis" in result:
                result["diagnosis"]["primary"] = "unnecessary_control_edit"
        totals[group] += 1
        passed[group] += int(result["passed"])
        results[case_id] = {"variant": variant, **result}
    score = (passed["changed"] / totals["changed"]) * (passed["unchanged"] / totals["unchanged"])
    causes = Counter(row.get("diagnosis", {}).get("primary", "invalid_or_missing_patch")
                     for row in results.values() if row["variant"] != "shoulder_work")
    gaps = Counter(row.get("diagnosis", {}).get("new_gap", {}).get("result", "invalid_or_missing_patch")
                   for row in results.values() if row["variant"] == "relocated_access")
    diagnostic = {"cases": results, "passed": dict(passed), "totals": dict(totals),
                  "changed_outcomes": dict(causes), "new_gap_outcomes": dict(gaps)}
    return score, diagnostic


def verify():
    from robotics_grader import ratio_rewards, read_anchor, stage_config, write_verdict, zero_verdict

    config = stage_config()
    anchor, meta = read_anchor()
    path = Path(os.environ.get("ALE_HOME", "/home/user")) / "submission/submission.json"
    try:
        if not path.is_file() or path.is_symlink():
            raise ValueError("No regular submission.json")
        with path.open(encoding="utf-8") as stream:
            document = json.load(stream)
        if not isinstance(document, dict) or set(document) != {"patches"} or not isinstance(document["patches"], dict):
            raise ValueError("Expected a JSON object with a patches object")
    except (OSError, ValueError, RecursionError) as exc:
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None))
        return

    episode = build_episode(os.environ["ALE_EPISODE_ID"])
    patches = document["patches"]
    if set(patches) - set(episode):
        write_verdict(zero_verdict(reason="Submission includes unknown case IDs", anchored=anchor is not None))
        return
    measured, detail = evaluate(patches, episode)
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5), higher_is_better=True,
                            metric_name=meta["metric"], full_at=meta["full_at"])
    write_verdict(verdict, diagnostic=json.dumps(detail, separators=(",", ":")))


if __name__ == "__main__":
    verify()
