"""Regression coverage for historical-reference normalization."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent


@pytest.fixture
def score(monkeypatch):
    monkeypatch.setenv("ALE_STAGE_DIR", str(ROOT))
    monkeypatch.syspath_prepend(str(ROOT))
    spec = importlib.util.spec_from_file_location("task_scoring", ROOT / "scoring.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.score_verdict


def test_zero_completion_scores_zero(score):
    verdict = score(0, 0, 8, 0, {"match_lock_ok": 1})
    assert verdict["rewards"]["reward"] == 0
    assert verdict["metrics"]["completion_fraction"] == 0
    assert verdict["metrics"]["ratio"] == 0
    assert verdict["metrics"]["anchor_recorded"] == 1


@pytest.mark.parametrize("left,right", [(31, 31), (32, 32), (42, 38), (48, 56)])
def test_historical_reference_band_scores_full(score, left, right):
    verdict = score(left, right, 8, 0, {"match_lock_ok": 1})
    assert verdict["rewards"]["reward"] == 1
    assert verdict["metrics"]["completion_fraction"] == pytest.approx((left + right) / 104)
    assert verdict["metrics"]["ratio"] == pytest.approx(min((left + right) / 64, 1.5))


def test_below_reference_band_keeps_continuous_reward_and_raw_metrics(score):
    verdict = score(30, 31, 8, 0, {
        "match_lock_ok": 1, "probes_total": 34, "probes_passed": 34,
    })
    assert verdict["rewards"]["reward"] == pytest.approx(61 / 62)
    assert verdict["rewards"]["reward"] < 1
    assert verdict["metrics"] == pytest.approx({
        "completion_fraction": 61 / 104, "ratio": 61 / 64, "anchor_recorded": 1,
        "left": 30, "right": 31, "left_total": 48, "right_total": 56,
        "episodes": 8, "episodes_aborted": 0, "match_lock_ok": 1,
        "probes_total": 34, "probes_passed": 34,
    })


def test_lock_failure_preserves_raw_success(score):
    verdict = score(6, 7, 1, 0, {"match_lock_ok": 0})
    assert verdict["rewards"]["reward"] == 0
    assert verdict["metrics"]["ratio"] == 0
    assert verdict["metrics"]["completion_fraction"] == 1
    assert verdict["metrics"]["anchor_recorded"] == 1


def test_aborted_rounds_stay_in_denominator(score):
    verdict = score(6, 7, 8, 7, {"match_lock_ok": 1})
    assert verdict["metrics"]["completion_fraction"] == .125
    assert verdict["metrics"]["episodes_aborted"] == 7
    assert verdict["rewards"]["reward"] == pytest.approx(13 / 62)


@pytest.mark.parametrize("left,right,episodes", [(0, 0, 0), (7, 0, 1), (0, 8, 1), (-1, 0, 1)])
def test_invalid_completion_counts_are_rejected(score, left, right, episodes):
    with pytest.raises(ValueError, match="invalid completion counts"):
        score(left, right, episodes, 0, {"match_lock_ok": 1})
