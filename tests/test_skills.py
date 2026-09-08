"""Skill definitions are well-formed: every SKILL.md has valid YAML frontmatter with a
`name` + `description`, the onboarding skills exist, and the router references each mode
skill. A malformed skill (bad frontmatter, missing hand-off) should fail CI here rather
than silently mis-load in an agent."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

SKILLS_DIR = Path(__file__).resolve().parent.parent / "skills"

# The onboarding router + one skill per eval mode.
ONBOARDING_SKILLS = {
    "onboard-task",
    "onboard-open-loop",
    "onboard-closed-loop",
    "onboard-artifact-rollout",
}


def _all_skill_files() -> list[Path]:
    return sorted(SKILLS_DIR.rglob("SKILL.md"))


def _frontmatter(text: str) -> dict:
    """Parse the leading `---`-delimited YAML frontmatter block of a SKILL.md."""
    if not text.startswith("---"):
        raise ValueError("no frontmatter fence")
    _, fm, _ = text.split("---", 2)
    data = yaml.safe_load(fm)
    if not isinstance(data, dict):
        raise ValueError("frontmatter is not a mapping")
    return data


def test_onboarding_skills_present():
    names = {p.parent.name for p in _all_skill_files() if p.parent != SKILLS_DIR}
    missing = ONBOARDING_SKILLS - names
    assert not missing, f"missing onboarding skills: {sorted(missing)}"


@pytest.mark.parametrize("skill_file", _all_skill_files(), ids=lambda p: str(p.parent.name or "root"))
def test_skill_frontmatter_valid(skill_file: Path):
    fm = _frontmatter(skill_file.read_text(encoding="utf-8"))
    assert isinstance(fm.get("name"), str) and fm["name"].strip(), f"{skill_file}: missing name"
    assert isinstance(fm.get("description"), str) and fm["description"].strip(), (
        f"{skill_file}: missing description")


def test_router_hands_off_to_every_mode_skill():
    router = (SKILLS_DIR / "onboard-task" / "SKILL.md").read_text(encoding="utf-8")
    for mode_skill in ("onboard-open-loop", "onboard-closed-loop", "onboard-artifact-rollout"):
        assert mode_skill in router, f"router does not reference {mode_skill}"


def test_router_declares_give_up_and_the_gate():
    router = (SKILLS_DIR / "onboard-task" / "SKILL.md").read_text(encoding="utf-8").lower()
    # The router owns the shared abort rules and names the admission gate. A skill that
    # cites the wrong gate sends an agent to run a command that no longer exists.
    assert "give-up" in router or "give up" in router, "router missing give-up rules"
    assert "ale validate" in router, "router does not name the `ale validate` gate"
    assert "harness.ale_onboard" in router, "router does not name the onboarding entry point"
    assert "lint_domain" in router, "router does not name the domain lint"


def test_no_skill_cites_the_retired_gate():
    """The old shape's vocabulary must not survive in a skill.

    These names are load-bearing instructions, not prose: an agent told to run
    `harness/run_task.py --agent example`, or to write a `grader/` directory subclassing
    `grader_protocol`, produces a task the engine cannot load.
    """
    retired = ("run_task.py", "--agent example", "grader_protocol", "eval.mode",
               "description.md", "practice_grader/grade.py at the task root",
               # the pre-core/v1 layout: the visible channel is setup/payload/, the
               # shared machinery is vendored under verify/robotics_grader/, every
               # task owns image/Dockerfile, and the grader entry is verify/verify.py
               "files/", "kits/", "ale-domain/kits", "ale-domain/images",
               "robotics-cli", "verify/grade.py",
               # the retired reproduction / open-ended genre split (design 20260902 §8)
               "genre", "reproduction", "open_ended", "open-ended", "faithful", "\"probe\": true")
    for skill_file in SKILLS_DIR.rglob("SKILL.md"):
        if skill_file.parent.name not in ONBOARDING_SKILLS:
            continue
        text = skill_file.read_text(encoding="utf-8")
        for name in retired:
            assert name not in text, f"{skill_file}: still cites the retired {name!r}"


def test_skills_name_the_current_layout():
    """The load-bearing paths of the core/v1 shape must be spelled out, so an agent
    following a skill lands files where the engine and the domain lint look for them."""
    for skill_file in SKILLS_DIR.rglob("SKILL.md"):
        if skill_file.parent.name not in ONBOARDING_SKILLS:
            continue
        text = skill_file.read_text(encoding="utf-8")
        for current in ("setup/payload", "verify/verify.py", "verify/robotics_grader",
                        "image/Dockerfile"):
            assert current in text, f"{skill_file}: does not name {current!r}"


def test_skills_name_the_engine_oracle_gate():
    """Engine >= 90a7c1c fails validation when any oracle reward is below one
    (`oracle_not_full`); the old `partial_oracle` warning and the "domain policy" framing
    are gone. A skill that still calls the rule a warning teaches an agent to accept a
    broken oracle; one that trusts the exit status alone mistakes the null-anchor bootstrap
    run for a broken task, so every skill also names the record reader."""
    for skill_file in SKILLS_DIR.rglob("SKILL.md"):
        if skill_file.parent.name not in ONBOARDING_SKILLS:
            continue
        # Prose wraps at ~95 columns, so match on whitespace-collapsed text.
        text = " ".join(skill_file.read_text(encoding="utf-8").split())
        assert "oracle_not_full" in text, f"{skill_file}: does not name the engine's failure"
        assert "harness/ale_onboard.py" in text, f"{skill_file}: does not name the record reader"
        for stale in ("partial_oracle", "domain policy", "only warns", "engine itself only"):
            assert stale not in text, f"{skill_file}: still says {stale!r}"
