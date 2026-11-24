"""document_claims.py: the numbers the agent-facing documents state, checked against the code.

    python document_claims.py

Maintainers' tooling (nothing runs it during grading). Each claim is a piece of text that must appear in a
document, built from the constant or the computation it states (cell_data.py, arm_model.py, submission_format.py,
the lab's configuration and plugin). A claim whose text is missing is reported: either the document or the code
changed. Figures measured from units (document_figures.py) and from the reference's runs (reference_check.py)
are listed at the end with their source, for a reader to compare by hand.
"""
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.join(_HERE, "..", "..")
sys.path.insert(0, os.path.join(TASK, "tests"))

import numpy as np  # noqa: E402

import arm_model as am  # noqa: E402
import cell_data as cd  # noqa: E402
import submission_format as sf  # noqa: E402

sys.path.append(os.path.join(TASK, "environment", "setup", "lab"))
import calib_plugin as cp  # noqa: E402  (the lab's plugin: no execute)

D = am.D
DOCS = {k: open(os.path.join(TASK, p), encoding="utf-8").read() for k, p in (
    ("instruction", "instruction.md"), ("cell", "environment/setup/payload/cell.md"),
    ("camera", "environment/setup/payload/camera.md"), ("defects", "environment/setup/payload/defects.md"))}
LAB = json.load(open(os.path.join(TASK, "environment/setup/lab/lab.json")))
# no acceptance check (cell_data.PRACTICE_CHECKS = 0): the plugin has no execute method and no plan validator, so
# the lab's server (alelab/server.py: ops offered by attribute) does not offer the op and answers it `unsupported`
SERVER = open(os.path.join(TASK, "environment", "setup", "lab", "alelab", "server.py")).read()
NO_CHECK = (cd.PRACTICE_CHECKS == 0 and not hasattr(cp.CalibLab, "execute") and not hasattr(cp.CalibLab, "validate_plan")
            and not LAB.get("min_reply_s") and '"unsupported": "this lab does not support that op"' in SERVER
            and 'raise ReplyError("unsupported")' in SERVER)
N_GRADED = len(json.load(open(os.path.join(TASK, "tests/hidden/units.json")))["graded"])


def cc_step():
    """The contact check's first sampling step (calib_cell.JOB_STEP, hidden: read from the file, not imported)."""
    import re
    text = open(os.path.join(TASK, "tests", "calib_cell.py")).read()
    return float(re.search(r"^JOB_STEP = ([0-9.e-]+)", text, re.M).group(1))


WORDS = {1: "one", 2: "two", 3: "three", 4: "four", 5: "five"}


def mm(x, digits=2):
    return "{:.{}f}".format(x * 1e3, digits)


env_r, env_top = cd.plate_envelope()
# cell.md, section 11, the end: the cold program's displacement at the bores on the warm unit, typically from the
# first to the second figure, the median about the third (mm; document_figures.py F12, 5 %, 95 % and the median over
# every bore of the units planned: 0.14, 1.10, 0.51 on 36 units)
WARM_DISPLACEMENT_MM = (0.1, 1.1, 0.50)
# cell.md, section 12: the stylus's largest displacement on the drawing (mm; joints 0 and 1: 4.27, 4.55), the serviced
# pin's displacement at the bores from the second to the third figure, the median about the fourth (mm; 5 %, 95 %,
# median over every bore of 40 units: 0.58, 2.57, 1.19; above the clearance on 40 of 40; document_figures.py F13)
SERVICE_DISPLACEMENT_MM = (4.5, 0.6, 2.6, 1.2)
# cell.md, section 12: the same displacement anywhere inside the joint limits (mm): the largest change of the joint
# angle, (0.25 + 0.4 * 0.06) degrees, times the stylus's largest distance from joint 0's axis on the drawing (about
# 1.06 m): 5.07; 20000 random poses inside the limits reach 5.06 (joint 0) and 4.83 (joint 1)
SERVICE_REACH_MM = 5.1
CLAIMS = [
    # the plate (cell.md, section 10; instruction.md, facts)
    ("cell", "`PLATE_SIZE` = {:.0f} x {:.0f} x {:.0f} mm".format(*(np.array(cd.PLATE_SIZE) * 1e3))),
    ("cell", "|x| ≤ {:.0f} mm, |y| ≤ {:.0f} mm, -{:.0f} ≤ z ≤ 0 mm".format(cd.PLATE_SIZE[0] * 500, cd.PLATE_SIZE[1] * 500,
                                                                    cd.PLATE_SIZE[2] * 1e3)),
    ("cell", "`N_BORES` = {} blind bores of diameter {} mm (`BORE_RADIUS` = {} mm) and depth".format(
        cd.N_BORES, mm(2 * cd.BORE_RADIUS), mm(cd.BORE_RADIUS))),
    ("cell", "{:.0f} mm (`BORE_DEPTH`".format(cd.BORE_DEPTH * 1e3)),
    ("cell", "| x | " + " | ".join("{:.0f}".format(v) for v in cd.BORES[:, 0] * 1e3) + " |"),
    ("cell", "| y | " + " | ".join("{:.0f}".format(v) for v in cd.BORES[:, 1] * 1e3) + " |"),
    ("cell", "radius {:.0f} mm (`SHIM_RADIUS`)".format(cd.SHIM_RADIUS * 1e3)),
    ("cell", ", ".join("({:.0f}, {:.0f})".format(*s) for s in cd.SHIMS * 1e3) + " mm of the bottom face"),
    ("cell", "`PLATE_XY_NOMINAL` = ({:.3f}, {:.3f}) m ± `PLATE_XY_TOL` = {:.0f} mm".format(
        *cd.PLATE_XY_NOMINAL, cd.PLATE_XY_TOL * 1e3)),
    ("cell", "`PLATE_YAW_TOL` = {:.0f} degrees of `PLATE_YAW_NOMINAL` = {:.0f} degrees".format(
        cd.PLATE_YAW_TOL / D, cd.PLATE_YAW_NOMINAL / D)),
    ("cell", "`PLATE_TILT_MAX` = {:.0f} degrees".format(cd.PLATE_TILT_MAX / D)),
    ("cell", "{:.0f} to {:.0f} mm above the table (`PLATE_LIFT`".format(*(np.array(cd.PLATE_LIFT) * 1e3))),
    ("cell", "radius {:.3f} m about ({:.3f}, {:.3f}) m, from the table up to {:.3f} m".format(env_r, *cd.PLATE_XY_NOMINAL,
                                                                                         env_top)),
    ("instruction", "{:.0f} × {:.0f} × {:.0f} mm on three shims".format(*(np.array(cd.PLATE_SIZE) * 1e3))),
    ("instruction", "within {:.0f} mm of\n  ({:.3f}, {:.3f}) m on each axis".format(cd.PLATE_XY_TOL * 1e3, *cd.PLATE_XY_NOMINAL)),
    ("instruction", "yaw within {:.0f} degrees of {:.0f} degrees".format(cd.PLATE_YAW_TOL / D, cd.PLATE_YAW_NOMINAL / D)),
    ("instruction", "up to {:.0f} degrees about each".format(cd.PLATE_TILT_MAX / D)),
    ("instruction", "{:.0f} to {:.0f} mm above the table".format(*(np.array(cd.PLATE_LIFT) * 1e3))),
    ("instruction", "radius {:.3f} m about ({:.3f}, {:.3f}) m, up to\n  {:.3f} m above the table".format(
        env_r, *cd.PLATE_XY_NOMINAL, env_top)),
    # the pin and the job
    ("cell", "pin of diameter {} mm (`PIN_RADIUS` = {} mm".format(mm(2 * cd.PIN_RADIUS), mm(cd.PIN_RADIUS))),
    ("cell", "`PIN_LENGTH` = {:.0f} mm".format(cd.PIN_LENGTH * 1e3)),
    ("instruction", "pin of diameter {} mm".format(mm(2 * cd.PIN_RADIUS))),
    ("instruction", "the bores are {} mm".format(mm(2 * cd.BORE_RADIUS))),
    ("cell", "radial clearance between the pin and a bore is {} mm".format(mm(cd.BORE_RADIUS - cd.PIN_RADIUS))),
    ("instruction", "radial clearance in a bore is {} mm".format(mm(cd.BORE_RADIUS - cd.PIN_RADIUS))),
    ("cell", "`JOB_MAX_ROWS` = {}".format(cd.JOB_MAX_ROWS)),
    ("instruction", "1 to {} rows of 6 numbers".format(cd.JOB_MAX_ROWS)),
    ("cell", "`JOB_SPEED` = {} rad/s".format(cd.JOB_SPEED)),
    ("cell", "`SETTLE` = {} s: it takes max_j |Δq_j| / {} + {} s".format(cd.SETTLE, cd.JOB_SPEED, cd.SETTLE)),
    ("instruction", "joint move at {} rad/s plus {} s".format(cd.JOB_SPEED, cd.SETTLE)),
    ("cell", "every link at {:.0f} °C\n  (`T_AMBIENT`)".format(cd.T_AMBIENT)),
    ("cell", "`S_THRESHOLD` = 1e-4 rad" if cd.S_THRESHOLD == 1e-4 else "S_THRESHOLD changed"),
    ("cell", "`JOB_DEPTH` = {:.1f} mm".format(cd.JOB_DEPTH * 1e3)),
    ("instruction", "between {:.1f} mm below the mouth and the bore's bottom".format(cd.JOB_DEPTH * 1e3)),
    ("cell", "{} mm (`BORE_RADIUS`) from the bore's axis".format(mm(cd.BORE_RADIUS))),
    ("cell", "the bore's bottom ({:.0f} mm below it)".format(cd.BORE_DEPTH * 1e3)),
    ("cell", "max_j |Δq_j| / {} + 1 poses".format(0.002 if cc_step() == 0.002 else "JOB_STEP changed")),
    ("cell", "about {:.0f} rad of joint travel".format(cd.JOB_MAX_POSES * cc_step())),
    ("instruction", "every {} rad of a row's farthest-moving joint counts".format(cc_step())),
    ("instruction", "`ops` (`hello`, `budget`, `close`, `reset`, `step`)"
     if NO_CHECK and callable(getattr(cp.CalibLab, "reset", None)) else "the lab offers execute"),
    ("instruction", '"images": {}}}'.format(cd.BUDGET["images"])),
    ("instruction", "images used and left of the session's ({}; {} in a re-qualification session)".format(
        cd.BUDGET["images"], cd.BUDGET_SERVICE["images"])),
    ("instruction", "probes used and left of the session's ({}; {} in a re-qualification session)".format(
        cd.BUDGET["probes"], cd.BUDGET_SERVICE["probes"])),
    ("camera", "the session's {} images".format(cd.BUDGET["images"])),
    ("cell", "`JOB_RESOLUTION` = {:g} mm".format(cd.JOB_RESOLUTION * 1e3)),
    ("instruction", "a pin within {:g} mm of a surface".format(cd.JOB_RESOLUTION * 1e3)),
    ("cell", "`JOB_MAX_POSES` = {}".format(cd.JOB_MAX_POSES)),
    ("instruction", "more than {} poses".format(cd.JOB_MAX_POSES)),
    ("cell", "Passed** when all {} bores".format(cd.N_BORES)),
    ("cell", "stands typically {:.1f} to {:.1f} mm (median about {:.2f} mm) from where it stands on the cold\nunit".format(
        *WARM_DISPLACEMENT_MM)),
    ("cell", "centre by up to about {:.1f} mm at the acceptance conditions' poses and by up to about {:.1f} mm anywhere "
             "inside the joint limits (joints 0 and 1, the arm reaching far)".format(SERVICE_DISPLACEMENT_MM[0], SERVICE_REACH_MM)),
    ("cell", "bores typically by {:.1f} to {:.1f} mm (median about {:.1f} mm), more than the clearance on every unit".format(
        *SERVICE_DISPLACEMENT_MM[1:])),
    # the submission
    ("instruction", "keys `tip`, `camera`, `job`, `job_warm` and `job_serviced`"
     if sf.KEYS == ("tip", "camera", "job", "job_warm", "job_serviced") else "submission_format.KEYS changed"),
    ("instruction", "exactly {}\nkeys:".format({3: "three", 4: "four", 5: "five"}.get(len(sf.KEYS), "?"))),
    ("instruction", "`job_serviced`: 1 to {} rows of 6 each".format(cd.JOB_MAX_ROWS)),
    ("instruction", "`unsupported` (an `execute`\nrequest: the lab has no such operation)" if NO_CHECK else "the lab offers execute"),
    # the service and the re-qualification session (cell.md, section 12; instruction.md: budgets, lab, facts)
    ("cell", "one of joints {} to {} (`SERVICE_JOINTS`".format(min(cd.SERVICE_JOINTS), max(cd.SERVICE_JOINTS))
     if tuple(cd.SERVICE_JOINTS) == tuple(range(min(cd.SERVICE_JOINTS), max(cd.SERVICE_JOINTS) + 1)) else "SERVICE_JOINTS changed"),
    ("cell", "between {:.2f} and {:.2f} degrees ({:.3e} to {:.3e} rad, `SERVICE_ZERO`)".format(
        cd.SERVICE_ZERO[0] / D, cd.SERVICE_ZERO[1] / D, *cd.SERVICE_ZERO).replace("e-03", "e-3")),
    ("cell", "at most {:.2f} degrees plus 0.4 times `BACKLASH_MAX[j]` ({:.3f} degrees for joints 0 to 2, {:.2f}\n  degrees "
             "for joints 3 and 4)".format(cd.SERVICE_ZERO[1] / D, (cd.SERVICE_ZERO[1] + 0.4 * cd.BACKLASH_MAX[0]) / D,
                                         (cd.SERVICE_ZERO[1] + 0.4 * cd.BACKLASH_MAX[3]) / D)),
    ("cell", "`BUDGET_SERVICE`: {:.0f} s of virtual time, {} probes, {} images".format(
        cd.BUDGET_SERVICE["time_s"], cd.BUDGET_SERVICE["probes"], cd.BUDGET_SERVICE["images"])),
    ("instruction", "a re-qualification\n  session {:.0f} s, {} probes, {} images".format(
        cd.BUDGET_SERVICE["time_s"], cd.BUDGET_SERVICE["probes"], cd.BUDGET_SERVICE["images"])),
    ("instruction", "zero offset (the third rpy angle of its\n  frame) moves by {:.2f} to {:.2f} degrees".format(
        cd.SERVICE_ZERO[0] / D, cd.SERVICE_ZERO[1] / D)),
    ("instruction", "**Trials: {} for the whole job.**".format(LAB["budget"])),
    # the practice unit: one measuring session, never serviced (no re-qualification session), no check
    ("instruction", "{} + {} + {} = {}".format(N_GRADED, N_GRADED, cd.PRACTICE_SESSIONS, LAB["budget"])
     if 2 * N_GRADED + cd.PRACTICE_SESSIONS + cd.PRACTICE_CHECKS == LAB["budget"] and cd.PRACTICE_CHECKS == 0
     else "the lab's budget is not the sessions' sum"),
    ("instruction", "which leaves {} for the practice unit's {} measuring\n  session:".format(
        WORDS.get(LAB["budget"] - 2 * N_GRADED, "?"), WORDS.get(cd.PRACTICE_SESSIONS, "?"))
     if cd.PRACTICE_SESSIONS == 1 else "PRACTICE_SESSIONS changed: the wording is singular"),
    ("instruction", "there is no `p1-serviced`" if "if uid not in self.practice" in open(cp.__file__).read()
     else "the plugin lists a practice re-qualification instance"),
    ("instruction", "There is no instance `p1-serviced`: a `reset` naming it is answered `bad_instance`, which costs no trial"),
    ("cell", "The practice unit `p1` is not serviced: it is the shop's demonstration unit and is not due for service"),
    ("instruction", "The lab has no\n`execute` operation: `hello` does not list it" if NO_CHECK else "the lab offers execute"),
    ("instruction", "is\nanswered with the error `unsupported`, which costs no trial" if NO_CHECK else "the lab offers execute"),
    ("instruction", "**Nothing about\npredictions or job programs is ever reported before grading, on any unit**"
     if NO_CHECK else "PRACTICE_CHECKS changed"),
    ("instruction", "It is for **rehearsing a measuring session against the lab**"),
    ("cell", "No job program is run before grading, on any\nunit (the practice unit included)" if NO_CHECK
     else "PRACTICE_CHECKS changed"),
    ("instruction", "like a graded unit it has\n**one measuring session** (one trial)"
     if cd.PRACTICE_SESSIONS == 1 else "PRACTICE_SESSIONS changed"),
    ("instruction", "one measuring session for every unit, `p1` included; one re-qualification session for\n  every graded unit"
     if cd.PRACTICE_SESSIONS == 1 else "PRACTICE_SESSIONS changed"),
    # no images in the re-qualification session: the camera is off (cell.md, sections 8 and 12; camera.md)
    ("cell", "**No images in the re-qualification session**" if cd.BUDGET_SERVICE["images"] == 0
     else "BUDGET_SERVICE has images: the camera-off text is stale"),
    ("cell", "An `image` command in this session is refused as `no_images`"),
    ("cell", "`image` once its images are used ({}; {}: in\n  a re-qualification session from its start), `no_images`".format(
        cd.BUDGET["images"], cd.BUDGET_SERVICE["images"])),
    ("instruction", "An `image` command there is refused as\n  `no_images`"),
    ("instruction", "with a small budget of probes and no images"
     if cd.BUDGET_SERVICE["images"] == 0 else "BUDGET_SERVICE has images"),
    ("camera", "A re-qualification session\nhas no images" if cd.BUDGET_SERVICE["images"] == 0 else "BUDGET_SERVICE has images"),
    # the warm job (cell.md, section 11; instruction.md: the conditions file, grading, the facts)
    ("cell", "`JOB_WARM_TEMP` = {:.0f} to {:.0f} °C".format(*cd.JOB_WARM_TEMP)),
    ("instruction", "each between {:.0f} and {:.0f} °C (`cell.md`, section 11)".format(*cd.JOB_WARM_TEMP)),
    ("instruction", "the warm job starts at the unit's published `job_warm_temp` ({:.0f} to {:.0f} °C)".format(
        *cd.JOB_WARM_TEMP)),
    # heating and cooling (cell.md, section 7; instruction.md, facts) and the acceptance temperatures
    ("cell", "{:.2f} to {:.2f} K per radian ({:.1f} to {:.1f} times {:.2f})".format(
        cd.HEAT_NOMINAL * cd.HEAT_FACTOR_RANGE[0], cd.HEAT_NOMINAL * cd.HEAT_FACTOR_RANGE[1], *cd.HEAT_FACTOR_RANGE,
        cd.HEAT_NOMINAL)),
    ("cell", "{:.0f} to {:.0f} s, one per unit".format(*cd.TAU_COOL_RANGE)),
    ("instruction", "({:.2f} to {:.2f} K per radian, per joint and cell".format(
        cd.HEAT_NOMINAL * cd.HEAT_FACTOR_RANGE[0], cd.HEAT_NOMINAL * cd.HEAT_FACTOR_RANGE[1])),
    ("instruction", "time constant of {:.0f} to\n  {:.0f} s (per cell".format(*cd.TAU_COOL_RANGE)),
    ("instruction", "cold (20 to 22 °C) and half warm ({:.0f} to {:.0f} °C)".format(*cd.ACCEPT_TEMP[1])
     if cd.ACCEPT_TEMP[0] == (20.0, 22.0) else "ACCEPT_TEMP changed"),
    ("cell", "every link between {:.0f} and {:.0f} °C (`ACCEPT_TEMP`)".format(*cd.ACCEPT_TEMP[1])),
    ("instruction", "(every link between\n{:.0f} and {:.0f} °C)".format(*cd.ACCEPT_TEMP[1])),
    ("instruction", "at most {} bytes (`MAX_BYTES`)".format(sf.MAX_BYTES)),
    ("instruction", "magnitude at most {:.0f} (`LIMIT`)".format(sf.LIMIT)),
    ("instruction", "Every accepted `reset`, on any unit, costs one" if NO_CHECK else "the lab charges something else"),
    ("instruction", "\"job\": [[{}], ...]".format(", ".join("{:.4f}".format(v) if v else "0.0" for v in cd.Q_HOME))),
    # acceptance limits and budget (unchanged, rechecked)
    ("instruction", "95th percentile at most **{:.2f} mm**, largest at most **{:.2f} mm**".format(
        cd.ACCEPT_P95 * 1e3, cd.ACCEPT_MAX * 1e3)),
    ("instruction", "95th percentile at most **{:.2f} mm**,\n  largest at most **{:.2f} mm**".format(
        cd.ACCEPT_CAM_P95 * 1e3, cd.ACCEPT_CAM_MAX * 1e3)),
    ("instruction", "1000 s of virtual time, {} probes, {} images".format(cd.BUDGET["probes"], cd.BUDGET["images"])),
    ("cell", "| virtual time | {:.0f} s | {:.0f} s |".format(cd.BUDGET["time_s"], cd.BUDGET_SERVICE["time_s"])),
    ("cell", "| probes | {} | {} |".format(cd.BUDGET["probes"], cd.BUDGET_SERVICE["probes"])),
    ("cell", "| images | {} | {} (the camera is off) |".format(cd.BUDGET["images"], cd.BUDGET_SERVICE["images"])),
    ("instruction", "## No acceptance check before grading" if NO_CHECK else "PRACTICE_CHECKS changed"),
    ("instruction", "`trial_step_cap` {}".format(LAB["trial_step_cap"])),
    ("instruction", "`trial_wall_s` {}".format(LAB["trial_wall_s"])),
]

MEASURED = [
    ("camera.md: top face about 2 to 4.5 times as bright as the table; bores' inside a few percent of the top face",
     "document_figures.py F11 (5 %..95 % over units: 2.07..4.42; bores 0.019..0.060)"),
    ("cell.md section 11 and instruction.md: a program that keeps the pin clear needs a few thousand poses",
     "reference_check.py (the reference's programs: median and largest poses examined)"),
    ("cell.md section 11: for the same joint targets the pin of a unit at its warm start stands typically {} to {} mm "
     "(median about {} mm) from where it stands cold, more than the clearance on most units".format(*WARM_DISPLACEMENT_MM),
     "document_figures.py F12 (every bore 5 % .. 95 %: 0.14 .. 1.10, median 0.51; per unit the largest above the "
     "clearance on 32 of 36 units) and reference_check.py (disp, and the wcold baseline: 5 of 64 units pass)"),
    ("cell.md section 12: the stylus moves by up to about {} mm; the pin at the bores typically {} to {} mm (median "
     "about {} mm), more than the clearance on every unit".format(*SERVICE_DISPLACEMENT_MM),
     "document_figures.py F13 ((a) joints 0 and 1: 4.27 and 4.55 mm; (b) every bore 5 % .. 95 %: 0.58 .. 2.57, median "
     "1.19; the largest per unit above the clearance on 40 of 40 units) and reference_check.py (the snone baseline)"),
]


def section(doc, start, end=None):
    text = DOCS[doc]
    i = text.index(start)
    j = text.index(end, i) if end else len(text)
    return text[i:j]


def numbers_of(text):
    """The numbers written in a piece of text (backticked names, joint indices like q_j and k45 left out)."""
    import re
    text = re.sub(r"`[^`]*`", " ", text)
    text = re.sub(r"[A-Za-z_]+[0-9]+", " ", text)
    return [float(t) for t in re.findall(r"(?<![\w.])-?\d+(?:\.\d+)?(?:e-?\d+)?", text)]


def allowed_job_numbers():
    """Every value sections 10 and 11 of cell.md and the plate's and job's facts in instruction.md may state, from
    the constants (in the units the documents use) and the figures measured by document_figures.py F11."""
    v = set()
    v |= {x * 1e3 for x in cd.PLATE_SIZE} | {x * 500 for x in cd.PLATE_SIZE} | {cd.PLATE_SIZE[2] * -1e3}
    v |= {cd.N_BORES, 2e3 * cd.BORE_RADIUS, 1e3 * cd.BORE_RADIUS, 1e3 * cd.BORE_DEPTH}
    v |= set(np.round(cd.BORES.ravel() * 1e3, 6)) | set(np.round(cd.SHIMS.ravel() * 1e3, 6)) | {cd.SHIM_RADIUS * 1e3}
    v |= set(cd.PLATE_XY_NOMINAL) | {cd.PLATE_XY_TOL * 1e3, cd.PLATE_YAW_TOL / D, cd.PLATE_YAW_NOMINAL / D,
                                     cd.PLATE_TILT_MAX / D} | {x * 1e3 for x in cd.PLATE_LIFT}
    v |= set(cd.plate_envelope())
    v |= {2e3 * cd.PIN_RADIUS, 1e3 * cd.PIN_RADIUS, 1e3 * cd.PIN_LENGTH, cd.JOB_MAX_ROWS, cd.JOB_SPEED, cd.SETTLE,
          cd.T_AMBIENT, cd.S_THRESHOLD, cd.JOB_DEPTH * 1e3, cd.JOB_RESOLUTION * 1e3, cd.JOB_MAX_POSES,
          1e3 * (cd.BORE_RADIUS - cd.PIN_RADIUS), 0.002, cd.JOB_MAX_POSES * 0.002}
    v |= {1.6, 14.4}                       # document_figures.py F11: the bottom face's corners (mm)
    v |= set(cd.JOB_WARM_TEMP)
    v |= set(WARM_DISPLACEMENT_MM)         # document_figures.py F12: the cold program's displacement when warm (mm)
    v |= {-1, 1, 0, 6}                     # signs, "one row", "six numbers"
    return v


def check_numbers():
    """Each number in the job's and the plate's sections must be one the constants give."""
    ok = allowed_job_numbers()
    texts = [("cell.md §10-11", section("cell", "## 10. The gauge plate", "## 12. The service")),
             ("instruction.md plate fact", section("instruction", "- **The gauge plate is never touched.**", "- **Knocked")),
             ("instruction.md jobs fact", section("instruction", "- **The jobs** (", "- **The service**"))]
    bad = 0
    for name, text in texts:
        nums = numbers_of(text.replace("## 11. The job", "").replace("section 10", "").replace("section 11", "").replace("section 5", "")
                          .replace("section 6", "").replace("section 7", "").replace("| bore | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |", ""))
        unknown = sorted({x for x in nums if not any(abs(x - a) <= 1e-9 * max(1.0, abs(a)) for a in ok)})
        bad += len(unknown)
        print("{:<28} {} numbers, not from the constants: {}".format(name, len(nums), unknown or "none"))
    ok = allowed_job_numbers() | allowed_service_numbers()
    for name, text in (("cell.md §12", section("cell", "## 12. The service")),
                       ("instruction.md service fact", section("instruction", "- **The service**", "- **The camera"))):
        for sec in ("sections 5 to 8", "section 2", "section 3", "section 7", "section 8", "section 11", "section 12",
                    "## 12."):
            text = text.replace(sec, "")
        nums = numbers_of(text)
        unknown = sorted({x for x in nums if not any(abs(x - a) <= 1e-9 * max(1.0, abs(a)) for a in ok)})
        bad += len(unknown)
        print("{:<28} {} numbers, not from the constants: {}".format(name, len(nums), unknown or "none"))
    return bad


def allowed_service_numbers():
    """The values section 12 of cell.md and the service's fact in instruction.md may state."""
    v = set(cd.SERVICE_JOINTS) | {max(cd.SERVICE_JOINTS) + 1}
    v |= {cd.SERVICE_ZERO[0] / D, cd.SERVICE_ZERO[1] / D, round(cd.SERVICE_ZERO[0], 6), round(cd.SERVICE_ZERO[1], 6)}
    v |= {0.2, 1.0, 0.4, 2}                  # the backlash's range (section 3), the largest change of half of it, joint 2
    v |= {round((cd.SERVICE_ZERO[1] + 0.4 * cd.BACKLASH_MAX[j]) / D, 3) for j in range(6)}
    v |= set(cd.BUDGET_SERVICE.values()) | {cd.T_AMBIENT, cd.N_BORES, 1}
    v |= set(SERVICE_DISPLACEMENT_MM)       # document_figures.py F13
    v |= {SERVICE_REACH_MM}                 # the largest change at the arm's full reach
    return v


def main():
    bad = check_numbers()
    for doc, text in CLAIMS:
        ok = text in DOCS[doc]
        bad += not ok
        print("{:>4}  {:<12} {}".format("ok" if ok else "MISS", doc, text.replace("\n", " ")))
    print("measured figures (compare by hand):")
    for claim, source in MEASURED:
        print("      {}  <-  {}".format(claim, source))
    print("claims missing: {}".format(bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
