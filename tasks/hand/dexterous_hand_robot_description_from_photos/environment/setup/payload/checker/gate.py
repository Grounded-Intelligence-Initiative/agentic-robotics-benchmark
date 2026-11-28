"""gate: the format check `model_loads` of the hand description (payload-safe: public rules only).

The same code runs in the format checker the agent is given and in the grader, before anything is scored. It reads
no number of the photographed unit, so it cannot tell how accurate a description is. It holds only rules of four
kinds: loading, the element whitelist, names and size caps.

    ok, details, models = gate_model_loads(sub_dir)   # models = {"robot": MjModel, "calib": calib.Calib}

Rules (each a function below; details["violations"] lists every failure, each message starting "rule <n>"):
  rule 1  files: robot.xml and config.yaml regular files in the submission root; only regular files and directories
          below it (no symbolic link, no special file); at most MAX_SUBMISSION_BYTES (the larger of apparent and
          allocated size) and MAX_SUBMISSION_ENTRIES entries; every file= and meshdir / texturedir / assetdir a
          relative path without a ".." component; at most contract.TRI_CAP mesh triangles, counted per mesh asset
          from the files BEFORE anything is compiled (count_mesh_triangles: a binary STL by its triangle count, an OBJ
          by its faces, k - 2 triangles for a face of k corners, an MSH by its face count, inline faces by their
          number, a mesh given by vertices alone as 2 V - 4, the most its convex hull can have; a built-in mesh is
          bounded from its parameters before MuJoCo builds it), and again in the compiled model.
  rule 2  the MJCF whitelist (whitelist.py), applied before anything is compiled; <equality> holds only <connect>
          and <joint>.
  rule 3  <compiler angle="radian"> stated explicitly; robot.xml compiles within COMPILE_CPU_S seconds of CPU time
          with zero compiler warnings.
  rule 4  structure: the body palm is a jointless child of the world body; the 16 contract hinges exist, each a
          limited hinge in the body of its own name, in the chains palm > thumb_roll > thumb_side > thumb_mp >
          thumb_ip and palm > <f>_abad > <f>_mcp > <f>_pip > <f>_dip (each contract body's nearest contract
          ancestor is the one before it); jointless extra bodies anywhere; extra hinges only in bodies
          <f>_loop1 .. <f>_loop4 below <f>_abad; no joint other than a hinge.
  rule 5  coupling form: each <f>_dip is coupled either by exactly one joint equality (joint1 = <f>_dip,
          joint2 = <f>_pip, polycoef) or by at most MAX_LOOP_CONNECTS active connect equalities whose two sides lie
          on bodies of finger f, not both; a finger coupled by a joint equality has no extra hinge. An inactive
          equality couples nothing (and is not counted).
  rule 6  the four pad sites <digit>_pad, each rigid on its digit's last contract body or on a jointless body below it.
  rule 7  exactly 13 actuators act_<driven hinge>, each a position servo on its own hinge (<position>, or <general>
          with gaintype fixed, biastype affine, biasprm[1] = -gainprm[0], gainprm[0] > 0), gear 1, force-limited with
          forcerange [-f, +f], f finite and > 0.
  rule 8  every body mass finite and >= 0, the total > 0.
  rule 9  config.yaml (calib.py).
  rule 10 (grader only) nothing in the submission written after the session ended.
  rule 11 (grader only) the grader's own work on the description after rules 1 to 10 (closing each finger's chain,
          the IK of every request, every item) within GRADE_CPU_S seconds of CPU time, with a wall-clock watchdog of
          GRADE_WALL_FACTOR x GRADE_CPU_S as a second line; the grader runs that work in a child process under the
          limit and scores 0 a description that needs more (this module only holds the numbers).
Not gate rules (scored per item): whether a closed chain closes at a pose, and the sense of a last hinge.
"""
import math
import os
import re
import stat as _stat
import tempfile
import time
import xml.etree.ElementTree as ET

try:
    from . import contract as C
    from . import calib as CAL
    from . import whitelist as WL
except ImportError:  # loaded as plain modules beside each other (the format checker's folder)
    import contract as C
    import calib as CAL
    import whitelist as WL

GATE_VERSION = "hand-description-gate/1"
MAX_SUBMISSION_BYTES = 200 * 1024 * 1024
MAX_SUBMISSION_ENTRIES = 20000
COMPILE_CPU_S = 120.0
MAX_LOOP_CONNECTS = 8           # active connect equalities of one finger's closed chain (the cost of every loop solve)
GRADE_CPU_S = 300.0             # rule 11: CPU seconds of the grader's work after rules 1 to 10
GRADE_WALL_FACTOR = 6           # rule 11's second line: a wall-clock watchdog of 6 x GRADE_CPU_S (a grading that sleeps)
PATH_ATTRS = ("meshdir", "texturedir", "assetdir")
CONTRACT_BODIES = (C.ROOT_BODY,) + tuple(C.CONTRACT_JOINTS)


# ------------------------------------------------------------------------------------------------ rule 1: files
def submission_files(sub_dir, max_entries=MAX_SUBMISSION_ENTRIES):
    """(regular files {relpath: (apparent, allocated)}, irregular entries, number of entries) below sub_dir, never
    following a link; counting stops after max_entries + 1 entries."""
    regular, irregular, n = {}, [], 0
    for root, dirs, files in os.walk(sub_dir, followlinks=False):
        for name in list(dirs):
            n += 1
            p = os.path.join(root, name)
            if os.path.islink(p):
                irregular.append(os.path.relpath(p, sub_dir) + " (symbolic link)")
                dirs.remove(name)
        for fn in files:
            n += 1
            p = os.path.join(root, fn)
            try:
                st = os.lstat(p)
            except OSError:
                irregular.append(os.path.relpath(p, sub_dir) + " (unreadable)")
                continue
            if _stat.S_ISLNK(st.st_mode):
                irregular.append(os.path.relpath(p, sub_dir) + " (symbolic link)")
            elif not _stat.S_ISREG(st.st_mode):
                irregular.append(os.path.relpath(p, sub_dir) + " (not a regular file)")
            else:
                regular[os.path.relpath(p, sub_dir)] = (int(st.st_size), int(getattr(st, "st_blocks", 0)) * 512)
        if n > max_entries:
            break
    return regular, irregular, n


def rule_files(sub_dir):
    v = []
    for nm in ("robot.xml", "config.yaml"):
        p = os.path.join(sub_dir, nm)
        if not os.path.isfile(p) or os.path.islink(p):
            v.append("rule 1: %s missing (a regular file in the submission root)" % nm)
    if v:
        return v, {}
    regular, irregular, n = submission_files(sub_dir)
    apparent = sum(a for a, _ in regular.values())
    allocated = sum(b for _, b in regular.values())
    det = {"submission_entries": n, "submission_bytes": max(apparent, allocated)}
    if n > MAX_SUBMISSION_ENTRIES:
        v.append("rule 1: more than %d entries in the submission" % MAX_SUBMISSION_ENTRIES)
    if irregular:
        v.append("rule 1: the submission must hold regular files and directories only: %s" % irregular[:6])
    if max(apparent, allocated) > MAX_SUBMISSION_BYTES:
        v.append("rule 1: the submission holds %d bytes (at most %d)" % (max(apparent, allocated),
                                                                         MAX_SUBMISSION_BYTES))
    return v, det


def _bad_path(val):
    """Why a file / directory attribute may leave the submission (None when it cannot)."""
    val = str(val)
    if not val.strip():
        return None
    if os.path.isabs(val) or val.startswith(("~", "\\")) or re.match(r"^[A-Za-z]:", val):
        return "an absolute path"
    if any(p == ".." for p in re.split(r"[\\/]+", val)):
        return "a '..' component"
    return None


def rule_paths(xml_path):
    """Every file= and meshdir / texturedir / assetdir of robot.xml a relative path without '..'."""
    v = []
    try:
        root = ET.parse(xml_path).getroot()
    except (ET.ParseError, OSError) as exc:
        return ["rule 1: robot.xml does not parse: %s" % str(exc)[:160]]
    for el in root.iter():
        for k, val in el.attrib.items():
            if k == "file" or (el.tag == "compiler" and k in PATH_ATTRS):
                why = _bad_path(val)
                if why:
                    v.append("rule 1: <%s %s=%r> is %s (every file must resolve inside the submission)"
                             % (el.tag, k, val, why))
    return v


def _hull_faces(n_vertices):
    """The most triangles the convex hull of n points can have (2 V - 4): MuJoCo meshes a mesh given by vertices
    alone as their convex hull."""
    return max(0, 2 * int(n_vertices) - 4)


def _stl_triangles(path, size):
    """A binary STL's triangle count when the file's size is the one that count implies (84 + 50 n); else 0 (MuJoCo's
    decoder refuses the file and the compile reports it)."""
    if size < 84:
        return 0
    with open(path, "rb") as fh:
        fh.seek(80)
        n = int.from_bytes(fh.read(4), "little")
    return n if size == 84 + 50 * n else 0


def _msh_triangles(path, size):
    """MuJoCo's binary MSH: the face count of its header when the file holds what the header declares (its vertices
    only: the hull bound); else 0."""
    if size < 16:
        return 0
    with open(path, "rb") as fh:
        nv, nn, nt, nf = (int.from_bytes(fh.read(4), "little", signed=True) for _ in range(4))
    if min(nv, nn, nt, nf) < 0 or 16 + 12 * nv + 12 * nn + 8 * nt + 12 * nf > size:
        return 0
    return nf if nf > 0 else _hull_faces(nv)


def _obj_triangles(path):
    """An OBJ's triangles: each face of k corners is k - 2 triangles (MuJoCo's fan); an OBJ without faces counts its
    vertices' hull bound."""
    faces, verts = 0, 0
    with open(path, "rb") as fh:
        for raw in fh:
            s = raw.lstrip()
            if s[:2] in (b"f ", b"f\t"):
                faces += max(0, len(s.split()) - 3)
            elif s[:2] in (b"v ", b"v\t"):
                verts += 1
    return faces if faces else _hull_faces(verts)


def mesh_file_triangles(path):
    """The triangles MuJoCo can build from one mesh file, read from the file (never more than the count it states).
    The decoder is chosen by the file's extension; for any other extension the largest of the three readings."""
    size = os.path.getsize(path)
    ext = os.path.splitext(path)[1].lower()
    if ext == ".stl":
        return _stl_triangles(path, size)
    if ext == ".msh":
        return _msh_triangles(path, size)
    if ext == ".obj":
        return _obj_triangles(path)
    return max(_stl_triangles(path, size), _msh_triangles(path, size), _obj_triangles(path))


def _ints(text):
    try:
        return [int(math.ceil(abs(float(x)))) for x in str(text).replace(",", " ").split()]
    except (ValueError, OverflowError):
        return []


def builtin_triangles_bound(builtin, params):
    """An upper bound of the triangles MuJoCo builds for a built-in mesh, from its XML parameters (MuJoCo builds a
    built-in mesh while it parses the file, so the bound is taken before MuJoCo reads anything)."""
    p = _ints(params)
    a = p[0] if p else 0
    b = p[1] if len(p) > 1 else 0
    kind = str(builtin).strip().lower()
    if kind == "sphere":
        return 20 * 4 ** a if a <= 4 else 0                       # MuJoCo refuses a subdivision above 4
    if kind == "hemisphere":
        return 4 * (a + 1) * (a + 2) if a <= 10 else 0            # MuJoCo refuses a resolution above 10
    if kind in ("supersphere", "supertorus"):
        return 2 * a * a
    if kind == "plate":
        return 2 * a * b
    if kind == "wedge":
        return _hull_faces(a * b)                                 # vertices only: the hull
    if kind == "cone":
        return _hull_faces(2 * a)
    return 2 * (a + 1) * (b + 1)                                  # any other kind: a grid of a x b


def _xml_files(xml_path, main_dir, sub_real, depth=0, seen=()):
    """xml_path and every file it includes, as MuJoCo reads them (an <include> is resolved against the folder of the
    main file at every depth), each a regular file inside the submission; a cycle or a depth above 5 stops."""
    out = [xml_path]
    seen = set(seen) | {os.path.realpath(xml_path)}
    if depth > 5:
        return out
    try:
        root = ET.parse(xml_path).getroot()
    except (ET.ParseError, OSError):
        return out
    for el in root.iter("include"):
        p = os.path.realpath(os.path.join(main_dir, el.attrib.get("file", "")))
        if p not in seen and os.path.isfile(p) and os.path.commonpath([p, sub_real]) == sub_real:
            out += _xml_files(p, main_dir, sub_real, depth + 1, seen)
    return out


def count_mesh_triangles(xml_path, sub_dir):
    """(triangles, note): the mesh triangles of the submission counted per mesh asset BEFORE anything is compiled.
    First the built-in meshes are bounded from their XML parameters (note "built-in meshes" when that bound alone
    exceeds the cap: MuJoCo is then not called at all); then MuJoCo's parser (it builds no mesh from a file) names
    every mesh asset, and each counts its inline faces, its vertices' hull bound (vertices without faces) and the
    triangles of its file (meshdir / assetdir and strippath as MuJoCo applies them). A file is read once."""
    sub_real = os.path.realpath(sub_dir)
    bound = 0
    for f in _xml_files(os.path.abspath(xml_path), os.path.dirname(os.path.abspath(xml_path)), sub_real):
        try:
            root = ET.parse(f).getroot()
        except (ET.ParseError, OSError):
            continue
        for el in root.iter("mesh"):
            if el.attrib.get("builtin"):
                bound += builtin_triangles_bound(el.attrib.get("builtin"), el.attrib.get("params", ""))
    if bound > C.TRI_CAP:
        return bound, "built-in meshes"
    import mujoco
    try:
        spec = mujoco.MjSpec.from_file(str(xml_path))
    except Exception:  # noqa: BLE001 - a file MuJoCo cannot parse fails the compile (rule 3), which reports it
        return 0, None
    base = spec.modelfiledir or (os.path.dirname(os.path.abspath(xml_path)) + os.sep)
    meshdir = spec.meshdir or ""
    total, files = 0, {}
    for mesh in spec.meshes:
        nf = len(mesh.userface) // 3
        total += nf if nf else _hull_faces(len(mesh.uservert) // 3)
        if mesh.file:
            name = os.path.basename(mesh.file) if spec.strippath else mesh.file
            p = os.path.realpath(os.path.join(base, meshdir, name))
            if p not in files:
                inside = os.path.commonpath([p, sub_real]) == sub_real
                files[p] = mesh_file_triangles(p) if inside and os.path.isfile(p) else 0
            total += files[p]
    return total, None


def rule_mesh_count(xml_path, sub_dir):
    n, note = count_mesh_triangles(xml_path, sub_dir)
    if n > C.TRI_CAP:
        return ["rule 1: %d mesh triangles (at most %d, counted per mesh asset from the mesh files before anything is "
                "compiled%s)" % (n, C.TRI_CAP, "; " + note + " bounded from their parameters" if note else "")], n
    return [], n


def rule_triangles(m):
    faces = int(sum(int(x) for x in m.mesh_facenum)) if m.nmesh else 0
    if faces > C.TRI_CAP:
        return ["rule 1: %d mesh triangles in the compiled model (at most %d, counted per mesh asset)"
                % (faces, C.TRI_CAP)], faces
    return [], faces


# ------------------------------------------------------------------------------------------------ rule 2: whitelist
def rule_whitelist(xml_path, sub_dir):
    ok, viol, wstats = WL.check_mjcf(xml_path, submission_root=sub_dir, robot_root=C.ROOT_BODY)
    return ([] if ok else ["rule 2: %s" % s for s in viol[:12]]), wstats


# ------------------------------------------------------------------------------------------------ rule 3: loading
def rule_header(xml_path):
    try:
        root = ET.parse(xml_path).getroot()
    except (ET.ParseError, OSError) as exc:
        return ["rule 3: robot.xml does not parse: %s" % str(exc)[:160]]
    comps = root.findall("compiler")
    v = []
    if not any(c.attrib.get("angle") == "radian" for c in comps):
        v.append('rule 3: robot.xml must state <compiler angle="radian"> explicitly (every angle in radians)')
    for c in comps:
        if c.attrib.get("angle") not in (None, "radian"):
            v.append("rule 3: <compiler angle=%r> (radian required)" % c.attrib.get("angle"))
    return v


def compile_model(xml_path):
    """(model | None, warnings, error, cpu seconds): MuJoCo's warning callback and fd-level stderr capture."""
    import mujoco
    warnings, error = [], None
    saved = os.dup(2)
    tmp = tempfile.TemporaryFile(mode="w+b")
    os.dup2(tmp.fileno(), 2)
    model = None
    t0 = time.process_time()
    try:
        try:
            mujoco.set_mju_user_warning(lambda msg: warnings.append(str(msg)))
        except Exception:  # noqa: BLE001
            pass
        try:
            model = mujoco.MjModel.from_xml_path(str(xml_path))
        except Exception as exc:  # noqa: BLE001
            error = str(exc)[:600]
        finally:
            try:
                mujoco.set_mju_user_warning(None)
            except Exception:  # noqa: BLE001
                pass
    finally:
        cpu = time.process_time() - t0
        os.dup2(saved, 2)
        os.close(saved)
        tmp.seek(0)
        captured = tmp.read().decode("utf-8", "replace")
        tmp.close()
    if "WARNING" in captured.upper():
        warnings.append(captured.strip()[:400])
    return model, warnings, error, cpu


def rule_compile(xml_path):
    m, warnings, error, cpu = compile_model(xml_path)
    v = []
    if error:
        v.append("rule 3: robot.xml does not compile: %s" % error[:300])
    elif warnings:
        v.append("rule 3: robot.xml compiles with warnings: %s" % " | ".join(w[:160] for w in warnings[:3]))
    if cpu > COMPILE_CPU_S:
        v.append("rule 3: the compile took %.0f s of CPU time (at most %.0f s)" % (cpu, COMPILE_CPU_S))
    return v, (m if not error else None), len(warnings)


# ------------------------------------------------------------------------------------------------ rules 4 - 8
def _names(m, objtype, n):
    import mujoco
    return [mujoco.mj_id2name(m, objtype, i) for i in range(n)]


def _ancestors(m, b):
    out = []
    while b > 0:
        b = int(m.body_parentid[b])
        out.append(b)
    return out


def nearest_contract_ancestor(m, b, body_names=None):
    """Name of the nearest proper ancestor of body b among palm and the 16 contract bodies ("world" if none)."""
    bn = body_names or _names(m, __import__("mujoco").mjtObj.mjOBJ_BODY, m.nbody)
    for a in _ancestors(m, b):
        if a == 0:
            return "world"
        if bn[a] in CONTRACT_BODIES:
            return bn[a]
    return "world"


def rule_structure(m):
    import mujoco
    v = []
    bn = _names(m, mujoco.mjtObj.mjOBJ_BODY, m.nbody)
    jn = _names(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt)
    bid = {n: i for i, n in enumerate(bn) if n}
    palm = bid.get(C.ROOT_BODY, -1)
    if palm < 0:
        return ["rule 4: body %s missing" % C.ROOT_BODY]
    if int(m.body_parentid[palm]) != 0 or int(m.body_jntnum[palm]) != 0:
        v.append("rule 4: body %s must be a jointless child of the world body" % C.ROOT_BODY)
    for i in range(m.njnt):
        if int(m.jnt_type[i]) != int(mujoco.mjtJoint.mjJNT_HINGE):
            v.append("rule 4: joint %s is not a hinge (only hinges are allowed)" % (jn[i] or "#%d" % i))
    for d in C.DIGITS:
        parent = C.ROOT_BODY
        for j in C.CHAINS[d]:
            ji = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
            if ji < 0:
                v.append("rule 4: joint %s missing" % j)
            else:
                if not m.jnt_limited[ji]:
                    v.append("rule 4: joint %s must be limited (a range)" % j)
                if bn[int(m.jnt_bodyid[ji])] != j:
                    v.append("rule 4: joint %s must sit in the body %s (it is in %s)" % (j, j, bn[int(m.jnt_bodyid[ji])]))
            b = bid.get(j, -1)
            if b < 0:
                v.append("rule 4: body %s missing" % j)
            else:
                anc = nearest_contract_ancestor(m, b, bn)
                if anc != parent:
                    v.append("rule 4: body %s must come after %s in its chain (its nearest contract ancestor is %s)"
                             % (j, parent, anc))
            parent = j
    contract_j = set(C.CONTRACT_JOINTS)
    loop_names = {n: f for f in C.FINGERS for n in C.LOOP_BODIES[f]}
    for i in range(m.njnt):
        if jn[i] in contract_j:
            continue
        b = int(m.jnt_bodyid[i])
        f = loop_names.get(bn[b])
        if f is None:
            v.append("rule 4: the extra hinge %s sits in body %s (extra hinges only in the bodies <finger>_loop1 .. "
                     "<finger>_loop4)" % (jn[i] or "#%d" % i, bn[b]))
            continue
        first = bid.get(C.FIRST_BODY[f], -1)
        if first < 0 or first not in _ancestors(m, b):
            v.append("rule 4: body %s must lie below body %s" % (bn[b], C.FIRST_BODY[f]))
    return v


def _finger_of_body(m, b, first_ids):
    anc = [b] + _ancestors(m, b)
    for f in C.FINGERS:
        if first_ids.get(f, -1) in anc:
            return f
    return None


def rule_coupling(m):
    import mujoco
    v = []
    bn = _names(m, mujoco.mjtObj.mjOBJ_BODY, m.nbody)
    jn = _names(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt)
    first = {f: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, C.FIRST_BODY[f]) for f in C.FINGERS}
    form = {f: [] for f in C.FINGERS}
    for e in range(m.neq):
        t = int(m.eq_type[e])
        if t == int(mujoco.mjtEq.mjEQ_JOINT):
            if not m.eq_active0[e]:
                continue
            j1, j2 = int(m.eq_obj1id[e]), int(m.eq_obj2id[e])
            n1 = jn[j1] if j1 >= 0 else None
            n2 = jn[j2] if j2 >= 0 else None
            f = next((ff for ff in C.FINGERS if n1 == C.DIP_OF_FINGER[ff]), None)
            if f is None or n2 != C.PIP_OF_FINGER[f]:
                v.append("rule 5: a joint equality must couple <f>_dip (joint1) to <f>_pip (joint2); found %s / %s"
                         % (n1, n2))
                continue
            form[f].append("joint")
        elif t == int(mujoco.mjtEq.mjEQ_CONNECT):
            if not m.eq_active0[e]:
                continue
            ot = int(m.eq_objtype[e])
            if ot == int(mujoco.mjtObj.mjOBJ_SITE):
                ba, bb = int(m.site_bodyid[int(m.eq_obj1id[e])]), int(m.site_bodyid[int(m.eq_obj2id[e])])
            elif ot == int(mujoco.mjtObj.mjOBJ_BODY):
                ba, bb = int(m.eq_obj1id[e]), int(m.eq_obj2id[e])
            else:
                v.append("rule 5: a connect equality must join two bodies or two sites")
                continue
            fa, fb = _finger_of_body(m, ba, first), _finger_of_body(m, bb, first)
            if fa is None or fa != fb:
                v.append("rule 5: a connect equality must join two bodies of one finger (it joins %s and %s)"
                         % (bn[ba], bn[bb]))
                continue
            form[fa].append("loop")
        else:
            v.append("rule 5: only joint and connect equalities are allowed")
    driven = set(C.MOTOR_ORDER)
    for f in C.FINGERS:
        kinds = set(form[f])
        if not kinds:
            v.append("rule 5: %s is not coupled (a joint equality to %s or a closed chain of connect equalities)"
                     % (C.DIP_OF_FINGER[f], C.PIP_OF_FINGER[f]))
        elif kinds == {"joint", "loop"}:
            v.append("rule 5: finger %s has both a joint equality and connect equalities" % f)
        elif kinds == {"loop"} and len(form[f]) > MAX_LOOP_CONNECTS:
            v.append("rule 5: finger %s has %d active connect equalities (at most %d in one finger)"
                     % (f, len(form[f]), MAX_LOOP_CONNECTS))
        elif kinds == {"joint"}:
            if len(form[f]) != 1:
                v.append("rule 5: finger %s has more than one joint equality" % f)
            if first[f] >= 0:
                extra = [jn[i] for i in range(m.njnt) if jn[i] not in driven and jn[i] != C.DIP_OF_FINGER[f]
                         and _finger_of_body(m, int(m.jnt_bodyid[i]), first) == f]
                if extra:
                    v.append("rule 5: finger %s is coupled by a joint equality but has the extra hinge(s) %s"
                             % (f, extra[:4]))
    return v


def rule_pads(m):
    import mujoco
    v = []
    bn = _names(m, mujoco.mjtObj.mjOBJ_BODY, m.nbody)
    for d, s in C.PAD_SITES.items():
        sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, s)
        if sid < 0:
            v.append("rule 6: site %s missing" % s)
            continue
        b = int(m.site_bodyid[sid])
        ok = False
        while b > 0:
            if bn[b] == C.LAST_BODY[d]:
                ok = True
                break
            if int(m.body_jntnum[b]) != 0:
                break
            b = int(m.body_parentid[b])
        if not ok:
            v.append("rule 6: site %s must be rigid on body %s or on a jointless body below it" % (s, C.LAST_BODY[d]))
    return v


def rule_actuators(m):
    import mujoco
    v = []
    an = _names(m, mujoco.mjtObj.mjOBJ_ACTUATOR, m.nu)
    if sorted(a or "" for a in an) != sorted(C.ACTUATORS):
        v.append("rule 7: the actuators must be exactly the 13 act_<driven hinge> (found %d: %s)" % (m.nu, an[:16]))
    for a, j in C.ACTUATORS.items():
        ai = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        if ai < 0:
            continue
        if int(m.actuator_trntype[ai]) != int(mujoco.mjtTrn.mjTRN_JOINT) or \
                mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, int(m.actuator_trnid[ai, 0])) != j:
            v.append("rule 7: actuator %s must drive the hinge %s" % (a, j))
        gear = [float(x) for x in m.actuator_gear[ai]]
        if abs(gear[0] - 1.0) > 1e-12 or any(abs(x) > 1e-12 for x in gear[1:]):
            v.append("rule 7: actuator %s must have gear 1" % a)
        kp = float(m.actuator_gainprm[ai, 0])
        if int(m.actuator_gaintype[ai]) != int(mujoco.mjtGain.mjGAIN_FIXED) or \
                int(m.actuator_biastype[ai]) != int(mujoco.mjtBias.mjBIAS_AFFINE) or \
                not math.isfinite(kp) or kp <= 0 or abs(float(m.actuator_biasprm[ai, 1]) + kp) > 1e-9 * max(1.0, abs(kp)):
            v.append("rule 7: actuator %s must be a position servo (<position>, or <general> with gaintype fixed, "
                     "biastype affine, biasprm[1] = -gainprm[0] and gainprm[0] > 0)" % a)
        lo, hi = float(m.actuator_forcerange[ai, 0]), float(m.actuator_forcerange[ai, 1])
        if not m.actuator_forcelimited[ai]:
            v.append("rule 7: actuator %s must be force-limited (forcerange)" % a)
        elif not (math.isfinite(hi) and hi > 0.0 and lo == -hi):
            v.append("rule 7: actuator %s: forcerange must be [-f, +f] with f finite and > 0" % a)
    return v


def rule_masses(m):
    v = []
    masses = [float(x) for x in m.body_mass]
    bad = [i for i, x in enumerate(masses) if not (math.isfinite(x) and x >= 0.0)]
    if bad:
        v.append("rule 8: %d body mass(es) are not finite and >= 0" % len(bad))
    elif sum(masses) <= 0.0:
        v.append("rule 8: the total mass must be > 0")
    return v


# ------------------------------------------------------------------------------------------------ the gate
def gate_model_loads(sub_dir):
    """(ok, details, models). Nothing is compiled once a static rule failed."""
    sub_dir = os.path.abspath(str(sub_dir))
    v = []
    det = {"violations": v, "gate_version": GATE_VERSION}
    models = {"robot": None, "calib": None}
    robot = os.path.join(sub_dir, "robot.xml")
    cfg = os.path.join(sub_dir, "config.yaml")
    fv, fdet = rule_files(sub_dir)
    v += fv
    det.update(fdet)
    if v:
        return False, det, models
    wv, wstats = rule_whitelist(robot, sub_dir)
    det["whitelist_stats"] = wstats
    v += wv
    if v:
        return False, det, models
    v += rule_paths(robot)
    if not v:                                  # every file is inside the submission: count before anything compiles
        mv, counted = rule_mesh_count(robot, sub_dir)
        det["mesh_triangles_counted"] = counted
        v += mv
    v += rule_header(robot)
    try:
        calib = CAL.load_calibration(cfg)
    except CAL.CalibError as exc:
        v.append(str(exc))
        calib = None
    if v:
        return False, det, models
    cv, m, n_warn = rule_compile(robot)
    det["compile_warnings"] = n_warn
    v += cv
    if m is None or v:
        return False, det, models
    tv, faces = rule_triangles(m)
    det["mesh_triangles"] = faces
    v += tv
    v += rule_structure(m)
    if not v:
        v += rule_coupling(m)
    v += rule_pads(m)
    v += rule_actuators(m)
    v += rule_masses(m)
    if v:
        return False, det, models
    models["robot"] = m
    models["calib"] = calib
    return True, det, models
