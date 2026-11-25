"""Build the data files of bev_interaction_quantification from IEDD's annotations and BEV clips.

Inputs, all from the IEDD release (Hugging Face dataset Egikk/IEDD, the same files as Zenodo
doi 10.5281/zenodo.18742437, CC BY 4.0):

  IEDD-VQA_test.json         the IEDD-VQA test split: one sample per clip, with IEDD-VQA's own answers
  IEDD-VQA_test_videos.zip   its clips, videos/<scene with / as _>_row<rrrr>_t<start>-<end>.mp4
  IEDD-waymo.zip             the Waymo annotations: <scene folder>/interaction.csv (the split is all Waymo)

  hf download Egikk/IEDD IEDD-VQA_test.json IEDD-VQA_test_videos.zip IEDD-waymo.zip \
      --repo-type dataset --local-dir iedd
  unzip -q iedd/IEDD-VQA_test_videos.zip -d iedd/clips && unzip -q iedd/IEDD-waymo.zip -d iedd/csv
  python3 build_task_data.py --csv-dir iedd/csv --videos iedd/clips --vqa-json iedd/IEDD-VQA_test.json \
      --task-dir <repo>/tasks/ground/bev_interaction_quantification --out upload

Options: [--sources waymo] [--n-test 24] [--n-labelled 32] [--max-per-scene 2] [--seed 20261125]. Without
--vqa-json any IEDD clips are used (IEDD-traj2VisAct.py's "<csv tag>__<scene>_row<rrrr>_t<start>-<end>.mp4" names
work too): a row's clip is found by its row number, its interval and its scene's name.

An event is one pairwise row of an interaction.csv (Car-follow, Merging, Crossing or Head-on; not Multi) with an
intensity series, whose scene has no other row for the same two vehicles over the same interval, and whose clip
exists. With --vqa-json the row must be an IEDD-VQA sample whose answers agree with the row: the same two vehicles,
the same type, and a peak intensity within 0.006 of the row's (IEDD-VQA prints two decimals).

The agent is given the event's two vehicle ids: a clip draws one vehicle in red and the others in black, and the same
red vehicle over the same interval often has several partners (328 of the 374 pair events of the GitHub sample), so
the pair is not recoverable from the clip alone. The answers are the type and the largest Q_i(t) of either vehicle
over the interval (IEDD-VQA's "Q peaks ... with Q =" answer, at four decimals). IEDD's efficiency E_i is not asked:
IEDD computes it over a vehicle's whole track in the scene, and the clip shows only the event's interval.

Test and labelled events come from disjoint sets of scenes (IEDD's Q_i and E_i belong to a vehicle, not to a pair, so
an answered event of the same scene would give away part of a test answer), at most --max-per-scene events from one
scene, round-robin over the types and over equal-width bins of the peak.

Writes
  <out>/test/Txx.mp4, <out>/labelled/Lxx/video.mp4, <out>/labelled/answers.csv     upload these to the HF dataset
  <task>/environment/setup/payload/clips.json                                       git: the agent-visible event list
  <task>/environment/setup/payload/test|labelled/                                   local copies for authoring (gitignored)
  <task>/tests/ref/clips.json, answers.csv, source.json; <task>/solution/ref/answers.csv   git: hidden answers
"""
import argparse
import collections
import csv
import glob
import json
import os
import random
import re
import shutil
import sys

TYPES = ("Car-follow", "Merging", "Crossing", "Head-on")
HEADER = ["clip", "interaction_type", "peak_intensity"]
INTERVAL = re.compile(r"\s*([0-9]+(?:\.[0-9]+)?)\s*-\s*([0-9]+(?:\.[0-9]+)?)\s*")
VQA_TYPES = {"car follow": "Car-follow", "car-follow": "Car-follow", "merging": "Merging", "crossing": "Crossing",
             "head on": "Head-on", "head-on": "Head-on"}
VQA_PEAK_TOL = 0.006


def read_rows(path):
    for enc in ("utf-8-sig", "gbk", "latin-1"):
        try:
            with open(path, newline="", encoding=enc) as f:
                return list(csv.DictReader(f))
        except UnicodeDecodeError:
            continue
    raise ValueError("cannot decode " + path)


def sanitize(s):
    """IEDD-traj2VisAct.py's sanitize_tag without its length cap."""
    s = str(s).strip().replace("/", "__")
    s = re.sub(r"[^0-9a-zA-Z_\-\.]+", "_", s)
    return re.sub(r"_+", "_", s).strip("_")


def vqa_id(scene, row_idx, t0, t1):
    """IEDD-2vqa.py's build_actions_base_id: the sample id and clip name of a row."""
    token = scene.strip().replace("\\", "/").strip("/").replace("/", "_")
    return "%s_row%04d_t%.1f-%.1f" % (token, row_idx, t0, t1)


def read_vqa(path):
    """IEDD-VQA json -> {sample id: {"vehicles": set or None, "type": label or None, "peak": float or None}}."""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    out = {}
    for s in data if isinstance(data, list) else data.get("data", []):
        text = "\n".join(str(t.get("value", "")) for t in s.get("conversations", []))
        pair = re.search(r"ego vehicle \(([^)]+)\) is interacting with vehicle ([^\s,]+?)[.,]?(?:\s|$)", text)
        kind = re.search(r"classified as\W*(car[ -]follow|merging|crossing|head[ -]on)", text, re.I)
        peak = re.search(r"Q peaks for vehicle \S+ at about t = [0-9.]+ s with Q\s*(?:≈|~|=)\s*([0-9.]+)", text)
        out[str(s.get("id", ""))] = {
            "vehicles": {pair.group(1).strip(), pair.group(2).strip()} if pair else None,
            "type": VQA_TYPES.get(kind.group(1).lower().replace("-", " ")) if kind else None,
            "peak": float(peak.group(1).rstrip(".")) if peak else None,
        }
    return out


def pair_ids(r):
    return [v.strip() for v in (r.get("Vehicle Pair") or "").split(";") if v.strip()]


def peak_of(text, ids):
    """Largest Q of the vehicles `ids` in a Q_i_Over_Time field ('623:5.000:0.3820,...'); None if there is none."""
    best = None
    for tok in (text or "").split(","):
        tok = tok.strip()
        if tok:
            vid, _t, q = tok.rsplit(":", 2)
            if vid.strip() in ids:
                best = float(q) if best is None else max(best, float(q))
    return best


def frames_of(path):
    os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
    os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "-8")
    try:
        import cv2
    except ImportError:
        return None
    cap = cv2.VideoCapture(path)
    n = 0
    while cap.read()[0]:
        n += 1
    cap.release()
    return n or None


def find_video(index, scene, row_idx, t0, t1):
    suffix = "_row%04d_t%.1f-%.1f.mp4" % (row_idx, t0, t1)
    name = sanitize(scene.rstrip("/").split("/")[-1])
    bounded = re.compile(r"(?:^|[_.])%s(?:[_.]|$)" % re.escape(name))
    match = [p for p in index.get(suffix, []) if bounded.search(os.path.basename(p)[: -len(suffix)])]
    if len(match) > 1:  # the same scene name in two sources: keep the clip whose name holds the whole scene path
        match = [p for p in match if sanitize(scene) in os.path.basename(p)]
    return match


def disagreement(ev, ref):
    if ref["vehicles"] is not None and ref["vehicles"] != set(ev["vehicles"]):
        return "IEDD-VQA names other vehicles"
    if ref["type"] is not None and ref["type"] != ev["interaction_type"]:
        return "IEDD-VQA gives another type"
    if ref["peak"] is None:
        return "IEDD-VQA gives no peak"
    if abs(ref["peak"] - ev["peak_intensity"]) > VQA_PEAK_TOL:
        return "IEDD-VQA gives another peak"
    return None


def candidates(csv_dir, videos, sources, vqa=None):
    index = collections.defaultdict(list)
    for p in glob.glob(os.path.join(videos, "**", "*.mp4"), recursive=True):
        name = os.path.basename(p)
        cut = name.rfind("_row")
        if cut >= 0:
            index[name[cut:]].append(p)
    events, stats = [], collections.Counter()
    for path in sorted(glob.glob(os.path.join(csv_dir, "**", "interaction.csv"), recursive=True)):
        rows = read_rows(path)
        seen = collections.Counter((r.get("Scene"), frozenset(pair_ids(r)), (r.get("Time Interval") or "").strip())
                                   for r in rows)
        for row_idx, r in enumerate(rows):
            kind = (r.get("Interaction Type") or "").strip()
            scene = (r.get("Scene") or "").strip()
            source = scene.split("/")[0].split("_")[0].lower()
            ids = pair_ids(r)
            m = INTERVAL.fullmatch(r.get("Time Interval") or "")
            if vqa is not None and (not m or vqa_id(scene, row_idx, float(m.group(1)), float(m.group(2))) not in vqa):
                stats["not an IEDD-VQA sample"] += 1
            elif kind not in TYPES:
                stats["not a pairwise type"] += 1
            elif sources and source not in sources:
                stats["source not allowed"] += 1
            elif len(set(ids)) != 2 or not m:
                stats["no pair or interval"] += 1
            elif seen[(r.get("Scene"), frozenset(ids), (r.get("Time Interval") or "").strip())] > 1:
                stats["another row for the same pair and interval"] += 1
            else:
                try:
                    peak = peak_of(r.get("Q_i_Over_Time"), ids)
                except ValueError:
                    peak = None
                t0, t1 = float(m.group(1)), float(m.group(2))
                match = find_video(index, scene, row_idx, t0, t1) if peak is not None else []
                ev = {"scene": scene, "row": row_idx, "source": source, "video": match[0] if match else None,
                      "start_s": t0, "end_s": t1, "vehicles": ids, "interaction_type": kind, "peak_intensity": peak,
                      "vqa_id": vqa_id(scene, row_idx, t0, t1)}
                why = None
                if peak is None:
                    why = "no intensity series"
                elif len(match) != 1:
                    why = "no unique clip" if not match else "ambiguous clip"
                elif vqa is not None:
                    why = disagreement(ev, vqa[ev["vqa_id"]])
                if why:
                    stats[why] += 1
                else:
                    events.append(ev)
                    stats["usable"] += 1
    if vqa is not None:
        stats["IEDD-VQA samples without an interaction.csv row"] = len(set(vqa) - {
            vqa_id((r.get("Scene") or ""), i, float(INTERVAL.fullmatch(r["Time Interval"]).group(1)),
                   float(INTERVAL.fullmatch(r["Time Interval"]).group(2)))
            for p in glob.glob(os.path.join(csv_dir, "**", "interaction.csv"), recursive=True)
            for i, r in enumerate(read_rows(p)) if INTERVAL.fullmatch(r.get("Time Interval") or "")})
    return events, stats


def spread(rng, evs, k):
    """evs in an order whose every run of k events takes one from each of k equally wide bins of the peak (between the
    5th and 95th percentile of evs; empty bins are skipped), so that the chosen events span the range of intensities
    and a typical value guessed for every event is rarely right."""
    evs = sorted(evs, key=lambda e: (e["peak_intensity"], e["scene"], e["row"]))
    lo = evs[len(evs) // 20]["peak_intensity"]
    hi = evs[len(evs) - 1 - len(evs) // 20]["peak_intensity"]
    bins = [[] for _ in range(k)]
    for e in evs:
        x = (e["peak_intensity"] - lo) / (hi - lo) if hi > lo else 0.0
        bins[min(k - 1, max(0, int(x * k)))].append(e)
    bins = [b for b in bins if b]
    for b in bins:
        rng.shuffle(b)
    return [b[j] for j in range(max(map(len, bins))) for b in bins if j < len(b)]


def pick(rng, pool, n, max_per_scene, banned=()):
    """Up to n events, round-robin over the types and over the peak bins within each type, at most max_per_scene from
    one scene, none from `banned` scenes."""
    by = collections.defaultdict(list)
    for ev in pool:
        if ev["scene"] not in banned:
            by[ev["interaction_type"]].append(ev)
    kinds = [k for k in TYPES if by.get(k)]
    for k in kinds:
        by[k] = spread(rng, by[k], max(1, -(-n // len(kinds))))[::-1]
    out, per_scene, progress = [], collections.Counter(), True
    while len(out) < n and progress:
        progress = False
        for k in kinds:
            while by.get(k) and len(out) < n:
                ev = by[k].pop()
                if per_scene[ev["scene"]] < max_per_scene:
                    out.append(ev)
                    per_scene[ev["scene"]] += 1
                    progress = True
                    break
    rng.shuffle(out)
    return out


def write_answers(path, events):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(HEADER)
        for clip, ev in events:
            w.writerow([clip, ev["interaction_type"], "%.4f" % ev["peak_intensity"]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv-dir", required=True)
    ap.add_argument("--videos", required=True)
    ap.add_argument("--task-dir", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--vqa-json", default="")
    ap.add_argument("--sources", default="")
    ap.add_argument("--n-test", type=int, default=24)
    ap.add_argument("--n-labelled", type=int, default=32)
    ap.add_argument("--max-per-scene", type=int, default=2)
    ap.add_argument("--seed", type=int, default=20261125)
    a = ap.parse_args()
    sources = {s.strip().lower() for s in a.sources.split(",") if s.strip()}
    vqa = read_vqa(a.vqa_json) if a.vqa_json else None
    if vqa is not None:
        print("IEDD-VQA samples: %d (%d with a peak)" % (len(vqa), sum(v["peak"] is not None for v in vqa.values())))

    events, stats = candidates(a.csv_dir, a.videos, sources, vqa)
    print("rows:", dict(stats))
    rng = random.Random(a.seed)
    test = pick(rng, events, a.n_test, a.max_per_scene)
    if len(test) < a.n_test:
        sys.exit("only %d usable test events for --n-test %d: add scenes or raise --max-per-scene" % (len(test), a.n_test))
    lab = pick(rng, events, a.n_labelled, a.max_per_scene, banned={ev["scene"] for ev in test})
    test_ids = [("T%02d" % (i + 1), ev) for i, ev in enumerate(test)]
    lab_ids = [("L%02d" % (i + 1), ev) for i, ev in enumerate(lab)]

    task = a.task_dir
    local = os.path.join(task, "environment", "setup", "payload")
    for root in (a.out, local):
        for clip, ev in test_ids:
            dst = os.path.join(root, "test", clip + ".mp4")
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(ev["video"], dst)
        for clip, ev in lab_ids:
            dst = os.path.join(root, "labelled", clip, "video.mp4")
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(ev["video"], dst)
        write_answers(os.path.join(root, "labelled", "answers.csv"), lab_ids)

    def entry(clip, ev):
        return {"id": clip, "frames": frames_of(ev["video"]), "start_s": ev["start_s"], "end_s": ev["end_s"],
                "vehicles": ev["vehicles"]}

    clips = {"test": [entry(c, ev) for c, ev in test_ids], "labelled": [entry(c, ev) for c, ev in lab_ids]}
    for path in (os.path.join(local, "clips.json"), os.path.join(task, "tests", "ref", "clips.json")):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(clips, f, indent=1)
            f.write("\n")
    write_answers(os.path.join(task, "tests", "ref", "answers.csv"), test_ids)
    write_answers(os.path.join(task, "solution", "ref", "answers.csv"), test_ids)
    with open(os.path.join(task, "tests", "ref", "source.json"), "w") as f:
        json.dump({c: {k: ev[k] for k in ("scene", "row", "source", "start_s", "end_s", "vehicles", "vqa_id")}
                   | {"video": os.path.basename(ev["video"])} for c, ev in test_ids + lab_ids}, f, indent=1)
        f.write("\n")
    for name, group in (("test", test_ids), ("labelled", lab_ids)):
        kinds = collections.Counter(ev["interaction_type"] for _, ev in group)
        print("%-9s %2d events from %2d scenes  types %s  sources %s" % (
            name, len(group), len({ev["scene"] for _, ev in group}), dict(kinds),
            dict(collections.Counter(ev["source"] for _, ev in group))))
    # what a correct type with a typical peak earns: the median peak of the event's type over all usable events
    med = {k: sorted(e["peak_intensity"] for e in events if e["interaction_type"] == k) for k in TYPES}
    med = {k: v[len(v) // 2] for k, v in med.items() if v}
    print("right type, median peak of the type: right on %s of the test events" % ", ".join(
        "%.0f%% at tol %.2f" % (100.0 * sum(abs(ev["peak_intensity"] - med[ev["interaction_type"]]) <= tol
                                            for ev in test) / len(test), tol) for tol in (0.05, 0.03, 0.02)))
    if len(lab) < a.n_labelled:
        print("warning: only %d labelled events in scenes without test events (asked for %d)" % (len(lab), a.n_labelled))
    unknown = sum(e["frames"] is None for e in clips["test"] + clips["labelled"])
    if unknown:
        print("warning: %d clips could not be decoded (frames: null); install opencv-python-headless" % unknown)
    print("next: hf upload grounded-intelligence-initiative/ale-robotics-assets %s "
          "tasks/ground/bev_interaction_quantification/payload --repo-type dataset --create-pr" % a.out)


if __name__ == "__main__":
    main()
