# Building this task's events (maintainers and the author; nothing runs this)

The events, their clips and their answers come from the IEDD release (Hugging Face `Egikk/IEDD`, the files of Zenodo
record 10.5281/zenodo.18742437, CC BY 4.0): the IEDD-VQA test split (`IEDD-VQA_test.json`,
`IEDD-VQA_test_videos.zip`) and the Waymo annotations its samples come from (`IEDD-waymo.zip`), about 220 MB.

```bash
pip3 install -U huggingface_hub opencv-python-headless       # hf CLI; OpenCV counts the frames of each clip
bash tasks/ground/bev_interaction_quantification/tests/review/build_from_hf.sh ~/iedd
```

`build_from_hf.sh` downloads and unpacks the three files into the work directory, then runs `build_task_data.py`,
which prints how many rows it kept and why it dropped the others, the test and labelled events per type and scene,
and what a correct type with a typical peak would score. It writes:

- `tests/ref/clips.json`, `tests/ref/answers.csv`, `tests/ref/source.json` and `solution/ref/answers.csv` (commit
  them);
- `environment/setup/payload/clips.json` (commit it);
- `environment/setup/payload/test/` and `labelled/` (git-ignored local copies, for a local `ale validate` before the
  upload);
- `<work dir>/upload/`, the files for the asset dataset.

Then, as `docs/task-assets.md` describes:

```bash
hf upload grounded-intelligence-initiative/ale-robotics-assets ~/iedd/upload \
    tasks/ground/bev_interaction_quantification/payload --repo-type dataset --create-pr
python3 scripts/task_assets.py list tasks/ground/bev_interaction_quantification ~/iedd/upload --revision <commit>
```

Selection rules (also in `build_task_data.py`'s docstring and in `../NOTICE.md`): a pairwise row with an intensity
series, no other row of its scene for the same pair and interval, a clip, and IEDD-VQA answers that agree with the row
(the same two vehicles, the same type, a peak within 0.006). Test and labelled events come from disjoint sets of
scenes, at most two per scene, round-robin over the types and over equal-width bins of the peak, seed 20261125.

Checks done on the GitHub sample of IEDD (`csv_dir/`, 13 Waymo scenes, 374 pair events) with stand-in clips named as
IEDD names them, and IEDD-VQA answers written in IEDD-2vqa.py's wording with every 25th peak corrupted: every clip is
matched to its own row and a decoy whose scene name shares a prefix (`scene_1` against `scene_112`) is never matched;
the 14 corrupted samples are rejected; the oracle scores 1.0; a constant guess scores 0.04; the right type with a
constant peak scores 0.17 to 0.25; peaks off by 0.049 pass and by 0.051 fail; a missing, empty, mis-headed,
duplicated, non-ASCII, symlinked or over-1-MiB file scores 0.
