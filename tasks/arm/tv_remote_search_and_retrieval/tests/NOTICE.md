# NOTICE - sources of this task's code, room and episodes (verify / maintainers only; never shown to the agent)

- **The robot** is the Franka Panda model of robosuite 1.5.2 (MIT), installed unmodified in the task image.
- **The room's meshes and textures** are pinned in `environment/assets.sha256` (dataset
  `tasks/arm/tv_remote_search_and_retrieval/payload/room/`); their sources, authors and licences are in that folder's
  `SOURCES.md` (Poly Haven, CC0; Objaverse 1.0, CC BY 4.0; a few made for this task). The scripts that made them from
  the downloaded sources are `review/build_*.py`.
- **The simulator** `remote_env.py` is byte-identical in tests/ and environment/setup/payload/ (the grader and the
  agent run the same room). It knows no place a remote may be in: `make_scene` puts the remote, the second remote and
  the other small things where its caller says.
- **The hidden rule** `episodes.py` (where the TV remote, the second remote, the phone, the glasses case and the book
  are in an episode of a seed and a place) exists only here.
- **The hidden episodes** (`hidden/episodes.json`) were chosen by `review/select_episodes.py` under the rule in its
  docstring; `hidden/selection.json` is its record (the key, the frozen files' sha256, every candidate run).
- **The reference** (solution/) is privileged: `verify.py` adds the grader's truth to the observations of a submission
  whose `.oracle_token` hashes to `hidden/oracle_token.sha256`, and solution/solver/oracle.py takes the layout and the
  remote's place from it. No controller without that truth is shipped as a solution.
- **`review/author_controller.py`** is the controller this task was contributed with (it fits the furniture to the
  first depth image and goes through six places in a fixed order). It is review material and one of the three
  pre-registered controls (`review/controls.py`: holds still; looks at the open surfaces only; the contributed
  controller); nothing ships it to the agent and no submission may use it.
- **The grader**: `remote_grade.py` is the episode loop; `nonet_exec.py` starts the controller (seccomp: local sockets
  only; nice 19; a process cap); `robotics_grader/` is the repository's kit, byte-identical to
  `shared/robotics_grader/`.
- **The numerics**: `numerics_check.py` and `numerics.json` are the fingerprint the grader checks before grading (the
  image's numerics pin, environment/Dockerfile); `numerics.json` was written by `numerics_check.py --write` in the
  task image.
