# Reproducing the scout ablation

Use Python 3.12, the packages in `environment/requirements.txt`, and Mesa's libOSMesa. Run from the task directory with `CUDA_VISIBLE_DEVICES="" LP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1`. The simulator uses CPU rendering. These are developer diagnostics, not secret grading seeds.

```bash
python tests/diagnostics/test_task.py -v
python tests/diagnostics/pilot.py --seeds $(seq 2101 2112) --ticks 150 --preview household.png > paired150.jsonl
python tests/diagnostics/pilot.py --seeds $(seq 2101 2112) --ticks 95 > paired95.jsonl
python tests/diagnostics/pilot.py --seeds $(seq 3200 3223) --modes adaptive > reference24.jsonl
```

The ground planner is the same in all three modes. **Solo** disables the scout. **Fixed** flies an obstacle-aware, row-ordered survey using only the aircraft's own observation history. **Adaptive** uses the shared map, ground position, currently needed rooms, and missing carton/mat information to select survey frontiers. Both scouts get 36 flight ticks. Neither controller has access to the grader state; its only inputs are the wire observation and time index. Pair runs by `seed` and `scene`; compare strict completion before comparing times on paired successes.

This is a finite family of four-room apartments, with a calibrated visual palette and ideal motion primitives; it is not a photorealistic household dataset, a test of low-level flight, or evidence about a frontier language model.
