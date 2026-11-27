#!/bin/bash
# needs: PY (python with mujoco), BT_PANDA_DIR, MUJOCO_GL=osmesa
cd $(dirname "$0")
mkdir -p logs/v5
for s in 1 2 3 4; do
  timeout 7200 "$PY" oracle.py $s 60 > logs/v5/oracle_s$s.json 2> logs/v5/oracle_s$s.err &
  BASE_TRUE_POSE=1 F_ABORT=0.4 timeout 7200 "$PY" baseline.py $s probe 60 > logs/v5/tp04_s$s.json 2> logs/v5/tp04_s$s.err &
  BASE_TRUE_POSE=1 F_ABORT=1.0 timeout 7200 "$PY" baseline.py $s probe 60 > logs/v5/tp10_s$s.json 2> logs/v5/tp10_s$s.err &
done
( for s in 1 2 3 4; do for v in "blind 1.0" "probe 1.0" "probe 0.4"; do set -- $v; F_ABORT=$2 timeout 3600 "$PY" baseline.py $s $1 60 > logs/v5/nom_${1}_$2_s$s.json 2> logs/v5/nom_${1}_$2_s$s.err; done; done ) &
wait
echo ALLDONE
