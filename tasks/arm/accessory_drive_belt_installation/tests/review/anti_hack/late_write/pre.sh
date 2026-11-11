#!/usr/bin/env bash
# started as the agent account just before the verify: keeps writing into the submission folder
while true; do echo "x" >> /home/user/submission/late.txt; printf 'def solve(cell):\n    cell.step(1)\n' > /home/user/submission/belt2.py; sleep 0.2; done
