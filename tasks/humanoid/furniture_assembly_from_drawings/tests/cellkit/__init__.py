"""The verify stage's process kit (grader side, stdlib only). The online variant's grader runs no program and uses:

  * `lockdown`: the verify-start lockdown (kill the agent account; its helpers);
  * `verify_freeze`: keeps the agent account stopped until the verdict is written;
  * `runner`: the stage lock (lock_stage / unlock_stage) and the rest of the offline run regime of a program.

Adapted from the arm assembly tasks' copies (robotic_arm_full_assembly_so101_panda: arm/assembly_cell/runner.py,
arm/assembly_six/lockdown.py, arm/verify_freeze.py). The offline grader's wire and network audit are not needed here.
"""
