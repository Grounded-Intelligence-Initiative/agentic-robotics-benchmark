"""The verify stage's process kit (grader side, stdlib only): how the program under test is run and served.

  * `wire`: the JSON-lines RPC between the program (caller) and the grader's cell server (server), with the
    program-time cap, a bounded request queue and the non-finite-number rejection;
  * `runner`: the run regime of the program (account, per-run directory, minimal environment, launcher, limits, the
    IPC sweep, clean-up);
  * `lockdown`: the verify-start lockdown (kill the agent account, snapshot the submission, late-write gate);
  * `verify_freeze`: keeps the agent account stopped until the verdict is written;
  * `netaudit`: the static and runtime network audit of the program;
  * `nonet`: the no-network seccomp filter every process of the program runs under, and its self-test;
  * `seeds`: the hidden episode seeds (the fixed hidden set pinned in tests/hidden/seeds.json).

Adapted from the benchmark's tasks/arm/shoe_lacing_and_boxing and tasks/arm/pump_skid_startup_and_operation
tests/cellkit/ (commit b7b55acb98cf), hardened here (bounded request queue, blank lines, monotonic deadlines, the
program's oom_score_adj, the file-size limit).
"""
