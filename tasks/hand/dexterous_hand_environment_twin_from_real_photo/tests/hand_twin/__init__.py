"""The grader of the hand-twin task (verify stage only; never shown to the agent).

Modules:
    load       the public payload modules (gate, settle, pieces, instrument) and the one pipeline up to the twin's
               settled pieces in its own support-top frame
    register   reference points and the registration onto the true support frame
    truth      the hidden scene files: readers, the truth's MJCF with a physics variant, the reference pieces
    use_score  the use of the pick table (units, weights, the cap per thing, stop charges)
    selftest   the verify-time self-test of the truth files
    grade      the stages of one grade (self-test, gate and settle, registration, requests, lifts, use) and the
               worker process verify.py starts
    dump       the opt-in grade dump (every plan and outcome; no clock value but the per-stage CPU seconds)
Nothing here runs at import.
"""

FORMAT_GRADE = "hand-twin-grade/1"
FORMAT_PROGRESS = "hand-twin-worker-progress/1"
FORMAT_JOB = "hand-twin-worker-job/1"
