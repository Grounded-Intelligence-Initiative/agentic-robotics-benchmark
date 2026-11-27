# Record a Session

> Every state sample and every command as JSON lines, and how to read the file back.

- Section: Python SDK

`robot.record(path)` writes every state sample the robot sent and every command the script
sent to one file, one JSON object per line, in order. Nothing is sampled or averaged.

`record_and_replay.py` records a few seconds of state, then reads the file back. It sends
nothing to the robot:

```python
# Every state sample and every command sent while the block runs goes to PATH.
with Robot().connect() as robot, robot.record(PATH) as recording:
    time.sleep(SECONDS)
print(f"recorded {recording.samples} states and {recording.commands_written} commands")

# Each line is one JSON object; "kind" says whether it is a state or a command.
lines = list(load(PATH))
states = [line for line in lines if line["kind"] == "state"]
print("robot modes seen:", dict(Counter(s["mode"] for s in states)))
if len(states) > 1:
    rate = (len(states) - 1) / (states[-1]["t"] - states[0]["t"])
    print(f"state rate {rate:.0f} Hz")
```

From `examples/record_and_replay.py` in the maker-sdk repository (the excerpt the page shows).

`record` is a context manager: recording stops when the block exits, including when it exits
because of an exception, which is the run you want the file for. `recording.samples` and
`recording.commands_written` count what went in.

## Read It Back

`maker.humanoid.recording.load(path)` yields one dict per line. `kind` is `"state"` or
`"sent"`, `t` is `time.monotonic()` when the line was written (subtract the first line's `t` for
the time since the recording began), and a state line carries the
robot mode as `mode` beside the fields of `State`.

## What It Is Good For

| Use | Why |
| - | - |
| **A bug report** | a fall or a robot that did not move is easier to read from the file than to describe |
| **Comparing runs** | the same script on two robots, or before and after a change, gives two files with the same shape |
| **Checking a loop** | the `t` of the `sent` lines shows whether a control loop held its rate |

## Related

- Read State: the fields in a state line
- SDK Reference: `record`, `Recording` and `load`
