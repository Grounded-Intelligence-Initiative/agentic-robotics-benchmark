# Hold the laparoscope: keep the surgeon's instruments in a usable view, from the video alone

You are the engineer of CH-4, a robotic camera holder for laparoscopic surgery. The holder carries the laparoscope in
the camera trocar and moves it with four joints about the trocar's remote centre of motion: pan, tilt, insertion and
roll. Write the controller that does the camera assistant's job: while a procedure runs, keep the surgeon's two
working instruments in a view the surgeon can work in.

Your controller gets, at every control step (0.1 s), **only the endoscope's RGB frame and the holder's own four joint
readings**, and answers with joint rates. It is never told where the instruments are, which instruments are in the
field, what the surgeon is doing or about to do, or which of the two scopes (0 or 30 degrees) is fitted. The frame it
gets is one control step old.

## What a usable view is

A control step's view is usable when (`~/given/holder_manual.md`, section 6, has the exact definitions):

1. both of the surgeon's instrument tips are inside the central safe zone of the image (the middle 60 % of its width
   and of its height);
2. the instruments' apparent size is inside the zoom band (the shaft, 5 mm thick, shows 22 to 44 px wide at the tip);
3. the horizon is level within 15 degrees;
4. while the surgeon does fine work (the tips stay put), the camera does not move the picture faster than 48 px/s;
5. the lens is clean. A lens that touches tissue is smeared: the picture is blurred and stained and no view is usable
   until the scope has been held withdrawn in its cannula for 4 s. Nothing reports a smeared lens but the picture.

An instrument that is being exchanged does not count while it is away (from the moment it starts to withdraw until it
is back at its place); the assistant's third instrument, when there is one, never counts.

**Your score is the fraction of control steps with a usable view, over 12 recorded procedures you have not seen**
(800 steps each). There are no other terms. Every procedure starts with the scope withdrawn at the home pose, and the
place where the surgeon works travels through the cavity: a camera that stays where it is sees almost nothing usable.
The procedures differ in kind (dissection, suturing, exploration, clipping and cutting), in the instruments and
their look, in the tissue, in smoke and highlights, in the scope fitted, in whether instruments are exchanged, and in
whether an assistant's instrument is in the field (manual, section 7). The graded procedures show instrument
finishes, tissue colourings, blood-stained instruments and clutter that the development set does not show.

## What you produce

`/home/user/submission/run.sh` and everything it needs, in that folder. The grader starts it as `bash run.sh` with
the submission folder as working directory, under your account, and speaks one JSON object per line with it. Your
program's **stdout is the wire**: print `READY` when you are ready, then only replies; diagnostics go to stderr.

```
                                            <- READY                    (within 40 s of the start)
{"type": "reset"}                           -> {"ok": true}             (a new procedure begins: forget the last one)
{"type": "act", "t": 0, "obs": {...}}       -> {"action": [pan_rate, tilt_rate, insertion_rate, roll_rate]}
{"type": "act", "t": 1, "obs": {...}}       -> {"action": [...]}
...
{"type": "close"}                           -> {"ok": true}, then exit
```

* `obs["image"]`: the frame, a base64-encoded JPEG, 640 x 480, RGB. `obs["joints"]`: `[pan, tilt, insertion, roll]`
  in rad, rad, m, rad, as they are now. An action is four finite numbers, rad/s, rad/s, m/s, rad/s; it is clipped to
  the holder's rate limits (0.5, 0.5, 0.05, 0.8) and held for one control step.
* `t` is the control step inside the procedure, counted by the grader, 0 after every `reset`. **A step may be asked
  more than once with the same `t`.** On about one step in sixteen the grader asks the step twice, once with the true
  observation and once with a copy, in either order, and nothing on the wire says which is which. The copy is
  rendered like any frame, at the same moment and from the same camera pose, with the same joint readings; only the
  instruments differ: each instrument in the picture is where it would be had it moved over the frame's last step as
  it moved over another step of the same procedure (one with the same kind of work). Both are possible observations.
  Only the reply to the true observation moves the holder. Answer every request from the observation it carries,
  count steps by `t` and not by requests, and do not let one request of a step overwrite the other in your
  controller's memory blindly: keep what each leaves and decide once the next step's frame shows which one it
  continues. Over the whole evaluation at least a tenth of these pairs must differ in the action (by more than 1e-4
  in some component, after clipping to the rate limits), or the evaluation scores 0: this checks that the controller
  reads its input, nothing else. A controller that stands still on both requests of a pair does not differ.
* Time: every reply within 10 s; all replies together within 2400 s (0.25 s a step on average over the 9600 steps;
  the repeated requests count). The grader's clock runs only while it waits for you: the procedure does not move
  while you compute, but when the total is used up the holder is commanded to stand still for everything that is
  left, and those steps are judged as they are.
* A reply that is late, malformed or missing ends your control of that procedure: its remaining steps run with the
  holder standing still and are judged as they are. Before the next procedure the grader restarts your program
  (`run.sh` again, `READY` again), at most three times in the evaluation; after that the remaining procedures run
  with the holder standing still.
* Your program runs in a fresh copy of this environment that holds nothing of yours but `/home/user/submission`: no
  `~/given`, no commissioning service, no file you left elsewhere, no network. The same GPU and the same Python are
  there. Keep the folder small (model weights included), load quickly, and write scratch files under `/tmp` only.

## What you are given

| path | content |
|---|---|
| `~/given/holder_manual.md` | the holder's kinematics and servo, the two scopes and their intrinsics, the operating-room set-up, the exact view rules, what varies between procedures, the development set's format, the commissioning service |
| `~/given/holder_model.py` | the reference implementation of the manual: kinematics, projection, the horizon, the servo, every constant (the grader computes with this same file) |
| `~/given/dev_procedures/` | the development set: ten recorded procedures as endoscope video (a frame every second control step) with labels (every instrument's tip in the image, which instruments count, fine work, a smeared lens, how the rules judge the frame). It is written during the first minutes of your session; `COMPLETE` appears when it is whole |
| `~/given/endocam_client.py` | the client of the commissioning service |
| `~/commission/run_session.py` | runs a controller folder through one commissioning session the way the grader will run it |
| `~/example/` | a minimal controller that speaks the wire and holds the camera still (it scores about 0): a starting point |

The **commissioning service** plays the ten development procedures for your controller, with the grader's
observations, latency, servo and lens, and gives no ground truth, no labels and no judgement of the view. You have 12
sessions, one at a time, each one procedure long at most. The graded procedures are not among the development
procedures.

Tools: Python 3.12 at `/opt/venv/bin/python3` (call it by that path) with numpy, OpenCV, Pillow, PyTorch and
torchvision on the sandbox's GPU. No network. Your session is limited to 4 hours; what is in
`/home/user/submission` when it ends is what is graded.
