# Site brief: ten household things on a wooden board

## The customer and the job

A service-robot team will send an arm carrying a four-digit robot hand to a client's site, to clear ten household
things from a wooden board laid on a table. The hand picks one thing at a time: a pinch between the index finger and
the thumb, or a hook through a handle. Every pick is rehearsed first in simulation, on a digital twin of the board as
it was found, and only picks that work in the twin are sent to the robot.

The site cannot be visited again. The team's calibrated survey camera took one colour photo of the board before
anything was touched (`photo.png`; the camera's calibration is in `camera_sheet.json`). That photo is all there is: no
depth, no measurements, no camera pose, no second view. You build the twin from it.

## What is on the board

The ten things, by the names the team uses (the twin's keys are in `twin_contract.md`):

wine glass, teapot, cup, tin, toy remote, tube, candy box, fork, small bottle and clog.

The pick sheet asks for picks of 9 of them; the clog is never picked, but it
is on the board, so the hand must not hit it. There is nothing else on the board.

The support is one flat rigid board. Its size is not given. Of the real board only the plane of its top matters (its
outline and what is under it play no part). Your twin's support is simulated, though, in the settle below: a thing
with nothing under it falls (unless the contract holds it still), and that fall fails the twin, so the support must
reach under the things that rest on it. The table under the board, the keyboard and the chairs around it are not part of the twin.
The arm can reach the board from any side.

## How the twin is used, and what that requires

- **The picks are planned on your twin.** For every scored request of the pick sheet (`pick_sheet.json`: a thing, a
  way of picking it, and a direction, or for a hook a side, the way the hook bar goes through the opening; which
  requests are scored is not said) the team's planner (`hand_instrument.py`) follows one fixed procedure for the
  request's rule. For a pinch it builds one pick from your twin's shape of the thing (where the hand's pads start next
  to the thing and close on it) and keeps it only if the pads and the fingers above them are clear of everything
  around; for a hook it looks through the whole thing along each horizontal direction for the openings the hook bar
  can enter (a handle's, or any other the thing's outline encloses), takes one site in each, tries the sites from the
  roomiest down, skips a site too low over the board and takes the first whose bar is clear. If that fails, the
  request is declined.
- **The planned picks are executed on the real board.** A planned pick then runs exactly as planned on the real
  board. If the real thing is where your twin puts it and as large as your twin makes it, the pads close on it (or
  the hook catches the handle) and the pick is done. If a pad, the finger above it or the hook bar meets something
  real where the pick starts, or starts below the board's top, the robot makes a protective stop: that costs more than a declined pick, because the
  hand has to be freed by a person.
- **So the twin must be right where the hand goes.** Rims and handles where they are and as large as they are (a
  vessel must be hollow, a handle a real loop); where a thing lies on or leans against another, the height and angle
  it really has; and the gaps between neighbouring things as they are.
- **The twin holds the photographed state.** The team starts every rehearsal from your twin after a short settle in
  its standard simulation; a twin whose things slide or topple in that settle is planned in the wrong place.

## What the team does not need

Textures and colours play no part. Exact masses and friction are not needed (the standard simulation uses its own
contact values; each thing's mass must only be plausible for its size). The twin's overall scale is fitted when it is
compared with the real board, so a twin that is uniformly a little too large or too small is not penalised for that.
The camera is not part of the twin: `photo_cam` is optional and serves only your own overlays and the direction frame
of your rehearsals.

The full technical contract (files, frames, the rules for the support and the things, the standard simulation, the
pick instrument and how the twin is assessed) is in `twin_contract.md`; the hand and the instrument's volumes are in
`hand_card.md`.
