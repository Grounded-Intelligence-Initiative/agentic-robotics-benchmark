# Split-deck development registration

Parent ba84de428b125f8538cf24cb31af4bf550107a16. Before any simulation or
tuning: retain all 24 seeds and their wiring order. Use the existing first
two entries, in order, as development pairs. Index 0 (1699855537204788234)
has the outboard runner on physical left; index 1 (2229355922383608593) has
it on physical right. Even/odd seed determines the runner, not the port map.
Do not replace a failed case or retry official validation for a better draw.

Hypothesis: the first camera inspection over two staggered steel runners
requires changing the foot track to load both surfaces. The operator can see
the opening, the plates and the outboard nosing. No fixed gait, foot identity,
contact order or additional points. A miss can leave a foot on the real bed.
Keep the original 14 milestones, arrival centre, yaw, dwell and recovery.

Use one CPU worker. Run the two RGB-feedback positives through dock
1 first, then the full route on both for the sealed candidate. Repeat the
first transfer with the same probes, visual steering, fall handling and pose
opportunity as its adaptive partner, but ordinary foot track. Record early
success, partial progress, bed contacts, loaded top contacts and falls; a
negative is allowed to succeed. A diagnostic horizon applies only to these
author checks. Retain each development iteration in a fresh directory, even
if a case or implementation fails. The engine validation is separate, on an
unforced draw, with its original oracle and untouched agent pair.

Secondary comparator, added after the ordinary-track pilot but before running
this comparator: use the complete adaptive motion, always for the left-hand
deck, regardless of the observed nosing. Both cases, same matched horizon.
The left branch should reproduce its positive even if the right branch fails.
