# Natural Navigation: Author Notes

This fork starts at `456df220ed39b6db8b6f2ef222d648ea22719e7d`. The flange,
detachable collets, load-preserving weld, cargo shutter, 1.80 m rigid litter,
two perpendicular offset loading throats and whole-board receiving condition
are unchanged. Skirting is registered as solid geometry, like the walls it
covers. No fiducial, localization event or estimator threshold is graded.

## What changed

There are no position squares in staging, the entire wing or the turning
pocket. Quarry slabs and mezzanine terrazzo are laid as continuous finishes;
the seams do not restart in each room. Slabs use different cuts of the same
licensed concrete material. The finish recipes are shared across cases, not
encoded from a site or room identifier. Skirting follows the actual walls
and terminates at openings. Wear plates extend from the physical thresholds;
their role is to distinguish ordinary doors, landings and shutters.

## What the reference observes

`solution/architecture.py` uses no site assets or simulator imports. At a
level, steady heading it matches normalized brightness patches in overlapping
down views, searches around the command prediction, and estimates translation
from patches agreeing on a displacement. Blank or inconsistent patches cast
no vote. The floor-plane camera calibration gives displacement in metres.
The pilot applies the accumulated correction after a level segment. If some
parts of the floor are occluded, it uses the command prior for those pairs;
it does not pretend to observe height-changing segments this way.

On first visits it uses the visible wall-floor junction and, where visible,
both sides of a doorway. On revisits it matches the current view against a
camera view it saved in its own topological map, falling back to the walls
when appearance does not agree. This is local registration with a small
command prior, not global recognition from an arbitrary starting position.
Equipment, extraction and loading use the parent task's RGB detectors. The
additional viewing maneuvers have 2,400 actions and 5,000 total calls; the
solver still has 30 minutes.

## Independent checks

`tests/review/check_natural.py SEED OUTPUT --paired` flies the RGB policy and
a fresh matched world. The latter continues to look, survey, choose, grasp,
scan doors, measure the load axis and read the loading jambs; only visual
localization corrections after pickup are withheld. The referee checks the
identical action prefix through lift. It records independent physical poses
for displacement-error diagnostics, real RGB pairs, camera revisit views,
stages, raw action tapes and contact outcomes. None go into the pilot.
Default author mode renders consumed camera pairs at their original physics
timestep; `--full` renders both cameras on every action, as the service does.
With `--odometry`, the same pilot withholds both kinds of localization
corrections from takeoff. Missing matches do not terminate any of the modes.

`test_architecture.py` covers translation under illumination change and
camera-fixed occlusion, rejection of a blank view, and depth and jamb offset
at an interrupted wall. `test_routes.py` covers the inherited navigation
graph. Native ALE controls and blind runs are separate from these checks.

## Author results

Eight sites were selected before rollout: the first new draw in each trim
sign / flange / loading-exit-side combination. The two development sites
were separate. No held-out site was replaced. All eight RGB flights
surveyed, chose the fitting without a miss, revisited, lifted, crossed
both loading throats and settled the whole board. They used 857-1,670
actions. Across 300 level segments the mean displacement error against
independent physics truth was 1.34 cm using the visual estimate (including
its command fallbacks), versus 8.70 cm using the nominal command. The
pilot registered 1,379 of 1,434 moving image pairs and matched 210
previously observed room views.

Withholding corrections from takeoff delivered 0/8: two physical wall
impacts, five searches with no reachable frontier in the pilot's map, and
one inconsistent doorway revisit. Withholding only after pickup delivered
4/8; the other four stopped on inconsistent revisits, not impacts. All
eight post-pickup controls matched the respective RGB actions through lift.
Thus the result is about reliable navigation over the whole sortie. It
does not establish that every leg needs visual correction, that direct
wall-based alternatives are impossible, or that a blind model will fail.
