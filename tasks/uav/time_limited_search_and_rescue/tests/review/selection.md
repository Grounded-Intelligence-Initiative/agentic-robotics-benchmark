# How the graded sites are chosen (written before any candidate was generated)

The grader flies a fixed set of eight sites, `tests/hidden/sites.json`. They are chosen by this rule; the rule was
committed before the candidates were generated, and `select_sites.py` applies it and writes `tests/hidden/`.

1. **Candidates.** The 96 site seeds `39000000 + k`, `k = 0 .. 95`. Nothing else is looked at. Commissioning sites
   are drawn with bit 63 set, so none of them can be a candidate.
2. **The reference must complete a site.** The privileged pilot (`solution/oracle/pilot.py`) flies every candidate
   once. A candidate it does not hand off is dropped and counted in `selection.json`; the rule does not retry it and
   does not change the pilot afterwards for the sake of a candidate.
3. **Strata.** A site has three two-valued properties that change what a program must do: the mirror of the
   building (seed bit 1), which receiving station is open (seed bit 2), and the flange size (small or large, from
   the site's layout). That gives eight strata. One site is taken from each stratum, so both values of each
   property appear four times.
4. **Spread over route length.** Within a stratum the candidates are ordered by the number of actions the privileged
   pilot needed (ties: the lower seed first). Stratum `i` (0 .. 7, the strata ordered by mirror, station, flange)
   takes the candidate at rank `floor((i + 0.5) / 8 * n_i)` of its `n_i` candidates. The eight sites therefore run
   from short routes to long ones instead of clustering around the median.
5. **No look at anything else.** No image, no model result and no unprivileged pilot's result enters the choice. If
   a stratum is empty the candidate range is extended by another 96 seeds and the rule is applied again from step 1
   on the larger range; this is recorded.

`selection.json` records the candidates' count, how many the reference completed, each stratum's size and the rule's
version. `candidates.json` keeps, per candidate, the stratum and the reference's action count. `sites.json` holds the
eight seeds in stratum order. The anchor is the reference's mean progress on the eight sites.
