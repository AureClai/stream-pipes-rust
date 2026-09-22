# Case 4 — NDW per-lane validation (in preparation)

Validates the **per-pipe distribution itself** against per-lane detection — the
top rung of the paper's resolution ladder, currently reported *not-exercised*.
The Givors ablation shows the partition improves aggregate GEH; this case
scores the per-pipe split (Fig. 9d-type output) directly against per-lane
loop data, and reads the friction coefficient φ off through-lane flow during
real off-ramp spill episodes.

## Why NDW

The Dutch Nationale Databank Wegverkeersgegevens serves historical loop data
via **Dexter** (dexter.ndw.nu): minute resolution, **per lane** (rijstrook)
and per vehicle category, motorway coverage at ~300–500 m detector spacing,
from 2010 to a few hours ago, CSV/Excel export. No other open source combines
per-lane resolution, minute aggregation, and corridor-dense coverage.

## Data access (action required — cannot be automated from this machine)

1. Register a free account at **mijnNDW** (https://www.ndw.nu → Dexter).
   "Everyone" role suffices for intensity/speed exports.
2. In Dexter, use the *Exports* module → "Intensiteit en snelheid":
   select the corridor's measurement sites, the study period, **minute**
   aggregation, and per-lane breakdown. Download as CSV.
3. Drop the export(s) into `scenarios/ndw/data/raw/` (git-ignored).
4. Run `python prepare_dexter_export.py` to normalise and screen (S1–S4).

Note: this workstation's network proxy blocks opendata.ndw.nu downloads
(Trustlane block page observed 2026-07-21); use the Dexter web UI from an
unrestricted connection.

Useful references:
- Dexter manual: https://docs.ndw.nu/handleidingen/DEXTER/
- Intensity/speed exports: https://docs.ndw.nu/handleidingen/DEXTER/intensiteiten-en-snelheden/
- Measurement-site table (locations, lane counts): `measurement.xml.gz` on
  https://opendata.ndw.nu/ and `ndw_avg_meetlocaties_shapefile.zip`.

## Candidate corridors (final pick after a first data exploration in Dexter)

Selection criteria, in order: (a) recurrent **off-ramp queueing** with
documented spillback onto the mainline — the regime where the pipe
distribution is discriminating; (b) all corridor entries/exits measured, so
S1 conservation closure is achievable; (c) ~300 m detector spacing upstream
of the diverge; (d) prior literature for cross-checking.

1. **A15 eastbound, exit Papendrecht** — off-ramp between two of five
   detectors at ~300 m spacing; among the most congested Dutch links;
   layout documented in arXiv:1909.12782.
2. **A20 Rotterdam → Gouda (Nieuwerkerk a/d IJssel area)** — 11 km, 4
   on-ramps, 3 off-ramps, 32 detectors at ~300 m; recurrent AM breakdown;
   used as case study in arXiv:1509.06146 and the dynamic-speed-limit
   literature. Richer but larger; good second corridor.

## Experiments (mapped to paper claims)

- **E1 — Lane-flow distribution.** Free-flow periods: per-lane observed flow
  shares vs the keep-right assignment rule (paper §4.2; Duret et al. 2012
  analogue on Dutch data). Exercises choice points 1–2.
- **E2 — Per-pipe distribution under exit queueing.** During off-ramp queue
  episodes: observed shoulder-lane vs median-lane flows/speeds at the
  approach detectors vs the simulated per-pipe split — the direct validation
  of the Fig. 9d-type output that Givors could not score.
- **E3 — Direct φ read-off.** During spill episodes (shoulder-lane speed
  collapsed, median lanes moving): through-lane discharge past the queue
  = φ·C_p by the model's own claim ("observable, not merely fittable",
  paper §4.3). Compare the read-off value with a calibrated fit, and with
  the Jin (2010) 8–18 % bracket.
- **E4 — GEH scoring at per-lane granularity.** The standard corridor
  validation (boundary gates, screened demand), scored per lane rather than
  per carriageway.

## Screening protocol adaptation (S1–S4, paper §5.3)

- S1 closure now holds **per carriageway** (sum over lanes) — lane-level
  conservation does not hold (mid-link lane changes are real); this is
  itself the committed-lane approximation's error meter: report the
  lane-level residual between adjacent detectors as the observed lane-change
  intensity, and use it to bound where the approximation is credible (G2).
- S3 gains a per-lane check: a lane whose flow is frozen or whose speed
  never varies is a dead per-lane channel even when the site total looks
  plausible.
- S4: exit-share endogeneity as at Givors, plus rush-hour hard-shoulder
  running (spitsstrook) — a lane that exists only part of the day must be
  screened out or modelled as a scheduled pipe (the schedule mechanism
  supports this natively; note it as an opportunity, not a blocker).

## Directory layout

```
scenarios/ndw/
  README.md                  this file
  prepare_dexter_export.py   Dexter CSV → normalised per-lane minute series
                             + S1/S3 screening report
  data/raw/                  Dexter exports (git-ignored)
  data/screened/             accepted series + screening report (checksummed)
  network/                   corridor network + pipe partition (after site pick)
```
