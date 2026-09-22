# NGSIM behavioral micro-checks (trajectory data)

Role in the paper: **not a fourth GEH corridor.** NGSIM sites are short
weaving sections — exactly where the committed-lane approximation is weakest
— and each recording is ~15–45 min. What trajectories buy instead is direct
measurement of the *behavioral quantities the model assumes or aggregates*:

| Experiment | Measures | Feeds paper claim |
|---|---|---|
| N1 lane-flow distribution | per-lane minute flow/speed from virtual loops | keep-right/keep-left assignment rule (§4.2, Duret et al. 2012 analogue) |
| N2 lane-commitment distance | where exit-bound vehicles complete their last lane change upstream of the off-ramp | G1's ~300–500 m commitment distance — currently asserted, would become measured |
| N3 late insertions | share of exiting vehicles changing into the exit lane in the last ~150 m | the friction mechanism's second micro-foundation (Laval–Daganzo voids) |
| N4 lane-change intensity | LC events per km per min by 100 m bin | committed-lane approximation error meter; Jin (2010) 8–18 % bracket for φ |
| N5 HOV lane contrast (I-80) | lane 1 (HOV) vs GP lanes flow/speed | class-permission masks against a real reserved lane |

## Data

Source: US DOT open data (no account, no license gate):
https://data.transportation.gov/resource/8ect-6jqj — Socrata API, dataset
"NGSIM Vehicle Trajectories". Confirmed reachable from this machine
(2026-07-21). Row counts: us-101 4 802 933, i-80 4 566 387.

Sites:
- **US-101** (Hollywood Fwy, LA; 2005-06-15, 07:50–08:35, three 15-min
  periods): ~640 m, 5 mainline lanes (lane_id 1–5, 1 = median/leftmost),
  auxiliary lane 6 between the Ventura on-ramp (7) and Cahuenga off-ramp
  (8). The off-ramp + aux lane makes it the primary site for N2/N3.
- **I-80** (Emeryville; 2005-04-13, 16:00–16:15 & 17:00–17:30): ~500 m,
  6 lanes, **lane 1 is HOV** (N5), lane 7 on-ramp merge.

Units: feet and feet/s (converted to metric in the loader);
frames at 0.1 s; `global_time` in epoch milliseconds.

Known caveats (report with any result): NGSIM raw trajectories carry
documented positioning/differentiation noise (velocities smoothed from
positions); lane_id is derived from position and flickers near lane
boundaries — the analysis debounces lane changes (a change must persist
≥ 2 s to count). US lane discipline permits passing on both sides, so N1
informs the *mechanism* (occupancy-driven choice), not the European
keep-right asymmetry itself.

## Pipeline

```
python fetch_ngsim.py --location us-101      # paginated, resumable download
python analyze_trajectories.py --location us-101
python make_fig_ngsim_behaviour.py            # behavioural panel figure

# replication: observed demand through the engine, no calibrated parameter
python build_us101_demand.py                  # -> data/out/us101_sim_input.json
cargo run --release --example ngsim_us101 > data/out/us101_sim_trace.csv
python make_fig_us101_replication.py          # replication figure + GEH summary
```

Replication results (2026-07-21): 6 071 observed vehicles injected, boundary
gate from observed 630 m discharge, u/c from the data envelope, phi = 1.
Throughput at 300 m: 8/8 five-minute windows GEH < 5 (mean 2.0, max 3.5).
Per-lane shares: interior lanes reproduced; aux under-predicted (1.7 % vs
5.7 % — through weaving excluded by the committed-lane approximation);
median under-loaded (4 % vs 18 % — myopic keep-right occupancy choice).

- `fetch_ngsim.py` — stdlib-only Socrata pager (500k rows/page, ordered by
  :id, retry + resume via manifest), writes `data/raw/<loc>_p####.csv`.
- `analyze_trajectories.py` — loads a location, converts units, then:
  virtual per-lane loops at configurable stations (minute flow + mean
  speed), N2 commitment-distance distribution, N3 late-insertion share,
  N4 lane-change intensity per 100 m bin. Emits `data/out/<loc>_metrics.json`
  and per-lane minute series CSV.

Directory layout mirrors scenarios/ndw/ (that folder holds the loop-data
plan pending NDW/MIDAS access; first source to land becomes Case 4 — NGSIM
here is the behavioral appendix either way).
