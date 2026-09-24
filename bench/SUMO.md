# Benchmark results: pipe-stream vs Eclipse SUMO

Produced by `bench/compare_with_sumo.py --repeats 5` (method and how to
rerun: [README.md](README.md#sumo)). pipe-stream, SUMO meso (`--mesosim`)
and SUMO micro (Krauss, 1 s step) simulate the same vehicles, with the same
routes and the same departure times. SUMO is timed with its own `Duration`
statistic, which leaves out network loading and has 10 ms resolution. The
Stream Python column comes from [RESULTS.md](RESULTS.md).

## Summary

- **Speed**: pipe-stream is **×4 to ×84 faster than SUMO meso** and
  **×37 to ×430 faster than SUMO micro**. The gap is widest on congested
  scenarios. On long free-flow corridors it narrows, down to ×4 on
  `chain_100`. There, pipe-stream's cost grows with the number of node
  passages (4 → 27 ms from 10 to 100 links), while SUMO meso's cost barely
  moves (0.09 → 0.11 s), because a meso segment is crossed in one event
  however long it is.
- **Free flow**: all three engines agree. Travel times match within 0.6 s
  (`chain_*`, `diverge`) once SUMO's vehicle type is calibrated on
  Stream's fundamental diagram (see below).
- **SUMO meso against pipe-stream**: close on uncongested networks, the
  5×5 and 8×8 grids included (107.2 s against 106.7 s, and 187.2 s against
  186.7 s). On congested scenarios, SUMO meso lets fewer vehicles through
  (about −10 %) and holds them longer before insertion.
- **SUMO micro**: explicit junction and lane-change rules lower the
  capacity of merges and crossings. At a merge, the minor road yields:
  995 vehicles completed against 1,759. In the grids, crossing flows at
  priority junctions leave 1,200 to 1,500 vehicles uninserted. This is a
  modelling difference, not a calibration error.

Eclipse SUMO sumo 1.27.1; Linux-6.18.44-fc-v37-x86_64-with-glibc2.39. Median of 5 runs per engine.

## Speed

| Scenario | Nodes | Links | Vehicles | pipe-stream | SUMO meso | SUMO micro | Stream Python | meso / pipe-stream | micro / pipe-stream |
|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| `bottleneck` | 3 | 2 | 2,499 | 1.98 ms | 0.16 s | 0.43 s | 0.82 s | **×81** | **×217** |
| `diverge` | 4 | 3 | 1,600 | 1.07 ms | 0.09 s | 0.21 s | 0.71 s | **×84** | **×196** |
| `merge` | 4 | 3 | 2,000 | 1.90 ms | 0.12 s | 0.82 s | 0.91 s | **×63** | **×430** |
| `real_network` | 12 | 12 | 2,991 | 3.86 ms | 0.15 s | 1.31 s | 2.20 s | **×39** | **×340** |
| `chain_10` | 11 | 10 | 1,499 | 4.05 ms | 0.09 s | 0.15 s | 2.79 s | **×22** | **×37** |
| `chain_50` | 51 | 50 | 1,499 | 14.12 ms | 0.11 s | 0.54 s | 14.08 s | **×8** | **×38** |
| `chain_100` | 101 | 100 | 1,499 | 27.38 ms | 0.11 s | 1.05 s | 27.91 s | **×4** | **×38** |
| `grid_3x3` | 9 | 24 | 2,998 | 3.37 ms | 0.22 s | 0.86 s | 4.70 s | **×65** | **×255** |
| `grid_5x5` | 25 | 80 | 2,998 | 7.17 ms | 0.12 s | 1.55 s | 16.81 s | **×17** | **×216** |
| `grid_8x8` | 64 | 224 | 2,998 | 12.62 ms | 0.16 s | 2.82 s | 33.82 s | **×13** | **×224** |

## Outputs

| Scenario | Completed (pipe-stream / meso / micro) | Not inserted at end (meso / micro) | Mean travel time (pipe-stream / meso / micro) | Incl. entry wait (pipe-stream / meso / micro) |
|---|--:|--:|--:|--:|
| `bottleneck` | 1,760 / 1,581 / 1,637 | 763 / 822 | 334.3 / 326.2 / 84.3 s | 572.5 / 701.2 / 661.6 s |
| `diverge` | 1,564 / 1,563 / 1,564 | 0 / 0 | 80.0 / 80.4 / 80.0 s | 80.0 / 80.8 / 80.4 s |
| `merge` | 1,759 / 1,580 / 995 | 255 / 832 | 202.9 / 301.3 / 111.6 s | 237.2 / 417.3 / 112.0 s |
| `real_network` | 2,872 / 2,717 / 2,092 | 72 / 553 | 100.5 / 113.1 / 116.8 s | 102.9 / 167.6 / 117.2 s |
| `chain_10` | 1,483 / 1,482 / 1,482 | 0 / 0 | 40.0 / 40.6 / 40.0 s | 40.0 / 41.0 / 40.4 s |
| `chain_50` | 1,416 / 1,416 / 1,416 | 0 / 0 | 200.0 / 200.6 / 200.0 s | 200.0 / 201.0 / 200.4 s |
| `chain_100` | 1,333 / 1,332 / 1,332 | 0 / 0 | 400.0 / 400.6 / 400.0 s | 400.0 / 401.0 / 400.4 s |
| `grid_3x3` | 1,773 / 1,609 / 1,417 | 1,330 / 1,511 | 101.6 / 126.7 / 96.0 s | 290.7 / 860.6 / 134.8 s |
| `grid_5x5` | 2,910 / 2,908 / 1,433 | 0 / 1,365 | 106.7 / 107.2 / 159.7 s | 106.7 / 107.6 / 214.9 s |
| `grid_8x8` | 2,844 / 2,842 / 1,372 | 0 / 1,211 | 186.7 / 187.2 / 298.4 s | 186.7 / 187.6 / 312.3 s |

*Completed* counts vehicles that reached their destination before the end
of the simulation. *Not inserted at end* counts vehicles SUMO was still
holding at their origin. The two travel-time columns only cover completed
vehicles. When many vehicles are never inserted (SUMO micro on `merge` and
on the grids), those averages are **biased low**: the vehicles stuck the
longest are missing from them. Compare throughput first on those rows.

## Calibration

SUMO gets one vehicle type per scenario, derived from the median link
fundamental diagram (u, C, kx):

- `length + minGap = 1/kx`, the jam spacing (6.67 m for kx = 0.15 veh/m);
- `tau = 1/C − (1/kx)/u`, so that the Krauss saturation headway equals
  1/C. With C = 0.5 veh/s, a lane carries 1,800 veh/h, as in Stream. With
  SUMO's default vehicle it carries about 2,800;
- `sigma = 0`, `speedFactor = 1`, `speedDev = 0`: no driver imperfection
  and no speed distribution, so every vehicle drives at the speed limit.
  With SUMO's defaults, free-flow travel times come out 17 to 34 %
  longer.

Other settings: `departLane="best"`, `departSpeed="max"`, junctions of
type `priority`, `--no-internal-links`, no U-turns, and
`--time-to-teleport -1`. Without that last option, a vehicle blocked for
300 s would jump ahead, where the other engines leave a jam as a jam.

## Caveats

- Different models: SUMO micro is a car-following model with explicit
  junction conflicts, and SUMO meso is a queue model per edge segment.
  pipe-stream and Stream are exact event-based LWR models. Only aggregate
  outputs can be compared; per-vehicle trajectories cannot.
- `real_network` mixes links with different fundamental diagrams, but
  SUMO gets a single vehicle type calibrated on the median one.
- SUMO micro runs with a 1 s step, the usual default. Departures are
  rounded to that step (`departDelay` < 1 s). A smaller step would make
  SUMO micro proportionally slower.
- Timings come from one cloud VM (4 vCPU Xeon @ 2.1 GHz), single-threaded.
  SUMO's 10 ms resolution makes its meso ratios approximate on the
  smallest scenarios.
