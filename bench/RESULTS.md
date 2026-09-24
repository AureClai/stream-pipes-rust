# Benchmark results: pipe-stream vs Stream Python

Produced by `bench/compare_with_stream_python.py --repeats 7 --py-repeats 3
--aligned` (method and how to rerun: [README.md](README.md)). Both engines
simulate the same vehicles, with one pipe per link.

## Summary

- **Speed**: pipe-stream is **380 to 3,400 times faster** than Stream
  Python's event loop. It handles 2.5 to 6.4 million node passages per
  second, against 1,300 to 6,700 for Stream Python.
- **Scaling**: on linear corridors the speed-up stays around ×1,000 from 10
  to 100 links, because Stream Python's cost per passage barely depends on
  network size there. On grids, Stream Python slows as the network grows
  (1,900 down to 1,300 passages/s) while pipe-stream does not, so the
  speed-up rises from ×1,760 (3×3) to ×3,350 (8×8).
- **Agreement**: in free flow, on a bottleneck with spillback and on
  diverges without a blocked branch, the trajectories are **identical to
  machine precision**. This holds on 7 of the 10 scenarios, including the 5×5
  and 8×8 grids. The three that differ, `merge`, `grid_3x3` and
  `real_network`, come from two node rules where the engines deliberately
  part ways (see below).

Machine: x86_64, Linux-6.18.44-fc-v37-x86_64-with-glibc2.39; Python 3.11.15, NumPy 1.26.4. pipe-stream: median of 7 runs; Stream Python: median of 3 run(s); seed 42.

## Speed

| Scenario | Nodes | Links | Vehicles | Node passages | Stream Python | pipe-stream | Speed-up | Python passages/s | pipe-stream passages/s |
|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| `bottleneck` | 3 | 2 | 2,499 | 5,490 | 0.82 s | 2.16 ms | **×382** | 6,671 | 2,546,134 |
| `diverge` | 4 | 3 | 1,600 | 4,746 | 0.71 s | 0.76 ms | **×937** | 6,643 | 6,225,160 |
| `merge` | 4 | 3 | 2,000 | 5,421 | 0.91 s | 1.98 ms | **×459** | 6,026 | 2,733,781 |
| `real_network` | 12 | 12 | 2,991 | 11,715 | 2.20 s | 3.87 ms | **×568** | 5,372 | 3,030,073 |
| `chain_10` | 11 | 10 | 1,499 | 16,401 | 2.79 s | 2.55 ms | **×1,094** | 5,885 | 6,435,649 |
| `chain_50` | 51 | 50 | 1,499 | 74,341 | 14.08 s | 14.49 ms | **×971** | 5,281 | 5,129,854 |
| `chain_100` | 101 | 100 | 1,499 | 143,016 | 27.91 s | 27.89 ms | **×1,001** | 5,123 | 5,127,907 |
| `grid_3x3` | 9 | 24 | 2,998 | 8,955 | 4.70 s | 2.67 ms | **×1,762** | 1,906 | 3,355,085 |
| `grid_5x5` | 25 | 80 | 2,998 | 26,590 | 16.81 s | 8.15 ms | **×2,064** | 1,581 | 3,263,262 |
| `grid_8x8` | 64 | 224 | 2,998 | 43,816 | 33.82 s | 10.09 ms | **×3,351** | 1,295 | 4,341,626 |

## Agreement

| Scenario | Completed (Py / Rust) | Mean travel time (Py / Rust) | Incl. entry wait (Py / Rust) | Passage-time MAE | Max error | Exact passages |
|---|--:|--:|--:|--:|--:|--:|
| `bottleneck` | 1,760 / 1,760 | 334.3 / 334.3 s | 572.5 / 572.5 s | 1.09e-16 s | 5.68e-14 s | 100.0 % |
| `diverge` | 1,564 / 1,564 | 80.0 / 80.0 s | 80.0 / 80.0 s | 0 s | 0 s | 100.0 % |
| `merge` | 1,759 / 1,759 | 252.6 / 202.9 s | 256.7 / 237.2 s | 149 s | 406 s | 23.7 % |
| `real_network` | 2,908 / 2,872 | 74.9 / 100.5 s | 74.9 / 102.9 s | 21.7 s | 206 s | 54.1 % |
| `chain_10` | 1,483 / 1,483 | 40.0 / 40.0 s | 40.0 / 40.0 s | 2.08e-14 s | 4.55e-13 s | 100.0 % |
| `chain_50` | 1,416 / 1,416 | 200.0 / 200.0 s | 200.0 / 200.0 s | 2.14e-14 s | 4.55e-13 s | 100.0 % |
| `chain_100` | 1,333 / 1,333 | 400.0 / 400.0 s | 400.0 / 400.0 s | 2.2e-14 s | 4.55e-13 s | 100.0 % |
| `grid_3x3` | 1,773 / 1,773 | 112.0 / 101.6 s | 762.1 / 290.7 s | 815 s | 2.38e+03 s | 1.4 % |
| `grid_5x5` | 2,910 / 2,910 | 106.7 / 106.7 s | 106.7 / 106.7 s | 2.12e-14 s | 4.55e-13 s | 100.0 % |
| `grid_8x8` | 2,844 / 2,844 | 186.7 / 186.7 s | 186.7 / 186.7 s | 2.16e-14 s | 4.55e-13 s | 100.0 % |

## Agreement with Stream Python aligned on pipe-stream's node rules

Deterministic merge + causal passages (`--aligned`, see `aligned_python` in the driver).

| Scenario | Completed (Py / Rust) | Mean travel time (Py / Rust) | Passage-time MAE | Max error | Exact passages |
|---|--:|--:|--:|--:|--:|
| `bottleneck` | 1,760 / 1,760 | 334.3 / 334.3 s | 1.09e-16 s | 5.68e-14 s | 100.0 % |
| `diverge` | 1,564 / 1,564 | 80.0 / 80.0 s | 0 s | 0 s | 100.0 % |
| `merge` | 1,759 / 1,759 | 202.9 / 202.9 s | 0.00627 s | 2 s | 99.7 % |
| `real_network` | 2,883 / 2,872 | 98.0 / 100.5 s | 2.92 s | 22.9 s | 55.2 % |
| `chain_10` | 1,483 / 1,483 | 40.0 / 40.0 s | 2.08e-14 s | 4.55e-13 s | 100.0 % |
| `chain_50` | 1,416 / 1,416 | 200.0 / 200.0 s | 2.14e-14 s | 4.55e-13 s | 100.0 % |
| `chain_100` | 1,333 / 1,333 | 400.0 / 400.0 s | 2.2e-14 s | 4.55e-13 s | 100.0 % |
| `grid_3x3` | 1,773 / 1,773 | 101.6 / 101.6 s | 0.00581 s | 2 s | 99.7 % |
| `grid_5x5` | 2,910 / 2,910 | 106.7 / 106.7 s | 2.12e-14 s | 4.55e-13 s | 100.0 % |
| `grid_8x8` | 2,844 / 2,844 | 186.7 / 186.7 s | 2.16e-14 s | 4.55e-13 s | 100.0 % |

## Reading the agreement tables

*Passage-time MAE* is the mean absolute difference between the two engines'
node passage times, over every node that both engines saw a vehicle cross.
*Exact passages* is the share of those differences below 1 µs. *Incl. entry wait*
measures travel time from the demanded departure time, so it counts the
time spent queuing at the entry before the vehicle gets onto the network.

The differences come from two node rules:

1. **Congested merges.** When every candidate at a merge is congested,
   Stream Python draws the vehicle that passes at random, weighted by the
   Daganzo coefficients (`Stochasticity.Merge`; its deterministic branch is
   not implemented). pipe-stream serves the incoming links in a fixed
   order. The same throughput leaves the merge in both engines (identical
   *Completed* counts), but the queue is split differently between the
   branches, and that changes per-vehicle times and entry waits. This
   explains **all** of `merge` and `grid_3x3`: with a deterministic merge in
   Stream Python (aligned table), the MAE falls from 149 s and 815 s to
   about 6 ms, and 99.7 % of passages are exact. The rest comes from a few
   equal-time ties resolved differently (max 2 s).
2. **Non-causal passages in Stream Python.** With `ActiveUpStreamCapacity`
   off (the default), Stream Python bounds a vehicle's passage time only
   by its own arrival and the downstream supply, not by the previous
   vehicle of the same incoming link. Behind a head vehicle blocked at a
   diverge, the next vehicle is processed after it, in FIFO order, but
   **time-stamped before it**. In `real_network`, vehicle 1254 crosses
   node 6 at 25 819.29 s, an event processed after vehicle 1253 had already
   crossed at 25 824.09 s. The follower virtually overtakes a blocked leader,
   and that shortens travel times (74.9 s against 100.5 s). pipe-stream
   enforces causality. With both rules aligned, `real_network` goes from
   21.7 s to 2.9 s MAE, and mean travel time from 74.9 s to 98.0 s against
   100.5 s.

**Open point:** a residual 2.9 s MAE on `real_network` (2.5 % of the mean
travel time), which first appears at the congested diverge node 6. It does not
come from the storage-fraction correction of the spillback delay:
Stream Python's `(dn_exact - dn) * C` is dimensionally off compared with
pipe-stream's `(dn_exact - dn) / C`, but replacing it barely changes the
result. It needs a closer investigation.

## Caveats

- Timings come from one cloud VM (4 vCPU Xeon @ 2.1 GHz). Treat the ratios
  as orders of magnitude, not as precise constants.
- Only the event loops are timed. Stream Python's validation, assignment and
  initialisation, and pipe-stream's JSON loading, are left out.
  pipe-stream's time includes `Simulation::new`.
- Stream Python needs NumPy < 2 (see README).
- Everything runs in single-stream mode (one pipe per link), the only
  configuration Stream Python can represent. Pipes, class-restricted lanes
  and friction are not benchmarked here.
