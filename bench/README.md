# Benchmarks: pipe-stream vs Stream Python and Eclipse SUMO

Measures how fast pipe-stream runs compared with the Python reference
implementation of Stream, and how closely the two engines agree, on the
same vehicles. Stream is published by Cerema Centre-Est under the CeCILL-B
licence at
[gitlab.cerema.fr/centre-est/mobilite/Stream/stream-python](https://gitlab.cerema.fr/centre-est/mobilite/Stream/stream-python);
these benchmarks were run against the copy at
[github.com/AureClai/stream-python](https://github.com/AureClai/stream-python).

Results: [RESULTS.md](RESULTS.md) (Stream Python), [SUMO.md](SUMO.md)
(Eclipse SUMO, micro and meso) and [DIVERGE.md](DIVERGE.md) (what each engine
does to the mainline beside a blocked off-ramp).

## Method

For each scenario:

1. The network and demand are built as a Stream Python `Inputs` dict.
   Stream Python completes it, assigns the demand (shortest paths, evenly
   spaced entry times) and initialises the simulation.
2. The assigned vehicles (paths and entry times) are exported to a compiled
   pipe-stream scenario, so **both engines simulate exactly the same
   vehicles**. Each link is one pipe (no `pipes`, `moves` or `friction`),
   which is pipe-stream's single-stream mode.
3. **Stream Python**: `main_simulation_meso` is timed in-process
   (`time.perf_counter`), median of `--py-repeats` runs. Validation,
   assignment and initialisation are not timed.
4. **pipe-stream**: the `bench_runner` example times `Simulation::new` plus
   `Simulation::run`, median of `--repeats` runs from a fresh clone of the
   scenario. JSON loading is not timed.
5. Every vehicle's node passage times are compared between the two engines.

The speed-up is `Python time / pipe-stream time`. The *node passages*
column counts the vehicle crossings of a node, the one unit of work both
engines share: pipe-stream's internal event count is not comparable with
Stream Python's.

`--aligned` adds a second, untimed Stream Python run with pipe-stream's two
node rules patched in, to show which differences come from them (see
[Agreement](RESULTS.md#reading-the-agreement-tables)):

- **deterministic merge**: at a congested merge, the lowest incoming link
  passes. Stream Python picks at random, weighted by the Daganzo
  coefficients;
- **causal passages**: a vehicle cannot cross a node before the previous
  vehicle of the same incoming link.

## Scenarios

| Name | Network | Demand | What it tests |
|---|---|---|---|
| `bottleneck` | 2-lane → 1-lane, 2 × 1 km | 2 500 veh/h, 1 h | queue build-up and spillback |
| `diverge` | 2-lane → two 1-lane branches | 2 × 800 veh/h | route split in free flow |
| `merge` | two 1-lane links → 1 lane | 2 × 1 000 veh/h | congested merge |
| `real_network` | 12-node motorway interchange (`stream-python/example/inputs.npy`) | 3 periods, 2 classes | realistic network with congested diverges and merges |
| `chain_10/50/100` | 10, 50 or 100 links of 100 m | 1 500 veh/h, 1 h | scaling with network size, free flow |
| `grid_3x3/5x5/8x8` | bidirectional Manhattan grid, 200 m links | two crossing diagonal flows of 1 500 veh/h | scaling with network size, crossing flows |

The scenario builders are in `compare_with_stream_python.py`, as plain
Python functions: add one to `SCENARIOS` to benchmark a new case.

## Running it

Requirements:

- a stream-python checkout, by default next to this repository
  (`../stream-python`);
- Python ≥ 3.9 with **NumPy < 2** and SciPy < 1.14. Stream Python does not run on
  NumPy 2: `int()` on a one-element array, in
  `stream/initialization/routes.py`, is an error there.

```bash
python3 -m venv .venv-bench
.venv-bench/bin/pip install "numpy<2" "scipy<1.14" pandas

cargo build --release --no-default-features --example bench_runner
.venv-bench/bin/python bench/compare_with_stream_python.py \
    --stream-python ../stream-python --repeats 7 --py-repeats 3 --aligned
.venv-bench/bin/python bench/make_report.py   # Markdown tables to stdout
```

Options: `--scenarios bottleneck grid_5x5 …` for a subset, `--seed` for the
Python random draws (merges), `--out` for the output directory. The driver
writes `bench/results/results.json`; the intermediate scenario and trajectory
files next to it are git-ignored.

Full run: about 5 minutes, almost all of it in Stream Python.

## SUMO

`compare_with_sumo.py` runs the same scenarios and the same assigned
vehicles through Eclipse SUMO, microscopic and mesoscopic (`--mesosim`),
and compares speed and aggregate outputs with pipe-stream. The network goes
through `netconvert` as plain nodes and edges, with one route per vehicle.
The vehicle type is calibrated on Stream's fundamental diagram; the
settings are in [SUMO.md](SUMO.md#calibration).

```bash
.venv-bench/bin/pip install eclipse-sumo      # sumo + netconvert
.venv-bench/bin/python bench/compare_with_sumo.py --stream-python ../stream-python --repeats 5
.venv-bench/bin/python bench/make_sumo_report.py
```

Stream Python is still needed, to build the scenarios and assign the
demand. `sumo` and `netconvert` are looked up in `$SUMO_HOME/bin`, then in
the `eclipse-sumo` pip package, then on `PATH`. Tested with SUMO 1.27.1.
Full run: about 2 minutes.

## Blocked off-ramp

`diverge_blocked.py` runs one diverge scenario — a three-lane approach, a
one-lane off-ramp blocked by a gate, 9 % exit-bound demand — through Stream
Python, pipe-stream with one pipe per link, pipe-stream with the approach
partitioned, and both SUMO modes, and reports what each does to the through
traffic beside the queue. It also sweeps the friction coefficient φ.
Results and discussion: [DIVERGE.md](DIVERGE.md).

```bash
.venv-bench/bin/python bench/diverge_blocked.py --stream-python ../stream-python
```
