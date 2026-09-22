# pipe-stream

**An event-based mesoscopic LWR traffic model with per-lane resolution,
class-restricted lanes and built-in realism verification.**

Written in Rust. Deterministic, event-exact, and it checks its own results.

---

## Why pipes?

First-order network traffic models force an unhappy choice at motorway exits:

- under **strict FIFO**, a queue on a one-lane off-ramp freezes every lane of
  the mainline;
- under **relaxed FIFO**, the same queue obstructs nobody.

Field observations sit between the two. pipe-stream removes the choice
structurally: every link is partitioned into **pipes**, parallel lane groups
down to single lanes, each solved as an independent kinematic wave stream
with its own capacity, storage and backward wave.

- **FIFO is strict inside a pipe**, where lane commitment makes it physically
  exact, and **overtaking is free across pipes**.
- **Class permissions** (bus, HOV, truck lanes…) and **movement
  restrictions** (which pipes reach which downstream link) are per pipe.
- A single **friction coefficient φ** couples a spilled pipe to its
  neighbours. It is the only parameter that needs calibration
  (φ ≈ 0.95 on field data).
- With one pipe per link, the engine **reduces bit for bit to its
  single-stream ancestor**, enforced in the test suite by trajectory
  checksums.

## Highlights

| | |
|---|---|
| **Exact** | Discrete-event solution of LWR with a triangular fundamental diagram (Newell's simplified theory). No time step, no RNG: two runs are bit-identical. |
| **Verified** | Nine closed-form LWR benchmarks (free flow, bottleneck discharge, queue delay, spillback wave, merge, bus-lane immunity, shoulder-exit wave and through flow, friction throttle) pass at machine precision. |
| **Self-auditing** | `stream-cli verify` checks every link, pipe and node of any scenario (free-flow traversal, capacity, storage bound, backward wave, FD adherence, conservation, FIFO, demand served) and reports **pass / warn / fail / not-exercised**. |
| **Validated** | Three instrumented corridors in two countries: 1 280 of 1 312 flow windows below GEH 5. NGSIM trajectories confirm the lane-commitment assumption. |
| **Scenario variants** | *Patches* (lane closures, capacity drops, lane add/shrink, demand scaling, new links) compose a variant from a base network, with time windows. |
| **Fast** | Tens to hundreds of times faster than the Python reference implementation of Stream. |

## Status

Research software accompanying a paper in preparation (see
[Citing](#citing)). The core engine, CLI, verification apparatus and HTTP
server are working and tested. Not yet available:

- **Python bindings**: the PyO3 module is a stub for now.
- `stream-cli run --output`: result export from the CLI (use `verify -o`
  or the HTTP server to get results).
- Time-dependent assignment, traffic signals and stochastic merges (see
  [Limitations](#limitations)).

APIs and file formats may change before 1.0.

## Getting started

Requires a stable Rust toolchain (edition 2021).

```bash
git clone https://github.com/AureClai/stream-pipes-rust.git
cd stream-pipes-rust

# Build the CLI. --no-default-features skips the (stub) Python extension.
cargo build --release --no-default-features --bin stream-cli
```

Run the nine analytic benchmarks and verify a bundled scenario:

```bash
cargo run --release --no-default-features --bin stream-cli -- \
    verify -i scenarios/diverge.json --assignment --benchmarks -o report.json
```

The exit code is non-zero if any check or benchmark fails, so `verify` can
gate CI pipelines. To see a failure report, try `scenarios/bottleneck.json`:
it is overloaded on purpose, and `verify` flags the entry whose demand is
never fully served before the end of the simulation.

### CLI

| Command | Purpose |
|---|---|
| `stream-cli build --network N.geojson --demand D.json --config C.json -o scenario.json` | Compile source files into one validated scenario |
| `stream-cli run -i scenario.json [--assignment]` | Simulate (with `--assignment`, generate and route vehicles from demand first) |
| `stream-cli verify -i scenario.json [--assignment] [--bin-size 300] [--benchmarks] [-o report.json]` | Simulate, then run the verification apparatus |

### Using it as a library

```rust
use stream_core_rust::{
    assignment::assign_demand, io::compile_scenario, simulation::Simulation,
    validation::Validate, verification::verify_scenario,
};

let mut scenario = compile_scenario(
    "scenarios/diverge/network.geojson",
    "scenarios/diverge/demand.json",
    "scenarios/diverge/config.json",
)?;
scenario.validate()?;
assign_demand(&mut scenario)?;

let mut sim = Simulation::new(scenario);
sim.run()?;

// Results live in the scenario: each vehicle's node_times and pipes_taken.
let report = verify_scenario(&sim.scenario, 300.0);
```

The library crate is named `stream_core_rust`, after the engine it was forked
from. Analysis modules are pure functions over a finished scenario:
`analysis` (binned flow, density and speed per link and per pipe),
`xt_analysis` (space-time diagrams), `diagnostics`, `verification`, and
`reference` (GEH against observed counts).

## Scenario format

A scenario is three source files, plus optional patches:

```
scenarios/<project>/
  network.geojson     # nodes (Points) and links (LineStrings)
  demand.json         # OD flows per period, optionally per vehicle class
  config.json         # start_time, duration, classes
  observed.json       # optional: field counts for GEH comparison
  patches/*.json      # optional: variants (closures, capacity changes, …)
```

**Link properties:** `id`, `node_up`, `node_down`, `length`, `speed`,
`lanes`, `capacity`, and the triangular fundamental diagram `fd_u`, `fd_w`,
`fd_kx`, `fd_c` (SI units; `fd_kx` and `fd_c` are per lane). Lanes are
resolved with three optional properties:

```jsonc
{
  "pipes":    [ { "lanes": 1 }, { "lanes": 2 }, { "lanes": 1, "classes": ["bus"] } ],
  "moves":    { "12": [0], "13": [1, 2] },   // downstream link id → allowed pipes
  "friction": 0.95                            // φ ∈ (0, 1]; 1 = no coupling
}
```

Pipe 0 is the rightmost. A link without `pipes` is a single all-lane stream,
exactly as in the single-stream model. The full model is described in the
paper (see [Citing](#citing)).

### Bundled scenarios

| Scenario | What it shows |
|---|---|
| `bottleneck`, `diverge`, `grid`, `real_world` | Regression fixtures (golden checksums in `tests/scenarios.rs`) |
| `m1_diverge`, `m60_stockport` | UK motorway corridors with WebTRIS detector data |
| `a47_givors` | French urban motorway: single stream vs pipes, φ sweep |
| `ngsim/` | Scripts to fetch NGSIM trajectories and measure lane commitment |
| `ndw/` | Per-lane validation on Dutch NDW data (in preparation) |

## Reproducing the paper

The [`examples/`](examples) directory regenerates the paper's worked
examples; each one writes CSV to stdout:

```bash
cargo run --release --no-default-features --example diverge_entrapment > entrapment.csv
cargo run --release --no-default-features --example fig4_friction_trace > fig4_trace.csv
cargo run --release --no-default-features --example segmentation_sensitivity
cargo run --release --no-default-features --example ngsim_us101 > us101_sim_trace.csv  # needs scenarios/ngsim data
```

## HTTP server

An optional [axum](https://github.com/tokio-rs/axum) server exposes projects,
runs, patches, space-time analysis and verification as JSON endpoints:

```bash
cargo run --release --no-default-features --features server --bin stream-server
# listens on 127.0.0.1:8080 and serves projects from ./scenarios
```

Main endpoints: `POST /run`, `POST /xt-analysis`, `POST /verification`,
`POST /verification/benchmarks`, `POST /verification/reference`, plus CRUD on
`/projects/:name/{scenarios,patches,observed}`. The server is meant for local
use; it has no authentication.

## Tests

```bash
cargo test --no-default-features
```

The suite includes the analytic benchmarks, the physics of dynamic patches,
the friction coupling, and the **back-compatibility gate**: single-pipe
scenarios must reproduce the single-stream engine's event counts and
FNV-1a checksums of every vehicle's node times, bit for bit.

## Limitations

These are deliberate modelling choices for now, not bugs:

- no lane changes in the middle of a link: a vehicle commits to a pipe on entry;
- static shortest-path assignment, blind to vehicle class, with no mid-run
  rerouting;
- deterministic node service order (no stochastic merge yet), no signals;
- myopic lane choice, and friction switches on or off rather than varying
  continuously.

## Citing

If you use pipe-stream, please cite the paper (preprint forthcoming):

> A. Clairais, *A Pipe-Stream Event-Based Mesoscopic LWR Model with Per-Lane
> Resolution, Class-Restricted Lanes and Built-In Realism Verification*,
> 2026.

and, for the underlying single-stream model,
[Stream](https://gitlab.cerema.fr/centre-est/mobilite/Stream/stream-python)
(Cerema Centre-Est; A. Clairais, E. Hans, A. Duret).

## Data

Datasets keep their own licences:

- **National Highways WebTRIS** (M1, M60): Open Government Licence v3.0.
  *Contains National Highways data © National Highways.*
- **NGSIM** (US DOT): public open data, fetched by script, not
  redistributed.
- **Cerema AVATAR** (A47): open data from Cerema.
- **NDW** (Dutch per-lane data): subject to NDW's terms, fetched by
  script, not redistributed.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE)), or
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.

pipe-stream derives from
**[Stream](https://gitlab.cerema.fr/centre-est/mobilite/Stream/stream-python)**
(Cerema Centre-Est), distributed under the permissive CeCILL-B license. As CeCILL-B requires, the attribution to the Stream authors
is kept in [NOTICE](NOTICE) and must be kept in redistributions.

Unless you explicitly state otherwise, any contribution you intentionally
submit for inclusion in this work, as defined in the Apache-2.0 license,
is dual licensed as above, without any additional terms or conditions.
