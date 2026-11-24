# urban

A synthetic town that exercises every node mechanism at once. Built by
`examples/build_urban.rs` (do not edit the files by hand):

```bash
cargo run --release --no-default-features --example build_urban
```

```text
           north entry (toll)
  G6 ─────── G7 ─────── G8        G0, G2, G6: priority junctions, yield
  │          │          │         G1: priority junction, stop (avenue major)
  G3 ═══════ G4 ═══════ G5 ══ M ══ east       G7: priority junction, yield
  (round-    (signals,   (signals,  M: on-ramp merge, ramp metered
   about)     3 phases)   2 phases)  G8: ordinary node, no rules
  │          │          │
  G0 ─────── G1 ─────── G2
```

A 3 × 3 grid, 300 m between intersections, local metric coordinates.

| Element | Where | What it exercises |
|---|---|---|
| Roundabout | G3 | ring, gap acceptance, arms inferred from bearings |
| Priority junctions | G0, G1 (stop), G2, G6, G7 | conflicts deduced from geometry, HCM ranks and gaps |
| Signals, 3 phases | G4 | boulevard through and right, protected boulevard lefts, avenue with permitted lefts and sneakers; plans change at 30 min (cycle 80 → 100 s) |
| Signals, 2 phases | G5 | offset 22 s from G4 (green wave), permitted lefts |
| Ramp metering | on-ramp → M | one vehicle per green, cycle 4 s, from 15 to 45 min; dark signal outside |
| Toll | northern entry of the avenue | one server, 6 s per car, 2 s per bus |
| Bus lane | boulevard | a bus-only pipe beside the general lane; buses every 5 min each way |
| Turn pockets | boulevard approaches of G4 | three pipes: bus lane, through/right, left (`moves`) |
| Ordinary node | G8 | no rules: movements cross freely |

Demand: one hour, 420 veh/h each way on the boulevard, 240 veh/h each way
on the avenue, 380 veh/h on the on-ramp, 8 veh/h between every other pair
of entries and exits (all movements of all nodes get traffic), and buses.

```bash
cargo run --release --no-default-features --bin stream-cli -- build \
    --network scenarios/urban/network.geojson --demand scenarios/urban/demand.json \
    --config scenarios/urban/config.json --signals scenarios/urban/signals.json -o urban.json
cargo run --release --no-default-features --bin stream-cli -- verify -i urban.json --assignment
```

Every check passes. `tests/urban.rs` also asserts that each mechanism is
exercised: ring checks, gap acceptance at every priority junction, every
signal group, the toll, the bus lane, the ramp meter and the plan change.

What the scenario taught the engine, while it was built:

- permitted left turns in a shared lane starved forever behind a dense
  opposing flow and gridlocked the centre: signal groups now let up to
  `sneakers` (2) permitted vehicles clear within `clearance` (3 s) of the
  red, as in the HCM;
- a capped ramp meter whose plan ends turned dark but kept the cap of its
  last green, so the ramp stayed blocked: the dark signal has no cap;
- the verification learned interrupted flow: the fundamental-diagram
  check does not apply to links ending at a stop line, nor to bins with
  fewer than 30 vehicles; node conservation counts node passages.
