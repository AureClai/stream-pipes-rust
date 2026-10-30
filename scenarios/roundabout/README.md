# roundabout

A four-arm urban roundabout coded as **one `Roundabout` node** (node 0).

| | |
|---|---|
| Ring | 30 m diameter (94 m), one lane, 30 km/h, kx = 0.15 veh/m, w = 5 m/s |
| Gap acceptance | t_c = 4.976 s, t_f = 2.609 s (HCM 6, single-lane entry) |
| Arms | E, N, W, S: 300 m approaches and exits, one lane, 50 km/h |
| Demand | one hour, about 600 veh/h per approach, unbalanced turning flows |

Node ids: arm k (E = 0, N = 1, W = 2, S = 3) enters at node 1 + 2k by link
2k and leaves to node 2 + 2k by link 2k + 1. The arms are not listed in the
node: they are found from the link bearings.

```bash
cargo run --release --no-default-features --bin stream-cli -- build \
    --network scenarios/roundabout/network.geojson \
    --demand scenarios/roundabout/demand.json \
    --config scenarios/roundabout/config.json -o roundabout.json
cargo run --release --no-default-features --bin stream-cli -- \
    verify -i roundabout.json --assignment
```

The western approach (380 veh/h straight on, facing the heaviest
circulating flow) queues; its FD adherence check reports a warning, every
other check passes.
