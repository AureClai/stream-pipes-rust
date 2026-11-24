# 5. Results & Verification

## Simulation Results (`stream-cli run -o`)

```bash
stream-cli run -i scenario.json --assignment --bin-size 60 -o results.json
```

The output JSON contains:
- **Run metadata:** `events_processed`, `duration_ms`, `vehicles_count`, `status`.
- **`link_stats`:** Binned time series (`t`, `values`) for every link:
  - `flow` (`veh/h`) — exit flow
  - `density` (`veh/km`) — Edie-style link density
  - `speed` (`km/h`) — space-mean speed (`flow / density`)
  - `kx_veh_km`, `capacity_veh_h` — theoretical bounds for reference
- **`pipe_stats`:** Same time series per pipe (included automatically when any link has more than 1 pipe).

See [JSON Format Reference](JSON_FORMAT.md#21-simulation-results-stream-cli-run--o-file) for the full schema.

---

## Verification (`stream-cli verify -o`)

Before trusting a new network or scenario, audit it with `verify`:

```bash
stream-cli verify -i scenario.json --assignment --bin-size 300 -o report.json
```

Each link, pipe, node, and entry is checked against LWR theory and assigned a status of `pass`, `warn`, `fail`, or `not_exercised`:

| Element | Checks |
|---|---|
| **Link / Pipe** | `free_flow_traversal` ($L/u$), `inflow_capacity` ($1/C$), `storage_bound` ($k_x \cdot L \cdot \text{lanes}$), `backward_wave` ($L/w$), `fd_adherence` |
| **Node** | `flow_conservation` (inflow vs. outflow), `fifo_discipline` |
| **Entry** | `demand_served` (fraction of scheduled vehicles that entered the network) |

- Use `--strict` to fail if any element was `not_exercised` (useful for test suites).
- Use `--benchmarks` to also run the 10 closed-form analytic LWR benchmarks.
