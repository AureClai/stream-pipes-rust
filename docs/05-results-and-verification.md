# 5. Results & Verification

## Simulation Results (`stream-cli run -o`)

```bash
stream-cli run -i scenario.json --assignment --bin-size 60 -o results.json
# Omit per-vehicle trajectories on large networks:
stream-cli run -i scenario.json --assignment --bin-size 60 --no-vehicles -o results.json
```

The output JSON (`RunOutput`) contains:
- **Run metadata:** `api_version`, `engine_version`, `start_time`, `duration`, `bin_size`, `classes`, `events_processed`, `vehicles_total`, `vehicles_arrived`, `warnings`.
- **`vehicle_kpis`:** Pre-computed summary KPIs (`total`, `entered`, `arrived`, `in_network`, `queued_at_entry`, `completion_rate`, `total_vkt_veh_km`, `total_vht_veh_h`, `total_entry_wait_veh_h`, `total_delay_veh_h`, `mean_travel_time_s`, `p50_travel_time_s`, `p95_travel_time_s`, `mean_free_flow_time_s`, `mean_delay_s`, `mean_entry_wait_s`, `max_entry_wait_s`, `mean_speed_kmh`).
- **`link_stats`:** Binned time series (`t`, `values`) for every link:
  - `flow` (`veh/h`) — exit flow
  - `density` (`veh/km`) — Edie-style link density
  - `speed` (`km/h`) — space-mean speed (`flow / density`)
  - `kx_veh_km`, `capacity_veh_h` — theoretical bounds for reference
- **`link_cvcs`:** Pre-computed entry and exit cumulative volume curves (`t`, `entry`, `exit`) per link sampled at bin boundaries from `start_time` to `start_time + duration`.
- **`pipe_stats`:** Same time series per pipe (included automatically for links with more than 1 pipe).
- **`turning_counts`:** Turning-movement volumes (`node_id`, `from_link`, `to_link`, `count`, `flow_veh_h`, `share`).
- **`queue_summaries`:** Per-link peak occupancy, end occupancy, occupancy ratio, spillback duration, and traversal delay.
- **`od_skims`:** Origin-destination travel-time skims (`mean`, `p50`, `p95`, total time including entry wait, free-flow time, delay).
- **`vehicles`:** Per-vehicle trips and node passage timestamps (empty when `--no-vehicles` is passed).

See [JSON Format Reference](09-json-format.md#21-simulation-results-stream-cli-run--o-file) for the full schema.

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
- Use `--benchmarks` to also run the 18 closed-form analytic LWR benchmarks.
- Use `--observed <FILE>` (or `--observed observed.json` in directory mode `-i <DIR>`) to compare simulated flows and speeds against field observations (`GEH`, `flow_rmse_veh_h`, `flow_mean_geh`, `speed_rmse_kmh`).
