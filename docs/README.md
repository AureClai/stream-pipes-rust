# Documentation

Welcome to the `pipe-stream` (`stream-cli`) user documentation.

## Guide

1. [Quickstart](01-quickstart.md) — Building `stream-cli` and running your first scenario (`build` → `verify` → `run`).
2. [Network & Physics](02-network-and-physics.md) — Nodes, links, units, triangular & trapezoidal Fundamental Diagrams, multi-pipe lanes, movement restrictions, and cross-pipe friction.
3. [Demand & Routing](03-demand-and-routing.md) — OD demand with built-in assignment (`--assignment`) vs. supplying pre-routed vehicles from an external assignment model.
4. [Dynamic Events](04-dynamic-events.md) — Mid-run parameter changes (capacity drops, lane closures, incidents) via `link_schedule` and patches.
5. [Results & Verification](05-results-and-verification.md) — Interpreting `stream-cli run -o` outputs and auditing runs with `stream-cli verify`.
6. [Traffic Signals & Tolls](06-signals.md) — Signal groups, fixed-time plans, ramp metering, and tolls.
7. [Roundabouts](07-roundabouts.md) — Single-node roundabout modeling (LWR ring + Siegloch / HCM 6 gap acceptance).

## Reference

- [JSON Format Reference](JSON_FORMAT.md) — Complete field-by-field schema for all input and output JSON/GeoJSON files.
