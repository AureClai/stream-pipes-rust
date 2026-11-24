# 2. Network & Physics

## Units

All internal and JSON values use SI units unless explicitly noted otherwise:

| Quantity | Unit |
|---|---|
| Distance / link length | `m` |
| Speed (`speed`, `fd_u`, `fd_w`) | `m/s` |
| Time (`start_time`, `duration`, `period_*`) | `s` |
| Capacity (`capacity`, `fd_c`) | `veh/s` |
| Jam density (`fd_kx`) | `veh/m/lane` |
| Demand `flow` | `veh/h` (informational; `count` sets generated vehicles) |
| Output `link_stats` / `pipe_stats` | `flow` in `veh/h`, `density` in `veh/km`, `speed` in `km/h` |

---

## Nodes and Links

A network is a GeoJSON `FeatureCollection` (`network.geojson`):

- **Nodes (`Point` features)** have a unique integer `id` and a `type`:
  - `"Entry"`: origin node where demand enters the network.
  - `"Exit"`: destination node where trips terminate.
  - `"Internal"`: junction / merge / diverge node.
  During `stream-cli build`, nodes are sorted by `id`, and node IDs must be contiguous `0..N-1`.

- **Links (`LineString` features)** connect `node_up` → `node_down` with `length`, `speed`, `lanes`, and `capacity`. Link IDs are also sorted `0..M-1` during `build`.

---

## Fundamental Diagram (Triangular & Trapezoidal)

Each link uses a kinematic-wave Fundamental Diagram (Newell's simplified LWR model) with **4 independent parameters**:

| GeoJSON Property | Scenario Field | Meaning |
|---|---|---|
| `fd_u` | `fd.u` | Free-flow speed (m/s). Defaults to `speed`. |
| `fd_w` | `fd.w` | Backward congestion wave speed (m/s). Defaults to `5.0`. |
| `fd_kx` | `fd.kx` | Per-lane jam density (veh/m/lane). Defaults to `0.2`. |
| `fd_c` | `fd.c` | Per-lane capacity (veh/s/lane). Defaults to `capacity / lanes`. |

- **Triangular FD:** Set $c = \frac{u \cdot w \cdot k_x}{u + w}$.
- **Trapezoidal FD:** Set `fd_c` (and `capacity`) *below* the triangular apex $\frac{u \cdot w \cdot k_x}{u + w}$. The engine does not clamp `c` to the triangular apex, so you can specify an independent capacity plateau (e.g. to represent a bottleneck or capacity drop).

> **Single-pipe vs. multi-pipe capacity:**
> When a link has no `pipes` array (single-pipe mode), the solver uses `Link.capacity` directly. When `pipes` is specified, each pipe's capacity is `fd.c * pipe.lanes`.

---

## Multi-Pipe Links (Lanes, Classes, Movements, Friction)

By default, a link is a single LWR stream (strict FIFO across all lanes). To model lane-group or per-lane behavior, add `pipes` to the link's GeoJSON `properties`:

```json
"pipes": [
  { "lanes": 1, "classes": ["bus"] },
  { "lanes": 2 }
]
```

- **Index order:** Pipe `0` is the rightmost lane group.
- **FIFO & overtaking:** FIFO is strict *within* a pipe; vehicles may overtake *across* pipes.
- **Class permissions:** `classes` lists allowed vehicle class names (resolved against `config.json` `classes`). Omitting `classes` allows all classes.
- **Movement restrictions (`moves`):** Map a downstream `LinkID` (as a string) to the pipe indices that can reach it:
  ```json
  "moves": {
    "5": [0],
    "6": [1]
  }
  ```
  Any downstream link omitted from `moves` is reachable from all pipes.
- **Cross-pipe friction (`friction`):** $\varphi \in (0, 1]$ (default `1.0`). When one pipe on a link spills back, sibling pipes discharge with headway $1 / (C \cdot \varphi)$. Setting `1.0` disables coupling.
