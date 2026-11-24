# 7. Roundabouts

A roundabout is modeled as **one `Roundabout` node** in `network.geojson`, rather than a ring of micro-links.

For the full mathematical formulation and implementation notes, see the [roundabout theory & solver documentation](../src/roundabout.rs#L1-L32) and [scenarios/roundabout/README.md](../scenarios/roundabout/README.md).

---

## Theory Summary

1. **Ring as a single closed LWR pipe:**
   The circulating roadway is solved as one FIFO kinematic-wave stream (`lanes` lanes, no overtaking) evaluated at two points per arm $i$: the exit point $X_i$ and (one splitter `island` downstream) the entry point $M_i$.
   - Free-flow travel between consecutive points on the ring.
   - Ring discharge headway $1 / C$ at each point ($C = \text{fd\_c} \times \text{lanes}$).
   - **Newell storage constraint** checked against *every* downstream point $Q$ up to the vehicle's exit $X_b$, so an exit queue spills back around the ring.

2. **Gap acceptance at entries (Siegloch / HCM 6):**
   An approach vehicle enters at $M_i$ when no ring vehicle is waiting at $M_i$, `lead` seconds have elapsed since the last circulating passage, no approaching ring vehicle can reach $M_i$ within `lag` seconds, `follow_up` ($t_f$) has elapsed since the previous entry from the same approach lane, and ring storage is available.
   With $\text{lead} + \text{lag} = t_0 = t_c - t_f / 2$, a circulating gap $g$ admits $\lfloor (g - t_0) / t_f \rfloor + 1$ entering vehicles (Siegloch's formula, with $\text{lead} = \max(t_f / 2, 1 / C)$).

---

## GeoJSON Input (`network.geojson`)

Code a `Point` feature with `"type": "Roundabout"` and a `"roundabout"` object in `properties`:

| Property | Type | Default | Description |
|---|---|---|---|
| `diameter` | `number` (`f64`) | required* | Inscribed circle diameter (m). *Give either `diameter` or `circumference`. |
| `circumference` | `number` (`f64`) | $\pi \times \text{diameter}$ | Ring path length (m); takes precedence over `diameter` if both are given. |
| `lanes` | `integer` (`1..=255`) | `1` | Number of circulating lanes on the ring. |
| `direction` | `"ccw" \| "cw"` | `"ccw"` | Circulation direction (`"ccw"` = counter-clockwise / right-hand traffic, `"cw"` = clockwise). |
| `speed` | `number` (`f64`) | `8.333` (30 km/h) | Circulating free-flow speed (m/s). |
| `fd_u` | `number` (`f64`) | `speed` | Ring FD free-flow speed (m/s). |
| `fd_w` | `number` (`f64`) | `5.0` | Ring FD backward wave speed (m/s). |
| `fd_kx` | `number` (`f64`) | `0.15` | Ring per-lane jam density (veh/m/lane). |
| `fd_c` | `number` (`f64`) | $\frac{k_x \cdot u \cdot w}{u + w}$ | Ring per-lane capacity (veh/s/lane). |
| `critical_gap` | `number` (`f64`) | `4.976` | Critical gap $t_c$ (s) — HCM 6 single-lane default. |
| `follow_up` | `number` (`f64`) | `2.609` | Follow-up time $t_f$ (s) — HCM 6 single-lane default. |
| `island` | `number` (`f64`) | `6.0` | Splitter island distance from an arm's exit point $X_i$ to its entry point $M_i$ (m). |
| `arms` | `Arm[]` | inferred | Optional explicit list of arms `{ "in": LinkID?, "out": LinkID?, "position": f64? }`. When omitted, arms are inferred from link bearings around the node. |

### Example (`network.geojson` feature)

```json
{
  "type": "Feature",
  "geometry": { "type": "Point", "coordinates": [1000, 1000] },
  "properties": {
    "id": 0,
    "type": "Roundabout",
    "roundabout": {
      "diameter": 30.0,
      "lanes": 1,
      "speed": 8.333,
      "fd_kx": 0.15,
      "fd_w": 5.0,
      "critical_gap": 4.976,
      "follow_up": 2.609,
      "island": 6.0
    }
  }
}
```

---

## Limitations

- At most `127` arms per roundabout (`MAX_ARMS = 127`).
- Signalized roundabouts and tolls on links ending at a roundabout are not supported yet.
