# Blocked off-ramp: what happens to the mainline?

The question the pipe partition exists to answer, put to four engines on one
scenario and one set of vehicles. Produced by `bench/diverge_blocked.py`
(method and how to rerun: [README.md](README.md#blocked-off-ramp)).

## Setup

| | |
|---|---|
| Network | 500 m three-lane approach → three-lane through branch + one-lane off-ramp |
| Blockage | the ramp ends at a gate discharging **one vehicle per 100 s** |
| Demand | through 3 600 veh/h, exit-bound 360 veh/h (9 %), over 30 minutes |
| FD (per lane) | u = 25 m/s, w = 5 m/s, kx = 0.127 veh/m, c = 0.55 veh/s |

The gate is the same bottleneck in every engine, expressed in each one's own
terms: a link of capacity 0.01 veh/s in the LWR engines, a traffic light with
2 s of green per 100 s cycle in SUMO — the usual physical cause of an
off-ramp queue. Stream Python assigns the demand once; all engines then run
the same 1 979 vehicles with the same routes and departure times.

## Result

| Engine | Lanes resolved? | Through veh/h | Through delay | Exit served | Exit delay |
|---|---|--:|--:|--:|--:|
| Stream Python (1 stream) | no | **1 444** | 0.0 s\* | 18 | 772 s |
| pipe-stream (1 pipe) | no | **1 444** | 41.8 s\* | 18 | 772 s |
| pipe-stream (pipes, φ = 0.95) | yes | **3 522** | 0.1 s | 18 | 772 s |
| SUMO micro (Krauss, 1 s step) | yes | **3 480** | 10.3 s | 17 | 769 s |
| SUMO meso (queue per segment) | no | **1 586** | 76.9 s | 17 | 766 s |

Demand is 3 600 veh/h through and the ramp serves ~18 vehicles in the half
hour whatever the engine: the bottleneck itself is identical everywhere, so
the entire spread is what each model does to the traffic *beside* the queue.

**The split is lane resolution, not solver family.** The two laneless models
lose 55–60 % of the mainline: Stream Python (event-based, exact) and SUMO
meso (time-stepped, queue-based) agree on entrapment because a single queue
per link has a single head of line. The two lane-resolved models keep it
moving: the pipe partition at 3 522 veh/h and SUMO micro, an independent
microsimulator with explicit lanes and lane changing, at 3 480 veh/h — 1.2 %
apart. Field observation of real diverges (Muñoz & Daganzo 2002, Newell 1999)
describes the second behaviour.

**pipe-stream reproduces its ancestor.** The single-pipe run matches Stream
Python's through throughput exactly (1 444 veh/h), on a scenario neither
engine handles well. The partition is what changes the answer, not the
reimplementation.

\* The two single-stream engines agree on throughput but not on delay,
because they hold blocked vehicles in different places: Stream Python leaves
them in the entry queue (never admitted, so excluded from the average),
pipe-stream admits them onto the link where they queue. Compare throughput
first, as in [SUMO.md](SUMO.md).

## What φ does, and what it does not

Same partition, friction alone varied:

| φ | 1.0 | 0.95 | 0.9 | 0.8 | 0.7 | 0.6 | 0.5 |
|---|--:|--:|--:|--:|--:|--:|--:|
| Through veh/h | 3 526 | 3 522 | 3 512 | 3 450 | 3 372 | 3 294 | 3 214 |
| Through delay (s) | 0.0 | 0.1 | 0.4 | 3.5 | 6.5 | 8.6 | 9.7 |

Calibrated on **throughput**, SUMO micro's 3 480 veh/h sits between φ = 0.8
and 0.9 here, close to the value calibrated on field data (≈ 0.95) and inside
the 8–18 % bracket the lane-changing literature reports. Calibrated on
**delay**, it would take φ ≈ 0.5, which then costs 270 veh/h of throughput
SUMO micro does not lose.

The two targets disagree because they measure different things. φ throttles
the *discharge* of a pipe whose sibling is spilled. SUMO micro's extra 10 s
is mostly time lost *manoeuvring*: slowing to change lane, and accepting gaps
near the gore. A committed-lane model has no such manoeuvre to pay for. So
this scenario supports the mechanism as a throughput term and marks its
limit: φ is not a delay model, and a per-vehicle delay comparison against a
microsimulator should not be used to calibrate it.

## Caveats

- Different traffic models. SUMO micro is car-following with explicit
  junction and lane-change rules; SUMO meso is a queue model per edge
  segment; the two LWR engines are exact event-based solvers. Only aggregate
  outputs are comparable.
- SUMO meso ignores traffic lights unless `--meso-junction-control` is on;
  without it the gate does not block at all (172 exit vehicles served instead
  of 17) and the comparison is void. The driver sets it.
- SUMO's vehicle type is calibrated on the same triangular FD
  (see [SUMO.md](SUMO.md#calibration)); defaults elsewhere. The result does
  not depend on the strategic lane-change eagerness: `lcStrategic` at its
  default gives the same four numbers.
- One run per engine: SUMO micro is deterministic here (`sigma=0`, fixed
  seed), and the LWR engines have no random draws at all.
