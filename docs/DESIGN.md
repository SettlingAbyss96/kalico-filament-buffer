# Design

kalico-filament-buffer turns a three-position filament buffer into a feeder that Kalico drives in
step with the extruder. This document covers the constraints, the control law, fault handling,
loading and calibration, testing, and what is still planned. Hardware specifics for the Mellow Fly
LLL Buffer Plus are in [mellow-buffer-plus.md](mellow-buffer-plus.md). The equations behind the
control law, the learning, the fault distances and calibration, with their assumptions, are in
[CONTROL.md](CONTROL.md).

## Goals

1. **Assist the extruder.** Keep the buffer's slider at its middle sensor (pos2). There the spring is
   partly compressed, so the buffer pushes the filament toward the extruder with a steady, moderate
   force. The extruder then does less work and its gears run cooler, which matters in a hot chamber
   where warm gears soften the filament. At pos1 the spring is relaxed and the extruder pulls the whole
   path on its own; at pos3 the buffer pushes too hard.
2. **Never interrupt a print.** Rate changes, sensor handling and everything else must leave the
   toolhead's motion untouched.
3. **Pause, never cancel, on a real fault**: inlet runout, a tangle upstream, or a jam downstream.
   These are the only cases where the buffer may stop a print.
4. **Buttons and loading only when idle.** Needing to nudge the buffer mid-print would mean the
   control logic failed.
5. **Fail safe.** The buffer is a critical MCU: if it disconnects, Klipper shuts down instead of
   printing for hours with an unfed filament path.

## The buffer, as the controller sees it

The filament runs spool → inlet switch → buffer gear → spring-loaded slider → PTFE tube → extruder.
When the buffer feeds more than the extruder consumes, the slack pushes the slider toward pos3. When
it feeds less, the slider falls back toward pos1, which is also where it rests with no filament.

Three Hall sensors see a magnet on the slider. Measured on an LLL Plus, in filament mm from the
slider's rest position:

| Slider position | pos1 | pos2 | pos3 |
|---|---|---|---|
| 0 to about 19 mm (rest end) | blocked | | |
| 19 to 28.6 mm (gap) | | | |
| 28.6 to 33.0 mm (the pos2 band) | | blocked | |
| 33.0 mm to near the end stop | | blocked | blocked |
| at the very end of travel | | | blocked |

So on this buffer pos2 is not a point: it stays blocked from its lower edge up through pos3 (the
"overlap" layout). Clearing pos2 while pos3 is clear can only mean the slider went down, and the
target is the lower edge of pos2, 4.4 mm below pos3. The plugin also supports buffers whose three
sensors are separate windows (`sensor_layout: separate`); there, between two sensors the
controller only knows which one it passed last, and it infers the exit side of pos2.

## Why a plugin: the motion-queue constraint

In Kalico (as in Klipper), the obvious G-code tools stop the toolhead when used mid-print:

| Action | Touches the motion queue? | Usable while printing? |
|---|---|---|
| `SET_EXTRUDER_ROTATION_DISTANCE`, `SYNC_EXTRUDER_MOTION` | Yes: `flush_step_generation()`. The planner then assumes "a complete stop" | No |
| `MANUAL_STEPPER`, `FORCE_MOVE`, `SET_TMC_FIELD`, `SET_TMC_CURRENT` | Yes: `get_last_move_time()` flushes the lookahead | No |
| `stepper.set_rotation_distance()` called from Python | No. It saves the step position, changes the step size and restores the position. It takes effect within the next step-generation window (about 0.1 to 0.3 s) | **Yes** |
| Reading sensors (`buttons.register_buttons`) and the extruder's past position | No | Yes |
| `PAUSE` (run from a reactor callback, as Kalico's runout sensor does) | Stops the print by design | Faults only |

So while a print runs, the plugin's only action is `set_rotation_distance()` on the buffer motor,
the same technique Kalico's built-in Belay module uses. Sync, unsync, loading and independent moves
are refused while printing, and `BUFFER_SYNC` additionally requires an empty lookahead queue, so it
can't add a stop even if called at the wrong moment. `tests/test_adapter.py` checks this against
stand-in Kalico objects: during a simulated print the plugin makes no motion-queue calls at all.

## Prior art

| Project | Relevance | Why it isn't used directly |
|---|---|---|
| [Belay](https://github.com/Annex-Engineering/Belay) (built into Kalico) | Syncs a secondary feeder and switches a rate multiplier without flushing | Reads one binary sensor; no fault handling, calibration or loading |
| [Happy Hare](https://github.com/moggieuk/Happy-Hare) | Sync feedback, bowden calibration, clog detection | A full MMU framework; does not guarantee Kalico support |
| [AFC](https://github.com/AFCProject/AFC-Klipper-Add-On) | Buffer multiplier, tested on Kalico | Built around specific multi-lane units |
| Community LLL Plus configs | Pin maps and first conversion notes | Use stock G-code commands that stop the toolhead, or keep the stock firmware |

## Architecture

```
[extruder_stepper buffer]        buffer motor; starts unsynced (empty extruder:)
[tmc2208 extruder_stepper buffer]
[filament_buffer]                this plugin
```

`filament_buffer.py` holds three parts:

- **`FeedController`**: pure logic with no Klipper imports. It tracks the slider zone, chooses the feed
  multiplier, learns the trim, detects faults and keeps statistics. The offline simulation drives
  it directly.
- **`BufferMover`**: independent moves of the buffer motor while it is unsynced. It gives the motor
  the same toolhead-like interface `manual_stepper` uses for homing, so a move can end at a sensor
  (see [Loading, buttons and calibration](#loading-buttons-and-calibration)).
- **`FilamentBuffer`**: the Kalico adapter. It owns the sensor and button pins, applies the rate,
  syncs and unsyncs, pauses on faults, drives the LEDs, loads and calibrates, and provides the
  `BUFFER_*` commands. Every callback is wrapped so an error is logged instead of crashing Klipper.

The plugin lives in `klippy/plugins/` (symlinked by `install.sh`), which Kalico loads like a built-in
module and its git tree ignores. `BUFFER_SYNC` belongs in `PRINT_START` after homing and leveling,
where the queue is already empty; `BUFFER_UNSYNC` after an `M400` in `PRINT_END`.

## Feed control

The buffer follows the extruder's motion, retractions included, at
`base_rotation_distance / (trim × m)`. The slider then moves as

```math
\frac{dx}{dE} = g\,m - 1, \qquad g = (1 + r)\,\tau
```

per mm of extrusion, where $r$ is the buffer's remaining feed error and $\tau$ the trim
([CONTROL.md, section 2](CONTROL.md#2-plant-model)):

| Slider zone | `m` |
|---|---|
| at pos1 | 1.50: strong catch-up |
| below pos2, approaching (came up from pos1) | 1.15 |
| below pos2, hovering (slipped out of pos2) | 1.02 |
| **at pos2** | **0.99** |
| at pos2, coming down from pos3 (overlap layout) | 0.85, until the lower edge |
| above pos2, hovering / approaching (separate layout) | 0.98 / 0.85 |
| at pos3 | 0.30: strong relief, and little pushed into a possible jam |

**Hover.** Inside pos2 the slider drifts down very slowly (the 1% bias); when it slips just below,
1.02 brings it back. This holds whenever $1/1.02 < g < 1/0.99$
([CONTROL.md, section 4](CONTROL.md#4-hovering-at-the-lower-edge-of-pos2)). Every filament settles at the bottom edge of pos2, so the push stays nearly
constant and the slider never creeps toward pos3. In simulation that costs 0.25 to 3.5 gentle rate
changes per 100 mm of filament.

**Which side did it leave pos2?** With the overlap layout the answer is always "below". With
separate sensors the multiplier inside pos2 is never exactly 1, so with a tuned trim the drift is
downward while extruding (upward while retracting). The controller also keeps a belief that is
updated from evidence: a hard pos3 hit means "drifts up", and a long stay inside pos2 proves any
upward drift is negligible. A belief proven wrong by a hard sensor reverts the trim nudge it caused.

**Auto-trim.** The trim folds in measurements of the remaining ratio error, bounded to ±5%:
- *Hover cycles.* One clean cycle (pos2, out, back into pos2) gives the error from the share of
  extrusion spent inside pos2: `e = f2 × (δ + ε) − δ` at the bottom edge, with `δ = 0.02` and
  `ε = 0.01`, folded in with gain 0.5.
- *Rising through the band* (overlap layout). If the slider enters pos2 at its lower edge and still
  rises all the way to pos3 at 0.99, the band width over the extrusion it took is the excess feed,
  so the trim is set to zero error in one step. A low overshoot is harmless (the slider then hovers
  at the lower edge and the cycles refine it); a rise that is implausibly fast counts as a
  disturbance instead.
- *Hard hits.* Reaching pos1 or pos3 from the hover rates, and hover legs that last far longer
  than expected, nudge it by 1%.

The derivations are in [CONTROL.md, section 5](CONTROL.md#5-learning-the-trim). Any
remaining error inside (−δ, +ε) is stable, and errors near +ε are the best case: the slider
barely moves. With a calibrated `rotation_distance` the feedback only compensates for the
filament's own variation.

**Debounce.** A sensor edge counts only after the extruder has moved 0.3 mm with the new state still
holding, which drops jitter while keeping the zone timing from the original edge.

**Retractions.** The synced buffer follows each retraction at `m`. While hovering (m within ±2% of 1),
a 1 mm retraction moves the slider less than 0.05 mm, and the buffer never pulls against the
extruder. The one bounded exception is a print that begins with a retraction while the spring is
fully relaxed: a single tug of `(1.50 − 1) × 1 mm`.

Rate changes are only issued when `m` changes, at most once every 0.2 s.

## Faults

Distances are real extruded millimeters, so slow moves and long travels can't trigger false faults.
Faults are armed only while synced and printing. How the distances follow from the slider geometry:
[CONTROL.md, section 8](CONTROL.md#8-faults).

| | Fault | Detection | Action |
|---|---|---|---|
| F1 | Inlet runout | inlet switch clears | PAUSE (a deadline-based pause is planned, see below) |
| F2 | Tangle or feed failure | still at pos1 after 60 mm extruded at ×1.50. From the rest end, a working buffer needs about 38 mm to leave the 19 mm deep pos1 zone | PAUSE |
| F3 | Clog or extruder slipping | still at pos3 after 25 mm extruded at ×0.30. From the end stop a working buffer leaves pos3 within about 10 mm, and the low rate keeps what is pushed into a jam small | PAUSE |
| F4 | Impossible sensor state | pos1 and pos3 blocked together | PAUSE |
| F5 | Buffer MCU lost | Klipper's critical-MCU handling | Klipper shutdown |
| F6 | Print started without `BUFFER_SYNC` | extruding while unsynced | F2 catches it |

A fault turns the red LED on, prints the measured numbers, and stays latched while the print is
paused. Resuming re-arms detection from that moment; the fault clears when the print ends.

## Loading, buttons and calibration

All of these run only when not printing. They move the buffer motor on its own, unsynced.

**Stopping at a sensor.** pos2 and pos3 are registered twice: as button inputs (zone tracking)
and as endstops on the buffer motor. A move that should end at a sensor runs as a homing move, so
the buffer MCU stops the motor itself the moment the sensor trips; there is no host latency in
the stop. A move can also end on a host condition, such as a released button, checked about every
0.1 s by the same drip mechanism homing uses; the motor then stops within about 0.15 s of motion.

**Loading** (`BUFFER_LOAD`, or by itself about a second after filament is inserted into an empty
buffer, i.e. the slider resting at pos1). It feeds 20 mm at 10 mm/s so the gear catches filament
that is still being pushed in, then up to `load_max_mm` at 30 mm/s, and stops at pos2: the tip has
reached the extruder gears and the slider is compressed to its target. Either button cancels. If
pos2 is never reached, the gear probably never gripped the filament.

**Buttons.** Holding FEED or RETRACT runs one continuous move that ends when the button is released.
FEED also ends at pos3. That way, holding the button too long can't push the filament against a
fully compressed slider hard enough to pop the PTFE tube out of its fitting.

**Calibration** (`BUFFER_CALIBRATE`, filament through the extruder, hotend hot). Each run:
1. Bring the slider to the top of the pos1 zone.
2. Feed with the buffer, extruder holding, until pos3. Record the buffer position at each sensor
   edge.
3. Extrude, buffer holding, until pos1. Record the extruder position at each sensor edge.

Edge times come from the sensor reports and are converted to motor and extruder positions
(`get_past_mcu_position`, `find_past_position`), so the stopping points don't matter. The span
from leaving pos1 to reaching pos3 is the same in both directions as long as pos1 and pos3 have
the same hysteresis (on the LLL Plus both are under 0.1 mm), so extruder span over buffer span is
the buffer's true feed per commanded mm ([CONTROL.md, section 9](CONTROL.md#9-calibration)). A first, unmeasured cycle takes up slack left in the tube while the motor was off. Then
three runs must agree within 4% (on the LLL Plus they scatter by 1 to 3% from elastic slack),
and the median is used. The new `rotation_distance` is applied until the next restart and
printed for the config, with the gap and band widths.

## Testing

| Layer | What it proves |
|---|---|
| `tests/test_controller.py` (26 tests) | A physical model of the slider, with the measured LLL Plus geometry, drives the real controller: retractions up to 1 mm at 25 to 45 mm/s, flow up to 15 mm/s, ratio errors of ±4%, sensor noise, slow rate application, other geometries in both layouts, a mid-print filament change, a slipping gear, a clog, soak runs |
| `tests/test_adapter.py` (22 tests) | No motion-queue calls or buffer moves mid-print; commands that would stop the toolhead are refused while printing; loading, buttons and autoload behave; calibration math; config defaults equal the simulated values |
| `config/buffer-test.cfg` | Hardware: sensors, TMC link, LEDs, sync, motor direction |
| `BUFFER_TEST_EXTRUDE` | Synced extrusion into the air at several speeds with 1 mm retractions. Checks the buffer every 25 mm and aborts safely when filament isn't consumed, isn't fed, or runs out |
| Planned | A full test print with the buffer active: `print_stall` stays 0 and print time matches a run without it |

## Planned

**More calibration.** Each measurement runs three times and is rejected if the runs disagree:

| | Measures | Method |
|---|---|---|
| C1 | Path length, inlet to extruder gears | `BUFFER_LOAD` already reports the fed distance; store it for faster loads and the runout deadline |
| C2 | Extruder grab point | with the tip at the gears, step the extruder until the slider responds; repeat back and forth |
| C3 | Slider position between sensors | from the measured geometry, to remove the one-off excursions when a very different filament is first loaded |
| C5 | Extruder gears to nozzle | from toolhead geometry (60 mm working estimate for a Galileo 2 with a Phaetus Conch); load purges absorb the error |

**Finishing the load and the unload** (idle only).
- **Materials:** a table of fixed load temperatures per material.
- **Load to the nozzle:** after `BUFFER_LOAD`, heat, let the extruder grab, sync, extrude to the
  nozzle, then purge, retract slightly and clean the nozzle through a hook macro.
- **`BUFFER_UNLOAD`:** clear the extruder, then retract until the tip sits just past the buffer gear,
  still gripped and ready to reload. There is no spool rewinder, so only the minimum is pulled back.

**Runout deadline.** On runout, track the old tail and pause only at the last safe point before it
reaches the extruder gears, instead of immediately.

**Same-spool continuation** (experimental, off by default until proven in real prints). If a new
spool is inserted before the deadline, feed it faster until its tip meets the old tail (detected as
early compression), then continue normally so the new filament pushes the tail through the extruder.
If contact isn't seen in time, it falls back to the deadline pause, never to a stranded tail.

**Print integration.** `PRINT_START`, `PRINT_END`, `PAUSE` and `RESUME` hooks for syncing and for the
fault and continuation flows.
