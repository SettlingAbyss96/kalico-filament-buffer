# Design

kalico-filament-buffer turns a three-position filament buffer into a feeder that Kalico drives in
step with the extruder. This document covers the constraints, the control law, fault handling, testing,
and what is still planned. Hardware specifics for the Mellow Fly LLL Buffer Plus are in
[mellow-buffer-plus.md](mellow-buffer-plus.md).

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
4. **Buttons only when idle.** Needing to nudge the buffer mid-print would mean the control logic failed.
5. **Fail safe.** The buffer is a critical MCU: if it disconnects, Klipper shuts down instead of
   printing for hours with an unfed filament path.

## The buffer, as the controller sees it

The filament runs spool → inlet switch → buffer gear → spring-loaded slider → PTFE tube → extruder.
When the buffer feeds more than the extruder consumes, the slack pushes the slider toward pos3. When
it feeds less, the slider falls back toward pos1, which is also where it rests with no filament.
Three Hall sensors report the slider's position at pos1, pos2 and pos3; they are points along the
travel, not ranges, so between them the controller only knows which sensor it passed last.

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
the same technique Kalico's built-in Belay module uses. Sync, unsync and independent moves are refused
while printing, and `BUFFER_SYNC` additionally requires an empty lookahead queue, so it can't add a
stop even if called at the wrong moment. `tests/test_adapter.py` checks this against stand-in Kalico
objects: during a simulated print the plugin makes no motion-queue calls at all.

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

`filament_buffer.py` holds two classes:

- **`FeedController`**: pure logic with no Klipper imports. It tracks the slider zone, chooses the feed
  multiplier, learns the trim, detects faults and keeps statistics. The offline simulation drives
  it directly.
- **`FilamentBuffer`**: the Kalico adapter. It owns the sensor and button pins, applies the rate,
  syncs and unsyncs, makes independent moves, pauses on faults, drives the LEDs and provides the
  `BUFFER_*` commands. Every callback is wrapped so an error is logged instead of crashing Klipper.

The plugin lives in `klippy/plugins/` (symlinked by `install.sh`), which Kalico loads like a built-in
module and its git tree ignores. `BUFFER_SYNC` belongs in `PRINT_START` after homing and leveling,
where the queue is already empty; `BUFFER_UNSYNC` after an `M400` in `PRINT_END`.

## Feed control

The buffer follows the extruder's motion, retractions included, at
`base_rotation_distance / (trim × m)`:

| Slider zone | `m` |
|---|---|
| at pos1 | 1.50: strong catch-up |
| below pos2, approaching (came up from pos1) | 1.15 |
| below pos2, hovering (slipped out of pos2) | 1.02 |
| **at pos2** | **0.99** |
| above pos2, hovering / approaching | 0.98 / 0.85 |
| at pos3 | 0.30: strong relief, and little pushed into a possible jam |

**Hover.** Inside pos2 the slider drifts down very slowly (the 1% bias); when it slips just below,
1.02 brings it back. Every filament settles at the bottom edge of pos2, so the push stays nearly
constant and the slider never creeps toward pos3. In simulation that costs 0.25 to 3.5 gentle rate
changes per 100 mm of filament.

**Which side did it leave pos2?** The multiplier inside pos2 is never exactly 1, so with a tuned trim
the drift is downward while extruding (upward while retracting). The controller also keeps a belief
that is updated from evidence: a hard pos3 hit means "drifts up", and a long stay inside pos2 proves
any upward drift is negligible. A belief proven wrong by a hard sensor reverts the trim nudge it caused.

**Auto-trim.** One clean hover cycle (pos2, out, back into pos2) reveals the remaining ratio error from
the share of extrusion spent inside pos2: `e = f2 × (δ + ε) − δ` at the bottom edge, with
`δ = 0.02` and `ε = 0.01`. The trim folds it in with gain 0.5, bounded to ±5%. Hard-sensor hits, and
hover legs that last far longer than expected, nudge it by 1%. Any remaining error inside (−δ, +ε)
is stable, and errors near +ε are the best case: the slider barely moves. With calibration (planned),
the feedback should only compensate for the filament's own variation.

**Debounce.** A sensor edge counts only after the extruder has moved 0.3 mm with the new state still
holding, which drops jitter while keeping the zone timing from the original edge.

**Retractions.** The synced buffer follows each retraction at `m`. While hovering (m within ±2% of 1),
a 1 mm retraction moves the slider at most 0.03 mm, and the buffer never pulls against the extruder.
The one bounded exception is a print that begins with a retraction while the spring is fully relaxed:
a single tug of `(1.50 − 1) × 1 mm`.

Rate changes are only issued when `m` changes, at most once every 0.2 s.

## Faults

Distances are real extruded millimeters, so slow moves and long travels can't trigger false faults.
Faults are armed only while synced and printing.

| | Fault | Detection | Action |
|---|---|---|---|
| F1 | Inlet runout | inlet switch clears | PAUSE (a deadline-based pause is planned, see below) |
| F2 | Tangle or feed failure | still at pos1 after 25 mm extruded at ×1.50. A working buffer leaves pos1 within a few mm | PAUSE |
| F3 | Clog or extruder slipping | still at pos3 after 15 mm extruded at ×0.30. The low rate keeps what is pushed into a jam small (simulated: under 12 mm against the end stop) | PAUSE |
| F4 | Impossible sensor state | pos1 and pos3 blocked together | PAUSE |
| F5 | Buffer MCU lost | Klipper's critical-MCU handling | Klipper shutdown |
| F6 | Print started without `BUFFER_SYNC` | extruding while unsynced | F2 catches it |

A fault turns the red LED on, prints the measured numbers, and stays latched while the print is
paused. Resuming re-arms detection from that moment; the fault clears when the print ends.

## Testing

| Layer | What it proves |
|---|---|
| `tests/test_controller.py` (19 tests) | A physical model of the slider drives the real controller: retractions up to 1 mm at 25 to 45 mm/s, flow up to 15 mm/s, ratio errors of ±4%, sensor noise, slow rate application, different sensor geometries, a mid-print filament change, a slipping gear, a clog, soak runs |
| `tests/test_adapter.py` (7 tests) | No motion-queue calls mid-print; commands that would stop the toolhead are refused while printing; config defaults equal the simulated values |
| `config/buffer-test.cfg` | Hardware: sensors, TMC link, LEDs, sync, motor direction |
| `BUFFER_TEST_EXTRUDE` | Synced extrusion into the air at several speeds with 1 mm retractions. Checks the buffer every 25 mm and aborts safely when filament isn't consumed, isn't fed, or runs out |
| Planned | A full test print with the buffer active: `print_stall` stays 0 and print time matches a run without it |

## Planned

**Calibration.** Each measurement runs three times and is rejected if the runs disagree, then is
saved with `SAVE_CONFIG`. Sensor edges are timestamped by the buffer MCU and converted to exact motor
and extruder positions (`get_past_mcu_position`, `find_past_position`), so edges are known to a few
hundredths of a millimeter.

| | Measures | Method |
|---|---|---|
| C1 | Path length, inlet to extruder gears | feed until the first compression edge |
| C2 | Extruder grab point | with the tip at the gears, step the extruder until the slider responds; repeat back and forth |
| C3 | Slider travel between sensor edges | feed with the extruder holding, both directions |
| C4 | True buffer `rotation_distance` | move the extruder across two sensor edges with the buffer holding, then the buffer across the same edges with the extruder holding |
| C5 | Extruder gears to nozzle | from toolhead geometry (60 mm working estimate for a Galileo 2 with a Phaetus Conch); load purges absorb the error |

C3 also lets the controller estimate the slider position between sensors, which removes the one-off
excursions that remain when a very different filament is first loaded.

**Load and unload** (idle only).
- **Materials:** a table of fixed load temperatures per material.
- **`BUFFER_LOAD`:** feed fast to the extruder, slow to the compression edge, let the extruder grab,
  sync, extrude to the nozzle, then purge, retract slightly and clean the nozzle through a hook macro.
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
