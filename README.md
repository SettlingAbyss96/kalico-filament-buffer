# kalico-filament-buffer

A [Kalico](https://github.com/KalicoCrew/kalico) plugin that turns a three-position filament buffer
(built for the **Mellow Fly LLL Buffer Plus**) into a Kalico-controlled feeder synced to the
extruder. The buffer holds its slider at the middle sensor, so it keeps a steady push on the filament
that **assists the extruder** (less load and less heat at the extruder gears). It pauses the print
only for a real runout, tangle or jam.

**Status:** feed control, fault pausing and the test tooling are running on a Voron 2.4.
Calibration, load/unload and same-spool continuation are next; see
[docs/DESIGN.md](docs/DESIGN.md#planned).

## Hard rules

1. **The buffer never interrupts printing.** While printing, its only action is changing the buffer
   motor's rotation distance (`stepper.set_rotation_distance()`, no toolhead flush: the technique
   Kalico's built-in Belay module uses). Anything that would stop the toolhead (sync, unsync,
   independent moves) is refused in code while a print runs, and a test proves it (`tests/test_adapter.py`).
2. **The only exception is a detected fault: a PAUSE, never a cancel.** Faults are a tangle (still at
   pos1), a jam (still at pos3), an impossible sensor state, or an inlet runout. Distances are measured
   in real extruded millimeters, so slow moves and travels can't cause false faults.
3. **Buttons work only when not printing.**
4. **The buffer is a critical MCU.** If it disconnects, Klipper shuts down instead of printing for
   hours with an unfed filament path.

## How it works

The buffer motor is an `[extruder_stepper]` synced to the extruder. Point sensors along the slider's
travel give the zone, and the plugin feeds slightly more or less than the extruder:

| Zone | Multiplier | |
|---|---|---|
| pos1 (spring relaxed) | 1.50 | catch up |
| below pos2: approaching / hovering | 1.15 / 1.02 | |
| **pos2 (target)** | **0.99** | drift down very slowly: the slider hovers at the bottom edge of pos2 |
| above pos2: hovering / approaching | 0.98 / 0.85 | |
| pos3 (over-pushed) | 0.30 | relieve, and push little into a possible jam |

An auto-trim learns each filament's true feed ratio from hover cycles, so the feedback ends up only
compensating the filament's own tolerance. Retractions (up to 1 mm and more) are followed exactly:
when hovering, a 1 mm retraction moves the slider ≤ 0.03 mm. Full design and reasoning:
[docs/DESIGN.md](docs/DESIGN.md).

## Requirements

- **Kalico** (uses `klippy/plugins/`; tested on v2026.10.00-58). Mainline Klipper has no plugin directory.
- The buffer running Kalico firmware as its own MCU:
  [docs/mellow-buffer-plus.md](docs/mellow-buffer-plus.md) (backup, Katapult, Kalico, pin map).
- Buffer powered from the printer's 12–24 V supply; USB for data only.

## Install

```bash
cd ~ && git clone https://github.com/SettlingAbyss96/kalico-filament-buffer.git
~/kalico-filament-buffer/install.sh --with-tests
```
Copy [config/mellow-buffer-plus.cfg](config/mellow-buffer-plus.cfg) into your config (set the
buffer's serial), `[include]` it plus `buffer-test.cfg`, and restart Klipper.

> **After updating the plugin code, restart the Klipper service** (`sudo systemctl restart klipper`).
> `RESTART` and `FIRMWARE_RESTART` reload the config inside the same Python process, which keeps
> the previously imported plugin module, so new code would not load. Config-only changes just need
> `RESTART`.

Updates through Moonraker (add to `moonraker.conf`):
```ini
[update_manager kalico-filament-buffer]
type: git_repo
path: ~/kalico-filament-buffer
origin: https://github.com/SettlingAbyss96/kalico-filament-buffer.git
primary_branch: main
managed_services: klipper
```

## Commands

| Command | |
|---|---|
| `BUFFER_STATUS` | zone, multiplier, trim, sensors, rotation distance, fault |
| `BUFFER_STATS [RESET=1]` | extrusion share per zone, zone entries, rate changes, trim learning since reset |
| `BUFFER_SYNC` / `BUFFER_UNSYNC` | sync the buffer to the extruder / release it. SYNC needs the toolhead stopped (call it after G28/QGL or M400 in PRINT_START); UNSYNC isn't allowed while printing |
| `BUFFER_MOVE DIST= [SPEED=] [ACCEL=]` | move the buffer on its own (not while printing) |
| `BUFFER_SET ...` | runtime tuning (multipliers, fault distances, trim, `REPORT_EVENTS=0/1`) |
| `BUFFER_TEST_EXTRUDE [TEMP=] [LENGTH=300] [SPEEDS=1.5,3,5] [RETRACT=1.0] [DRY_RUN=1] ...` | automated synced-extrusion test: syncs, extrudes in segments with retractions, checks the buffer every 25 mm (aborts safely on anomalies), reports PASS/CHECK with statistics |

`printer.filament_buffer` status: `synced`, `zone`, `multiplier`, `trim`, `applied_multiplier`,
`fault`, `faults_armed`, `base_rotation_distance`, and each input (`pos1`, `pos2`, `pos3`, `inlet`,
`key_feed`, `key_retract`).

## Configuration: `[filament_buffer]`

| Option | Default | |
|---|---|---|
| `extruder_stepper` | `buffer` | name of the `[extruder_stepper]` driving the buffer |
| `extruder` | `extruder` | extruder to sync to |
| `pos1_pin`, `pos2_pin`, `pos3_pin` | required | slider sensors, true = blocked |
| `inlet_pin` | required | inlet switch, true = filament present |
| `feed_button_pin`, `retract_button_pin` | none | optional buttons |
| `ok_led`, `fault_led` | none | `[output_pin]` names |
| `report_events` | True | print every sensor/button change |
| `button_step`, `button_speed` | 10 mm, 20 mm/s | hold-to-move chunks |
| `max_move_speed`, `move_accel` | 60 mm/s, 500 mm/s² | independent moves |
| `m_pos1`, `m_approach_below`, `m_below`, `m_target`, `m_above`, `m_approach_above`, `m_pos3` | 1.50, 1.15, 1.02, 0.99, 0.98, 0.85, 0.30 | zone multipliers |
| `tension_fault_mm`, `compression_fault_mm` | 25, 15 | extruded mm stuck at pos1 / pos3 before PAUSE |
| `trim_limit`, `trim_gain`, `trim_nudge` | 0.05, 0.5, 0.01 | auto-trim bounds, learning gain, nudge size |
| `hover_stall_mm`, `debounce_mm`, `up_stay_flip_mm` | 60, 0.3, 300 | control-law internals (see [DESIGN.md](docs/DESIGN.md#feed-control)) |

Defaults are the values the offline simulation suite validates. A test checks that the config defaults
can't drift from them.

## Testing

- **Offline** (no printer): `python3 -m unittest discover -s tests -v`. There are 26 tests:
  - A physical model of the slider drives the real controller: retractions up to 1 mm, flow up to
    15 mm/s, ratio errors ±4%, sensor noise, slow rate application, different sensor geometries,
    mid-print filament changes, a slipping gear, a clog, and long soak runs.
  - Adapter tests check the hard rule against stand-in Kalico objects.
- **Hardware:** `config/buffer-test.cfg` (`BUFFER_TEST_HELP` lists the steps), ending with
  `BUFFER_TEST_EXTRUDE`.

## Credits

The no-flush rate-change technique comes from [Belay](https://github.com/Annex-Engineering/Belay)
(Annex Engineering), which is built into Kalico. Happy Hare and AFC use the same approach. The stock
buffer firmware is from [FLY3DTeam/Buffer](https://github.com/FLY3DTeam/Buffer).

## License

GPLv3, like Kalico. See [LICENSE](LICENSE).
