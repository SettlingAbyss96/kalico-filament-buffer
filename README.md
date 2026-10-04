# kalico-filament-buffer

A [Kalico](https://github.com/KalicoCrew/kalico) plugin that turns a three-position filament buffer
(built for the **Mellow Fly LLL Buffer Plus**) into a Kalico-controlled feeder synced to the
extruder. The buffer holds its slider at the middle sensor, so it keeps a steady push on the filament
that **assists the extruder** (less load and less heat at the extruder gears). It pauses the print
only for a real runout, tangle or jam.

**Status:** feed control, fault pausing, loading on insert, calibration and the test tooling are
running on a Voron 2.4. Unload, a runout deadline and same-spool continuation are next; see
[docs/DESIGN.md](docs/DESIGN.md#planned).

## Hard rules

1. **The buffer never interrupts printing.** While printing, its only action is changing the buffer
   motor's rotation distance (`stepper.set_rotation_distance()`, no toolhead flush: the technique
   Kalico's built-in Belay module uses). Anything that would stop the toolhead (sync, unsync,
   loading, independent moves) is refused in code while a print runs, and a test proves it
   (`tests/test_adapter.py`).
2. **The only exception is a detected fault: a PAUSE, never a cancel.** Faults are a tangle (still at
   pos1), a jam (still at pos3), an impossible sensor state, or an inlet runout. Distances are measured
   in real extruded millimeters, so slow moves and travels can't cause false faults.
3. **Buttons and loading work only when not printing.**
4. **The buffer is a critical MCU.** If it disconnects, Klipper shuts down instead of printing for
   hours with an unfed filament path.

## How it works

The buffer motor is an `[extruder_stepper]` synced to the extruder. Hall sensors along the slider's
travel give the zone, and the plugin feeds slightly more or less than the extruder:

| Zone | Multiplier | |
|---|---|---|
| pos1 (spring relaxed) | 1.50 | catch up |
| below pos2: approaching / hovering | 1.15 / 1.02 | |
| **pos2 (target)** | **0.99** | drift down very slowly: the slider hovers at the bottom edge of pos2 |
| pos2, coming down from pos3 | 0.85 | relieve until the bottom edge of pos2 |
| pos3 (over-pushed) | 0.30 | relieve, and push little into a possible jam |

On the LLL Plus, pos2 stays blocked from its lower edge up through pos3 (`sensor_layout: overlap`).
Buffers with three separate sensor windows use `sensor_layout: separate`, which adds hover and
approach rates above pos2 (0.98 / 0.85).

An auto-trim learns each filament's true feed ratio, so the feedback ends up only compensating the
filament's own tolerance. Retractions (up to 1 mm and more) are followed exactly: when hovering, a
1 mm retraction moves the slider less than 0.05 mm. Full design and reasoning:
[docs/DESIGN.md](docs/DESIGN.md).

### The control law in brief

The slider stores slack, so its position $x$ follows the difference between what the buffer
delivers and what the extruder takes. With $E$ the extruder position, $m$ the zone multiplier,
$	au$ the learned trim and $r$ the buffer's remaining feed error:

```math
rac{dx}{dE} = g\,m(z) - 1, \qquad g = (1+r)\,	au
```

Everything is per mm of extrusion rather than per second, so pauses and slow moves don't matter.
Around the lower edge of pos2 the multiplier switches between $1-arepsilon$ (inside) and
$1+\delta$ (below), which holds the slider at the edge whenever

```math
rac{1}{1+\delta} < g < rac{1}{1-arepsilon}, \qquad \delta = 0.02,\ arepsilon = 0.01
```

The trim is learned so $g$ stays in that band. From a hover cycle, using the share $f_2$ of
extrusion spent inside pos2, the remaining error is $\hat e = f_2(\delta+arepsilon) - \delta$. From
a rise through the pos2 band of width $w$ over $\Delta E$ of extrusion, it is set in one step to
$	au' = 	au\,(1-arepsilon)/(1 + w/\Delta E)$. Calibration measures the buffer's true feed per
commanded mm as $k = S_e / S_b$, the extruder span over the buffer span between the same sensor
edges.

The derivations, assumptions and where every default comes from are in
[docs/CONTROL.md](docs/CONTROL.md).

## Loading and calibration

- **Loading:** insert filament at the buffer inlet. After a second the buffer feeds it, slowly for
  the first 20 mm so the gear catches it, then at 30 mm/s, until the slider reaches pos2. That
  means the tip is pressed against the extruder gears. pos2 and pos3 double as endstops on the
  buffer MCU, so the motor stops the moment a sensor trips. Press either buffer button to cancel.
  `BUFFER_LOAD` does the same on demand. Then heat and extrude to bring it to the nozzle.
- **Buttons:** hold FEED or RETRACT to move the buffer for as long as you hold it. FEED stops by
  itself at pos3, so it can't push the tube out of its fitting.
- **Calibration:** with filament loaded through the extruder and the hotend hot, `BUFFER_CALIBRATE`
  measures the buffer's true `rotation_distance` against the extruder (a warm-up cycle and three
  runs, about 15 mm extruded each) and applies it until the next restart. It prints the value to put in the config,
  along with the slider's sensor geometry.

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
| `BUFFER_LOAD [SPEED=] [MAX=]` | feed from the inlet until the slider reaches pos2 (also runs by itself when filament is inserted) |
| `BUFFER_CALIBRATE [RUNS=3] [TEMP=]` | measure and apply the buffer's true `rotation_distance` |
| `BUFFER_MOVE DIST= [SPEED=]` | move the buffer on its own; feeding stops early at pos3 |
| `BUFFER_SET ...` | runtime tuning (multipliers, fault distances, trim, `BAND_MM`, `REPORT_EVENTS=0/1`, `ROTATION_DISTANCE`) |
| `BUFFER_TEST_EXTRUDE [TEMP=] [LENGTH=300] [SPEEDS=1.5,3,5] [RETRACT=1.0] [DRY_RUN=1] [CHECK_Z=0] ...` | automated synced-extrusion test: syncs, extrudes in segments with retractions, checks the buffer every 25 mm (aborts safely on anomalies), reports PASS/CHECK with statistics. `CHECK_Z=0` skips the homing check when you know the nozzle is clear of the bed |

While printing, only `BUFFER_STATUS`, `BUFFER_STATS`, `BUFFER_SET` tuning and `BUFFER_SYNC` (with
the toolhead stopped, in `PRINT_START`) are accepted.

`printer.filament_buffer` status: `synced`, `zone`, `multiplier`, `trim`, `applied_multiplier`,
`fault`, `faults_armed`, `base_rotation_distance`, `loading`, `last_load_mm`, and each input
(`pos1`, `pos2`, `pos3`, `inlet`, `key_feed`, `key_retract`).

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
| `sensor_layout` | `overlap` | `overlap` (pos2 stays blocked through pos3, as on the LLL Plus) or `separate` |
| `band_mm` | 4.4 | filament mm from the lower edge of pos2 to pos3 (`BUFFER_CALIBRATE` measures it) |
| `autoload` | True | load to pos2 when filament is inserted into an empty buffer |
| `autoload_delay` | 1 s | wait after the inlet switch closes |
| `load_speed`, `load_max_mm` | 30 mm/s, 1500 mm | loading speed and the longest path it will feed |
| `load_grab_mm`, `load_grab_speed` | 20 mm, 10 mm/s | slow start so the gear catches the filament |
| `button_speed` | 20 mm/s | hold-to-move speed |
| `max_move_speed`, `move_accel` | 60 mm/s, 500 mm/s² | independent moves |
| `m_pos1`, `m_approach_below`, `m_below`, `m_target`, `m_above`, `m_approach_above`, `m_pos3` | 1.50, 1.15, 1.02, 0.99, 0.98, 0.85, 0.30 | zone multipliers |
| `tension_fault_mm`, `compression_fault_mm` | 60, 25 | extruded mm stuck at pos1 / pos3 before PAUSE |
| `trim_limit`, `trim_gain`, `trim_nudge` | 0.05, 0.5, 0.01 | auto-trim bounds, learning gain, nudge size |
| `hover_stall_mm`, `debounce_mm`, `up_stay_flip_mm` | 60, 0.3, 300 | control-law internals (see [DESIGN.md](docs/DESIGN.md#feed-control)) |

Defaults are the values the offline simulation suite validates. A test checks that the config defaults
can't drift from them.

## Testing

- **Offline** (no printer): `python3 -m unittest discover -s tests -v`. There are 48 tests:
  - A physical model of the slider, with the sensor geometry measured on a real LLL Plus, drives
    the real controller: retractions up to 1 mm, flow up to 15 mm/s, ratio errors ±4%, sensor
    noise, slow rate application, other sensor geometries and layouts, mid-print filament changes,
    a slipping gear, a clog, and long soak runs.
  - Adapter tests check the hard rule, loading, buttons and calibration math against stand-in
    Kalico objects.
- **Hardware:** `config/buffer-test.cfg` (`BUFFER_TEST_HELP` lists the steps), ending with
  `BUFFER_TEST_EXTRUDE`.

## Credits

The no-flush rate-change technique comes from [Belay](https://github.com/Annex-Engineering/Belay)
(Annex Engineering), which is built into Kalico. Happy Hare and AFC use the same approach. The stock
buffer firmware is from [FLY3DTeam/Buffer](https://github.com/FLY3DTeam/Buffer).

## License

GPLv3, like Kalico. See [LICENSE](LICENSE).
