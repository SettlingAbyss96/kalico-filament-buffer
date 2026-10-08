# Kalico plugin: filament buffer control for the Mellow Fly LLL Buffer Plus
# (and similar buffers with three slider position sensors + inlet switch).
#
# Hard rule: the buffer never interrupts printing. While printing, the only
# action taken is stepper.set_rotation_distance() (no toolhead flush - the
# same technique as Kalico's belay module). Anything that flushes the motion
# queue is refused while printing. The only exception is a detected fault,
# which PAUSEs the print.
#
# Design: docs/DESIGN.md - https://github.com/SettlingAbyss96/kalico-filament-buffer
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import importlib
import logging
import math

UPDATE_INTERVAL = 0.1  # seconds between extruder position samples
DIRECTION_WINDOW = 0.3  # seconds of extruder history used for direction
DIRECTION_DEADBAND = 0.05  # mm of net extruder motion treated as "stopped"
MIN_RATE_CHANGE_INTERVAL = 0.2  # seconds between applied rate changes
TEST_POS3_ABORT_MM = 25.0  # BUFFER_TEST_EXTRUDE: held at pos3 this long -> abort
TEST_POS1_ABORT_MM = 70.0  # BUFFER_TEST_EXTRUDE: held at pos1 this long -> abort
JOG_MAX_MM = 2000.0  # longest single independent move (buttons, BUFFER_MOVE)
LOAD_INLET_GRACE = 0.3  # seconds the inlet may read empty before a load stops
SENSOR_SETTLE = 0.15  # seconds for sensor reports to arrive after a move
CALIBRATE_SPREAD = 0.04  # BUFFER_CALIBRATE: runs must agree within 4%
# Loading and unloading work with margins, not exact distances (docs/DESIGN.md)
DEFAULT_NOZZLE_MM = 100.0  # gears to nozzle until BUFFER_UNLOAD measures it
NOZZLE_MARGIN_MM = 15.0  # extra feed past the nozzle estimate on a load
MIN_NOZZLE_MM = 20.0  # a shorter release means the filament wasn't in the hotend
MIN_PATH_MM = 100.0  # a shorter autoload isn't a full path measurement
PATH_AGREE_MM = 50.0  # a new path reading this far off the known one isn't saved
TEST_SPEED_MAX = 150.0  # BUFFER_TEST_SPEED may go past max_move_speed, not past this

# Unloading runs the buffer at a fixed multiplier, not the zone control: the
# slider stays relaxed so nothing presses the tip into the gears (DESIGN.md)
UNLOAD_FOLLOW = 1.02  # a hair faster than the extruder: pulls, never pushes
RELAX_LEAVE_MM = 2.0  # slack left in the slider before an unload

ZONE_POS1 = "pos1"
ZONE_BELOW = "below"
ZONE_POS2 = "pos2"
ZONE_ABOVE = "above"
ZONE_POS3 = "pos3"
ZONE_UNKNOWN = "unknown"


######################################################################
# Pure control logic (no Klipper imports - unit tested offline)
######################################################################


class FeedController:
    """Tracks the slider zone from sensor edges, chooses the feed
    multiplier, learns a trim for the filament's true ratio, and detects
    faults. Positions are extruder positions in mm.

    Zones: pos1 (spring relaxed) < below < pos2 (target) < above < pos3.
    'below'/'above' run in approach mode (strong correction) when entered
    from a hard sensor, and in hover mode (gentle) when entered from pos2.

    overlap=True is the Mellow LLL Buffer Plus layout: pos2 stays blocked
    from its lower edge up through pos3, so there is no 'above' zone and
    pos2 clearing (with pos3 clear) always means the slider went down.
    Coming down from pos3, pos2 runs in approach mode until its lower edge.
    With overlap=False the three sensors are separate windows.
    """

    def __init__(
        self,
        m_pos1=1.50,
        m_approach_below=1.15,
        m_below=1.02,
        m_target=0.99,
        m_above=0.98,
        m_approach_above=0.85,
        m_pos3=0.30,
        tension_fault_mm=60.0,
        compression_fault_mm=25.0,
        trim_limit=0.05,
        trim_gain=0.5,
        trim_nudge=0.01,
        hover_stall_mm=60.0,
        debounce_mm=0.3,
        up_stay_flip_mm=300.0,
        overlap=True,
        band_mm=4.4,
    ):
        self.overlap = overlap
        # overlap layout: filament mm from the lower edge of pos2 to pos3
        # (BUFFER_CALIBRATE measures it); 0 disables learning from it
        self.band_mm = band_mm
        self.band_entry = False  # current pos2 stay began at its lower edge
        self.m_pos1 = m_pos1
        self.m_approach_below = m_approach_below
        self.m_below = m_below
        self.m_target = m_target
        self.m_above = m_above
        self.m_approach_above = m_approach_above
        self.m_pos3 = m_pos3
        self.tension_fault_mm = tension_fault_mm
        self.compression_fault_mm = compression_fault_mm
        self.trim_limit = trim_limit
        self.trim_gain = trim_gain
        self.trim_nudge = trim_nudge
        self.hover_stall_mm = hover_stall_mm
        self.up_stay_flip_mm = up_stay_flip_mm
        self.stall_nudge = 0.0
        self.trim = 1.0
        # Sensor edges count only after the extruder has traveled
        # debounce_mm with the new state still holding (filters jitter)
        self.debounce_mm = debounce_mm
        self.travel = 0.0  # accumulated |extruder motion| in mm
        self.pending = {}  # name -> (state, direction, e_pos, travel)
        self.raw = {"pos1": False, "pos2": False, "pos3": False}
        self.sensors = {"pos1": False, "pos2": False, "pos3": False}
        self.zone = ZONE_UNKNOWN
        self.approach = False
        # Belief: while extruding at m_target, does the slider drift up?
        # (only true if the remaining ratio error exceeds 1 - m_target)
        self.drift_up = False
        self.fault = None
        self._new_fault = None
        self.faults_enabled = False
        self.last_e = None
        self.zone_entry_e = None
        # hover-cycle bookkeeping for auto-trim: pos2 -> below/above -> pos2
        self.cycle_pos2_mm = None
        self.trim_updates = 0
        # Fixed multiplier (unloading), see set_follow()
        self.follow = None
        self.reset_stats(None)

    # --- statistics (BUFFER_STATS, BUFFER_TEST_EXTRUDE) --------------------
    def reset_stats(self, e_pos):
        zones = (ZONE_POS1, ZONE_BELOW, ZONE_POS2, ZONE_ABOVE, ZONE_POS3, ZONE_UNKNOWN)
        self.stats = {
            "start_e": e_pos,
            "zone_mm": {z: 0.0 for z in zones},  # forward extrusion per zone
            "zone_entries": {z: 0 for z in zones},
            "first_pos2_mm": None,  # extruded mm until pos2 was first reached
            "trim_start": self.trim,
            "trim_updates_start": self.trim_updates,
        }

    # --- configuration ---------------------------------------------------
    def set_multipliers(self, **kw):
        for key, val in kw.items():
            if val is not None:
                setattr(self, key, val)

    def set_follow(self, multiplier):
        """A fixed multiplier instead of the zone control (None ends it).
        Unloading uses it: slack changes by (m - 1) * dE whichever way the
        extruder runs, so on a long retraction the forward multipliers drive
        the slider further the wrong way, and holding it at pos2 would press
        the tip into the gears the moment it comes out of them."""
        self.follow = multiplier

    # --- state -----------------------------------------------------------
    def multiplier(self):
        if self.follow is not None:
            return self.follow
        z = self.zone
        if z == ZONE_POS1:
            return self.m_pos1
        if z == ZONE_POS3:
            return self.m_pos3
        if z == ZONE_POS2:
            # approach mode here means coming down from pos3 (overlap layout)
            return self.m_approach_above if self.approach else self.m_target
        if z == ZONE_BELOW:
            return self.m_approach_below if self.approach else self.m_below
        if z == ZONE_ABOVE:
            return self.m_approach_above if self.approach else self.m_above
        return self.m_below  # unknown: gentle feed until a sensor reports

    def effective(self):
        return self.trim * self.multiplier()

    def set_raw_state(self, name, state):
        """Record a sensor state without zone logic (buffer not synced)."""
        if name in self.sensors:
            self.raw[name] = self.sensors[name] = bool(state)
            self.pending.pop(name, None)

    def reset_zone(self, e_pos):
        """Re-derive the zone from the current sensor states (after sync)."""
        self.pending.clear()
        self.sensors.update(self.raw)
        s = self.sensors
        approach = False
        if s["pos1"] and s["pos3"]:
            zone = ZONE_UNKNOWN
        elif s["pos1"]:
            zone = ZONE_POS1
        elif s["pos3"]:
            zone = ZONE_POS3
        elif s["pos2"]:
            zone = ZONE_POS2
        elif self.overlap:
            # nothing blocked can only be the gap between pos1 and pos2
            zone, approach = ZONE_BELOW, True
        else:
            zone = ZONE_UNKNOWN
        self._enter(zone, e_pos, approach=approach, learn=False)
        self.last_e = e_pos

    def set_faults_enabled(self, enabled, e_pos):
        if enabled and not self.faults_enabled:
            # Arm fresh: measure fault distances from now on
            self.zone_entry_e = e_pos
            self.fault = self._new_fault = None
        self.faults_enabled = enabled

    def _enter(self, zone, e_pos, approach=False, learn=True):
        prev, prev_approach = self.zone, self.approach
        prev_mm = None
        if self.zone_entry_e is not None and e_pos is not None:
            prev_mm = max(0.0, e_pos - self.zone_entry_e)
        if self.follow is not None:
            learn = False  # the trim learning assumes zone control
        if learn:
            # Hard sensor hits: correct the belief, and the trim only when
            # the correct-side hover rate failed to hold the slider
            if zone == ZONE_POS3:
                if prev == ZONE_POS2 and not prev_approach and self._learn_from_rise(prev_mm):
                    pass
                elif prev == ZONE_ABOVE or (prev == ZONE_POS2 and not prev_approach):
                    self._nudge_trim(-self.trim_nudge)
                elif prev == ZONE_BELOW and self.stall_nudge > 0.0:
                    # the stall nudge was based on the wrong side: undo it
                    self._nudge_trim(-self.stall_nudge)
                self.drift_up = True
                self.stall_nudge = 0.0
            elif zone == ZONE_POS1:
                if prev == ZONE_BELOW:
                    self._nudge_trim(self.trim_nudge)
                elif prev == ZONE_ABOVE and self.stall_nudge < 0.0:
                    self._nudge_trim(-self.stall_nudge)
                self.drift_up = False
                self.stall_nudge = 0.0
            elif zone == ZONE_POS2:
                self.stall_nudge = 0.0
            # Auto-trim from clean hover cycles at either edge of pos2 (a
            # pos2 stay in approach mode is a descent from pos3, not a hover)
            if (
                prev == ZONE_POS2
                and not prev_approach
                and zone in (ZONE_BELOW, ZONE_ABOVE)
            ):
                self.cycle_pos2_mm = prev_mm
            elif (
                prev in (ZONE_BELOW, ZONE_ABOVE)
                and not prev_approach
                and zone == ZONE_POS2
                and self.cycle_pos2_mm is not None
            ):
                self._learn_from_cycle(prev, self.cycle_pos2_mm, prev_mm)
                self.cycle_pos2_mm = None
            elif zone != ZONE_POS2:
                self.cycle_pos2_mm = None
        else:
            self.cycle_pos2_mm = None
        st = self.stats
        st["zone_entries"][zone] += 1
        if (
            zone == ZONE_POS2
            and st["first_pos2_mm"] is None
            and st["start_e"] is not None
            and e_pos is not None
        ):
            st["first_pos2_mm"] = max(0.0, e_pos - st["start_e"])
            # snapshots so behavior after reaching pos2 can be judged alone
            st["zone_mm_at_pos2"] = dict(st["zone_mm"])
            st["zone_entries_at_pos2"] = dict(st["zone_entries"])
        self.band_entry = zone == ZONE_POS2 and prev == ZONE_BELOW and learn
        self.zone = zone
        self.approach = approach
        self.zone_entry_e = e_pos

    def _learn_from_rise(self, p2_mm):
        """Overlap layout: the slider entered pos2 at its lower edge and rose
        through the whole band to pos3 while feeding at m_target. The band
        width over the extrusion it took is the remaining feed excess, so
        the trim can be set to zero error in one step. Overshooting low is
        harmless (it then hovers at the lower edge and learns from cycles);
        an undershoot just means another, slower, rise."""
        if not (self.overlap and self.band_entry and self.band_mm > 0.0 and p2_mm):
            return False
        rise = self.band_mm / p2_mm
        if rise > 2.0 * self.trim_limit + (1.0 - self.m_target):
            return False  # too fast to be a ratio error: a disturbance
        self._nudge_trim(self.m_target / (1.0 + rise) - 1.0)
        self.trim_updates += 1
        return True

    def _learn_from_cycle(self, side, p2_mm, side_mm):
        """One clean hover cycle (pos2 -> side -> pos2) reveals the remaining
        ratio error from the share of extrusion spent inside pos2. The trim
        always aims for zero error, so every filament ends up hovering at
        the BOTTOM edge of pos2 (steady push, never creeping toward pos3)."""
        if p2_mm is None or side_mm is None or p2_mm + side_mm < 1.0:
            return
        eps = 1.0 - self.m_target
        f2 = p2_mm / (p2_mm + side_mm)
        if side == ZONE_BELOW:
            delta = self.m_below - 1.0
            err = f2 * (delta + eps) - delta
        else:
            delta = 1.0 - self.m_above
            if delta <= eps:
                return
            err = delta - f2 * (delta - eps)
        self._nudge_trim(-self.trim_gain * err)
        self.trim_updates += 1
        if side == ZONE_ABOVE and err * (1.0 - self.trim_gain) < 0.5 * eps:
            # Top-edge hover only exists while the error exceeds eps. Once
            # the corrected error is safely below it the slider will drift
            # down inside pos2: switch belief now, before it proves it by
            # sinking all the way to pos1.
            self.drift_up = False

    def _nudge_trim(self, rel):
        lo, hi = 1.0 - self.trim_limit, 1.0 + self.trim_limit
        self.trim = min(hi, max(lo, self.trim * (1.0 + rel)))

    # --- inputs ----------------------------------------------------------
    def on_sensor(self, name, state, direction, e_pos):
        """Raw sensor edge. direction: +1 extruding, -1 retracting, 0
        stopped. Returns True if the multiplier may have changed."""
        if name not in self.sensors:
            return False
        state = bool(state)
        self.raw[name] = state
        if self.debounce_mm <= 0.0:
            return self._apply_edge(name, state, direction, e_pos)
        pend = self.pending.get(name)
        if pend is not None and pend[0] != state:
            del self.pending[name]  # flipped back before it counted: jitter
            return False
        if pend is None and state == self.sensors[name]:
            return False
        # extruder travel at the moment of the edge (not at the last sample)
        travel_edge = self.travel
        if self.last_e is not None:
            travel_edge += abs(e_pos - self.last_e)
        self.pending[name] = (state, direction, e_pos, travel_edge)
        return False

    def _commit_pending(self):
        changed = False
        for name, pend in sorted(self.pending.items(), key=lambda kv: kv[1][3]):
            state, direction, e_edge, travel_edge = pend
            if self.travel - travel_edge < self.debounce_mm:
                continue
            del self.pending[name]
            changed = self._apply_edge(name, state, direction, e_edge) or changed
        return changed

    def _apply_edge(self, name, state, direction, e_pos):
        self.sensors[name] = state
        s = self.sensors
        if s["pos1"] and s["pos3"]:
            self._set_fault("impossible sensor state: pos1 and pos3 both blocked")
            self._enter(ZONE_UNKNOWN, e_pos, learn=False)
            return True
        approach = False
        if name == "pos1":
            if state:
                zone = ZONE_POS1
            elif s["pos2"]:
                zone = ZONE_POS2
            else:
                zone, approach = ZONE_BELOW, True
        elif name == "pos3":
            if state:
                zone = ZONE_POS3
            elif s["pos2"]:
                # Left pos3 downward. With overlapping sensors the slider is
                # now somewhere in pos2: keep relieving until its lower edge.
                zone, approach = ZONE_POS2, self.overlap
            else:
                zone, approach = ZONE_ABOVE, True
        elif state:  # pos2 rising
            # with pos3 still blocked the slider is coming down from the top
            zone = ZONE_POS3 if s["pos3"] else ZONE_POS2
        elif s["pos1"]:
            zone = ZONE_POS1
        elif s["pos3"]:
            zone = ZONE_POS3
        elif self.overlap:
            # pos2 covers everything from its lower edge up through pos3, so
            # clearing with pos3 clear can only mean the slider went down
            zone = ZONE_BELOW
        else:
            # pos2 falling: which side did the slider leave on? Inside pos2
            # the multiplier is below 1, so with the trim tuned the slider
            # drifts down while extruding; retracting reverses it.
            up = self.drift_up if direction >= 0 else not self.drift_up
            zone = ZONE_ABOVE if up else ZONE_BELOW
        if zone != self.zone or approach != self.approach:
            self._enter(zone, e_pos, approach=approach)
        return True

    def on_progress(self, e_pos):
        """Periodic extruder position sample. Returns a fault message when a
        fault is newly detected, else None."""
        if self.last_e is not None:
            step = e_pos - self.last_e
            self.travel += abs(step)
            if step > 0.0:
                self.stats["zone_mm"][self.zone] += step
        self.last_e = e_pos
        self._commit_pending()
        if self.zone_entry_e is None:
            self.zone_entry_e = e_pos
            return self.take_new_fault()
        net = e_pos - self.zone_entry_e
        if self.follow is not None:
            # no trim nudges or stall handling at a fixed multiplier
            return self.take_new_fault()
        if self.zone == ZONE_POS2 and self.drift_up and net > self.up_stay_flip_mm:
            # Believed to drift up, yet it has stayed inside pos2 this long:
            # any upward drift is negligible, so the trim has crossed over.
            # Expect a bottom exit (otherwise it would sink to pos1 unseen).
            self.drift_up = False
        if self.zone == ZONE_UNKNOWN and not self.approach and net > self.hover_stall_mm:
            # No sensor has reported for a long time: go find one rather
            # than sit blind (a slider stuck at rest would otherwise starve)
            self.zone, self.approach = ZONE_BELOW, True
            self.zone_entry_e = e_pos
        elif (
            self.zone in (ZONE_BELOW, ZONE_ABOVE)
            and not self.approach
            and net > self.hover_stall_mm
        ):
            # A hover leg lasting far longer than expected means the gentle
            # rate barely beats the ratio error: correct trim, approach pos2
            sign = 1.0 if self.zone == ZONE_BELOW else -1.0
            self._nudge_trim(sign * self.trim_nudge)
            self.stall_nudge = sign * self.trim_nudge
            self.drift_up = self.zone == ZONE_ABOVE
            self.cycle_pos2_mm = None
            self.approach = True
            self.zone_entry_e = e_pos
        elif self.faults_enabled and self.fault is None:
            if self.zone == ZONE_POS1 and net > self.tension_fault_mm:
                self._set_fault(
                    "tension: still at pos1 after %.1f mm extruded while feeding"
                    " at x%.2f - feed is going missing (tangle, slipping gear or"
                    " skipping motor)" % (net, self.m_pos1)
                )
            elif self.zone == ZONE_POS3 and net > self.compression_fault_mm:
                self._set_fault(
                    "compression: still at pos3 after %.1f mm extruded while"
                    " feeding at only x%.2f - filament is not being consumed"
                    " (clog or extruder slipping)" % (net, self.m_pos3)
                )
        return self.take_new_fault()

    def _set_fault(self, msg):
        if not self.faults_enabled or self.fault is not None:
            return None
        self.fault = msg
        self._new_fault = msg
        return msg

    def take_new_fault(self):
        """Return a newly detected fault exactly once, else None."""
        msg, self._new_fault = self._new_fault, None
        return msg

    def raise_fault(self, msg):
        """External fault (e.g. inlet runout). Only counts while armed."""
        return self._set_fault(msg)

    def clear_fault(self):
        self.fault = self._new_fault = None


def calibration_result(b, e):
    """BUFFER_CALIBRATE math. b: buffer positions (commanded mm) of the
    sensor edges while the buffer pushed the slider up with the extruder
    holding; e: extruder positions (mm) of the edges while the extruder
    pulled it back down with the buffer holding. Keys are (sensor, state).

    The span from leaving pos1 to reaching pos3 is the same both ways when
    pos1 and pos3 have equal hysteresis, so the buffer's true feed per
    commanded mm is the extruder span over the buffer span (docs/CONTROL.md,
    section 9)."""
    b_span = b[("pos3", True)] - b[("pos1", False)]
    e_span = e[("pos1", True)] - e[("pos3", False)]
    if b_span <= 0.0 or e_span <= 0.0:
        raise ValueError("sensor edges out of order")
    ratio = e_span / b_span
    geometry = {"span_mm": e_span}
    if ("pos2", True) in b:
        geometry["gap_mm"] = (b[("pos2", True)] - b[("pos1", False)]) * ratio
        geometry["band_mm"] = (b[("pos3", True)] - b[("pos2", True)]) * ratio
    # overlapping layout: pos2 never cleared on the way up to pos3
    geometry["overlap"] = ("pos2", False) not in b
    return ratio, geometry


def slack_result(up, down):
    """BUFFER_TEST_SLACK math. up, down: buffer positions (mm) of each
    (sensor, state) edge while the buffer drove the slider up to pos3 and back
    down to pos1 with the far end held. A rigid path trips every sensor at
    the same buffer position both ways; the difference is the dead band in
    the path (filament snaking in the tube, friction, gear backlash). Returns
    that per sensor, and the pos1-to-pos3 span each way."""
    pairs = (
        ("pos1", ("pos1", False), ("pos1", True)),
        ("pos2", ("pos2", True), ("pos2", False)),
        ("pos3", ("pos3", True), ("pos3", False)),
    )
    dead = {}
    for name, u, d in pairs:
        if u in up and d in down:
            dead[name] = up[u] - down[d]
    span_up = up[("pos3", True)] - up[("pos1", False)]
    span_down = down[("pos3", False)] - down[("pos1", True)]
    return dead, span_up, span_down


def _mean_sd(values):
    n = len(values)
    mean = sum(values) / n
    sd = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1)) if n > 1 else 0.0
    return mean, sd


######################################################################
# Independent buffer moves (idle only)
######################################################################


def _klippy_module(name):
    try:
        return importlib.import_module("klippy." + name)
    except ImportError:
        return importlib.import_module(name)


def _move_time(dist, speed, accel):
    """Trapezoid timing for one move (same as force_move.calc_move_time)."""
    axis_r = -1.0 if dist < 0.0 else 1.0
    dist = abs(dist)
    if not accel or not dist:
        return axis_r, 0.0, dist / speed, speed
    if dist * accel < speed * speed:
        speed = math.sqrt(dist * accel)
    accel_t = speed / accel
    return axis_r, accel_t, (dist - accel_t * speed) / speed, speed


class _AbortableCompletion:
    """A drip move ends when its completion is done; this one is also done
    when abort() returns True."""

    def __init__(self, inner, abort):
        self.inner = inner
        self.abort = abort

    def test(self):
        return self.inner.test() or (self.abort is not None and bool(self.abort()))

    def wait(self, *args, **kwargs):
        return self.inner.wait(*args, **kwargs)


class _TriggerRecorder:
    """Passes everything through to an endstop and remembers the trigger
    time home_wait reports (0 when it did not trigger)."""

    def __init__(self, endstop):
        self._endstop = endstop
        self.trigger_time = 0.0

    def __getattr__(self, name):
        return getattr(self._endstop, name)

    def home_wait(self, home_end_time):
        self.trigger_time = self._endstop.home_wait(home_end_time)
        return self.trigger_time


class BufferMover:
    """Moves the buffer motor on its own while it is unsynced (never while
    printing). A move can end early: when an endstop on a slider sensor
    triggers (the buffer MCU stops the motor itself, so the stop is exact),
    or when abort() returns True (checked about every 0.1 s, e.g. a released
    button). This is the toolhead-like interface manual_stepper offers for
    homing, applied to the extruder_stepper's motor."""

    def __init__(self, printer, config, stepper):
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.stepper = stepper
        self.motion_queuing = printer.load_object(config, "motion_queuing")
        self.trapq = self.motion_queuing.allocate_trapq()
        self.trapq_append = self.motion_queuing.lookup_trapq_append()
        ffi_main, ffi_lib = _klippy_module("chelper").get_ffi()
        self.sk = ffi_main.gc(ffi_lib.cartesian_stepper_alloc(b"x"), ffi_lib.free)
        self.toolhead = None
        self.next_cmd_time = 0.0
        self.commanded_pos = 0.0
        self.accel = 0.0
        self.abort = None
        self.prev = None

    def move(self, dist, speed, accel, endstop=None, name="sensor", abort=None):
        """Returns (mm actually moved, endstop triggered)."""
        self.toolhead = self.printer.lookup_object("toolhead")
        self._attach()
        start = self.stepper.get_mcu_position()
        triggered = False
        self.accel, self.abort = accel, abort
        try:
            target = [dist, 0.0, 0.0, 0.0]
            if endstop is None:
                self.drip_move(target, speed, self.reactor.completion())
            else:
                homing = _klippy_module("extras.homing")
                recorder = _TriggerRecorder(endstop)
                hmove = homing.HomingMove(self.printer, [(recorder, name)], self)
                hmove.homing_move(target, speed, triggered=True, check_triggered=False)
                triggered = recorder.trigger_time > 0.0
        finally:
            self.abort = None
            self._detach()
        moved = (self.stepper.get_mcu_position() - start) * self.stepper.get_step_dist()
        return moved, triggered

    def _attach(self):
        self.toolhead.flush_step_generation()
        prev_sk = self.stepper.set_stepper_kinematics(self.sk)
        prev_tq = self.stepper.set_trapq(self.trapq)
        self.prev = (prev_sk, prev_tq)
        self.commanded_pos = 0.0
        self.stepper.set_position([0.0, 0.0, 0.0])
        self.next_cmd_time = 0.0

    def _detach(self):
        self.toolhead.flush_step_generation()
        prev_sk, prev_tq = self.prev
        self.stepper.set_trapq(prev_tq)
        self.stepper.set_stepper_kinematics(prev_sk)
        self.motion_queuing.wipe_trapq(self.trapq)

    # --- toolhead-like interface used by HomingMove ------------------------
    def sync_print_time(self):
        print_time = self.toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            self.toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time

    def flush_step_generation(self):
        self.toolhead.flush_step_generation()

    def get_position(self):
        return [self.commanded_pos, 0.0, 0.0, 0.0]

    def set_position(self, newpos, homing_axes=""):
        self.toolhead.flush_step_generation()
        self.commanded_pos = newpos[0]
        self.stepper.set_position([self.commanded_pos, 0.0, 0.0])

    def get_last_move_time(self):
        self.sync_print_time()
        return self.next_cmd_time

    def dwell(self, delay):
        self.next_cmd_time += max(0.0, delay)

    def drip_move(self, newpos, speed, drip_completion):
        self.sync_print_time()
        start_time = self.next_cmd_time
        cp = self.commanded_pos
        axis_r, accel_t, cruise_t, cruise_v = _move_time(newpos[0] - cp, speed, self.accel)
        self.trapq_append(
            self.trapq, start_time, accel_t, cruise_t, accel_t,
            cp, 0.0, 0.0, axis_r, 0.0, 0.0, 0.0, cruise_v, self.accel,
        )
        self.commanded_pos = newpos[0]
        end_time = start_time + 2.0 * accel_t + cruise_t
        self.motion_queuing.drip_update_time(
            start_time, end_time, _AbortableCompletion(drip_completion, self.abort)
        )
        # Clear what is left of the move if it stopped early
        self.motion_queuing.wipe_trapq(self.trapq)
        self.stepper.set_position([self.commanded_pos, 0.0, 0.0])
        self.sync_print_time()

    def get_kinematics(self):
        return self

    def get_steppers(self):
        return [self.stepper]

    def calc_position(self, stepper_positions):
        return [stepper_positions[self.stepper.get_name()], 0.0, 0.0]


######################################################################
# Kalico adapter
######################################################################


class FilamentBuffer:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.name = config.get_name()
        self.stepper_name = config.get("extruder_stepper", "buffer")
        self.extruder_name = config.get("extruder", "extruder")
        self.report_events = config.getboolean("report_events", True)
        self.button_speed = config.getfloat("button_speed", 20.0, above=0.0)
        self.max_move_speed = config.getfloat("max_move_speed", 60.0, above=0.0)
        self.move_accel = config.getfloat("move_accel", 500.0, minval=0.0)
        # Loading (autoload on insert, or BUFFER_LOAD): feed until the slider
        # reaches pos2, which means the tip is pressed against the extruder
        self.autoload = config.getboolean("autoload", True)
        self.autoload_delay = config.getfloat("autoload_delay", 1.0, minval=0.0)
        self.load_speed = config.getfloat(
            "load_speed", 30.0, above=0.0, maxval=self.max_move_speed
        )
        self.load_max_mm = config.getfloat(
            "load_max_mm", 1500.0, above=0.0, maxval=JOG_MAX_MM
        )
        self.load_grab_mm = config.getfloat("load_grab_mm", 20.0, minval=0.0)
        self.load_grab_speed = config.getfloat(
            "load_grab_speed", 10.0, above=0.0, maxval=self.max_move_speed
        )
        # The motor only needs power while it moves. Held at full current it
        # cooks itself in the closed buffer box (the stock firmware switched
        # it off after every move)
        self.idle_motor_off = config.getboolean("idle_motor_off", True)
        # Slider geometry in filament mm of stored slack: the top of the pos1
        # window and the lower edge of pos2 (LLL Plus: 19 and 28.6)
        self.pos1_slack_mm = config.getfloat("pos1_slack_mm", 19.0, above=0.0)
        self.pos2_slack_mm = config.getfloat(
            "pos2_slack_mm", 28.6, above=self.pos1_slack_mm
        )
        # Path lengths. Measured by autoload (inlet to extruder gears) and
        # BUFFER_UNLOAD (gears to nozzle), kept with SAVE_VARIABLE when
        # [save_variables] exists. Set here to override
        self.cfg_path_mm = config.getfloat("path_mm", None, above=0.0)
        self.cfg_nozzle_mm = config.getfloat("nozzle_mm", None, above=0.0)
        self.path_mm = self.cfg_path_mm
        self.nozzle_mm = self.cfg_nozzle_mm
        # Unloading (BUFFER_UNLOAD), all overridable per call
        self.unload_fast_mm = config.getfloat("unload_fast_mm", 25.0, above=0.0)
        self.unload_fast_speed = config.getfloat("unload_fast_speed", 35.0, above=0.0)
        self.unload_speed = config.getfloat("unload_speed", 20.0, above=0.0)
        self.unload_ram_mm = config.getfloat("unload_ram_mm", 3.0, minval=0.0)
        self.unload_test_mm = config.getfloat(
            "unload_test_mm", self.pos1_slack_mm + 3.0, above=self.pos1_slack_mm
        )
        # extruder retraction past the gears-to-nozzle length: room for the
        # free test with a margin, and the most the buffer can slip if stuck
        self.unload_overrun_mm = config.getfloat(
            "unload_overrun_mm", self.unload_test_mm + 23.0,
            above=self.unload_test_mm,
        )
        self.park_mm = config.getfloat("park_mm", 50.0, above=0.0)
        self.eject_margin_mm = config.getfloat("eject_margin_mm", 60.0, above=0.0)
        layout = config.get("sensor_layout", "overlap")
        if layout not in ("overlap", "separate"):
            raise config.error("sensor_layout must be 'overlap' or 'separate'")
        # Defaults come from FeedController itself (the values the offline
        # simulation suite validates), so config and tests can't drift apart
        d = FeedController()
        self.ctrl = FeedController(
            m_pos1=config.getfloat("m_pos1", d.m_pos1, above=1.0),
            m_approach_below=config.getfloat(
                "m_approach_below", d.m_approach_below, above=1.0
            ),
            m_below=config.getfloat("m_below", d.m_below, above=1.0),
            m_target=config.getfloat("m_target", d.m_target, above=0.0, below=1.0),
            m_above=config.getfloat("m_above", d.m_above, above=0.0, below=1.0),
            m_approach_above=config.getfloat(
                "m_approach_above", d.m_approach_above, above=0.0, below=1.0
            ),
            m_pos3=config.getfloat("m_pos3", d.m_pos3, above=0.0, below=1.0),
            tension_fault_mm=config.getfloat(
                "tension_fault_mm", d.tension_fault_mm, above=0.0
            ),
            compression_fault_mm=config.getfloat(
                "compression_fault_mm", d.compression_fault_mm, above=0.0
            ),
            trim_limit=config.getfloat(
                "trim_limit", d.trim_limit, minval=0.0, maxval=0.2
            ),
            trim_gain=config.getfloat("trim_gain", d.trim_gain, minval=0.0, maxval=1.0),
            trim_nudge=config.getfloat(
                "trim_nudge", d.trim_nudge, minval=0.0, maxval=0.05
            ),
            hover_stall_mm=config.getfloat(
                "hover_stall_mm", d.hover_stall_mm, above=0.0
            ),
            debounce_mm=config.getfloat("debounce_mm", d.debounce_mm, minval=0.0),
            up_stay_flip_mm=config.getfloat(
                "up_stay_flip_mm", d.up_stay_flip_mm, above=0.0
            ),
            overlap=layout == "overlap",
            band_mm=config.getfloat("band_mm", d.band_mm, minval=0.0),
        )
        # Pins: all on the buffer MCU. state True = blocked/present/pressed
        self.pin_states = {}
        buttons = self.printer.load_object(config, "buttons")
        pin_options = [
            ("pos1", "pos1_pin", True),
            ("pos2", "pos2_pin", True),
            ("pos3", "pos3_pin", True),
            ("inlet", "inlet_pin", True),
            ("key_feed", "feed_button_pin", False),
            ("key_retract", "retract_button_pin", False),
        ]
        for key, option, required in pin_options:
            pin = config.get(option) if required else config.get(option, None)
            if pin is None:
                continue
            self.pin_states[key] = False
            buttons.register_buttons(
                [pin], (lambda et, st, k=key: self._pin_event(k, et, st))
            )
        # pos2 and pos3 double as endstops for idle moves: the buffer MCU
        # stops the motor the moment the slider gets there
        ppins = self.printer.lookup_object("pins")
        pes = self.printer.load_object(config, "extruder_stepper " + self.stepper_name)
        stepper = pes.extruder_stepper.stepper
        self.endstops = {}
        for key in ("pos2", "pos3"):
            pin = config.get(key + "_pin")
            ppins.allow_multi_use_pin(pin.lstrip("^~! "))
            endstop = ppins.setup_pin("endstop", pin)
            endstop.add_stepper(stepper)
            self.endstops[key] = endstop
        self.mover = BufferMover(self.printer, config, stepper)
        self.ok_led_name = config.get("ok_led", None)
        self.fault_led_name = config.get("fault_led", None)
        self.printer.load_object(config, "pause_resume")
        # Runtime state
        self.synced = False
        self.applied_mult = 1.0
        self.last_rate_change = 0.0
        self.pending_rate_change = False
        self.rate_changes = 0
        self.rate_changes_at_reset = 0
        self.pause_pending = False
        self.button_held = None
        self.button_loop_running = False
        self.loading = False
        self.unloading = False
        self.keep_motor = False  # sequences that need the motor held between moves
        self.fresh_insert = False  # the next load starts with the tip at the inlet
        # Klipper reports every input at startup. Only an inlet seen empty and
        # then filled is an insert; filament that was already there is not
        self.inlet_was_empty = False
        self.load_cancel = False
        self.inlet_clear_since = None
        self.last_load_mm = None
        self.edge_log = None  # (sensor, state, eventtime) while calibrating
        self.toolhead = self.extruder = self.mcu = None
        self.es = self.mcu_stepper = None
        self.base_rd = None
        self.print_stats = self.pause_resume = None
        self.leds = {}
        self.update_timer = self.reactor.register_timer(self._update)
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        for cmd in (
            "BUFFER_STATUS",
            "BUFFER_STATS",
            "BUFFER_SYNC",
            "BUFFER_UNSYNC",
            "BUFFER_MOVE",
            "BUFFER_LOAD",
            "BUFFER_UNLOAD",
            "BUFFER_CALIBRATE",
            "BUFFER_SET",
            "BUFFER_TEST_EXTRUDE",
            "BUFFER_TEST_SLACK",
            "BUFFER_TEST_SPEED",
        ):
            self.gcode.register_command(
                cmd,
                getattr(self, "cmd_" + cmd),
                desc=getattr(self, "cmd_%s_help" % cmd),
            )

    # --- lifecycle -------------------------------------------------------
    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        self.mcu = self.printer.lookup_object("mcu")
        self.extruder = self.printer.lookup_object(self.extruder_name)
        pes = self.printer.lookup_object("extruder_stepper " + self.stepper_name)
        self.es = pes.extruder_stepper
        self.mcu_stepper = self.es.stepper
        self.base_rd = self.mcu_stepper.get_rotation_distance()[0]
        self.print_stats = self.printer.lookup_object("print_stats")
        self.pause_resume = self.printer.lookup_object("pause_resume")
        for led in (self.ok_led_name, self.fault_led_name):
            if led:
                self.leds[led] = self.printer.lookup_object("output_pin " + led)
        self.synced = bool(self.es.motion_queue)

    def _handle_ready(self):
        # extruder_stepper applies its configured sync on connect; read the
        # result here so config section order can't matter
        self.synced = bool(self.es.motion_queue)
        self._load_saved()
        self._update_leds()
        self.reactor.update_timer(self.update_timer, self.reactor.NOW)

    # --- measured distances ----------------------------------------------
    def _load_saved(self):
        sv = self.printer.lookup_object("save_variables", None)
        if sv is None:
            return
        try:
            saved = sv.get_status(self.reactor.monotonic()).get("variables", {})
        except Exception:
            logging.exception("filament_buffer: reading save_variables")
            return
        if self.cfg_path_mm is None and saved.get("buffer_path_mm"):
            self.path_mm = float(saved["buffer_path_mm"])
        if self.cfg_nozzle_mm is None and saved.get("buffer_nozzle_mm"):
            self.nozzle_mm = float(saved["buffer_nozzle_mm"])

    def _remember(self, gcmd, what, value):
        """Keep a measured distance for this session, and across restarts
        when [save_variables] exists (a config value always wins)."""
        if what == "path":
            if self.cfg_path_mm is not None:
                return
            self.path_mm = value
        else:
            if self.cfg_nozzle_mm is not None:
                return
            self.nozzle_mm = value
        if self.printer.lookup_object("save_variables", None) is not None:
            self.gcode.run_script_from_command(
                "SAVE_VARIABLE VARIABLE=buffer_%s_mm VALUE=%.1f" % (what, value)
            )
        else:
            gcmd.respond_info(
                "buffer: add [save_variables] to keep the %s length across"
                " restarts, or put %s_mm: %.0f in [filament_buffer]" % (what, what, value)
            )

    # --- motor power -----------------------------------------------------
    def _motor_off(self):
        """Switch the buffer motor off while it is idle. Kalico switches it
        back on by itself at its next step (stepper_enable)."""
        if not self.idle_motor_off or self.synced or self.keep_motor:
            return
        se = self.printer.lookup_object("stepper_enable", None)
        if se is None or self.mcu_stepper is None:
            return
        try:
            se.set_motors_enable([self.mcu_stepper.get_name()], False)
        except Exception:
            logging.exception("filament_buffer: switching the motor off")

    # --- helpers ---------------------------------------------------------
    def _print_state(self):
        return self.print_stats.get_status(self.reactor.monotonic())["state"]

    def _is_printing(self):
        return self._print_state() == "printing"

    def _faults_should_be_enabled(self):
        return (
            self.synced
            and self._is_printing()
            and not self.pause_resume.get_status(0)["is_paused"]
        )

    def _extruder_pos(self, eventtime):
        pt = self.mcu.estimated_print_time(eventtime)
        return self.extruder.find_past_position(pt)

    def _extruder_direction(self, eventtime):
        pt = self.mcu.estimated_print_time(eventtime)
        now = self.extruder.find_past_position(pt)
        past = self.extruder.find_past_position(max(0.0, pt - DIRECTION_WINDOW))
        diff = now - past
        if diff > DIRECTION_DEADBAND:
            return 1
        if diff < -DIRECTION_DEADBAND:
            return -1
        return 0

    def _respond(self, msg):
        self.gcode.respond_info("buffer: " + msg)

    def _set_led(self, name, value):
        led = self.leds.get(name)
        if led is not None:
            led.gcrq.send_async_request(1.0 if value else 0.0)

    def _update_leds(self):
        if self.ok_led_name:
            self._set_led(self.ok_led_name, self.pin_states.get("inlet", False))
        if self.fault_led_name:
            self._set_led(self.fault_led_name, self.ctrl.fault is not None)

    def _apply_rate(self, eventtime, force=False):
        """The ONLY mid-print action: change the buffer step distance. No flush."""
        if not self.synced or self.mcu_stepper is None:
            return
        mult = self.ctrl.effective()
        if abs(mult - self.applied_mult) < 1e-6:
            self.pending_rate_change = False
            return
        if not force and eventtime - self.last_rate_change < MIN_RATE_CHANGE_INTERVAL:
            self.pending_rate_change = True
            return
        self.mcu_stepper.set_rotation_distance(self.base_rd / mult)
        self.applied_mult = mult
        self.last_rate_change = eventtime
        self.pending_rate_change = False
        self.rate_changes += 1

    def _reset_rate(self):
        if self.mcu_stepper is not None:
            self.mcu_stepper.set_rotation_distance(self.base_rd)
        self.applied_mult = 1.0

    def _require_not_printing(self, gcmd, what):
        if self._is_printing():
            raise gcmd.error(
                "%s is not allowed while printing (the buffer must never"
                " interrupt a print)" % (what,)
            )

    def _require_empty_queue(self, gcmd, what):
        if self.toolhead.lookahead.get_last() is not None:
            raise gcmd.error(
                "%s needs the toolhead stopped (moves are queued). Call it"
                " after G28/QUAD_GANTRY_LEVEL or an M400 so it can't add a"
                " stop to a print." % (what,)
            )

    def _do_sync(self, sync):
        self._reset_rate()
        self.es.sync_to_extruder(self.extruder_name if sync else "")
        self.synced = sync
        self.ctrl.set_follow(None)
        if sync:
            e_pos = self._extruder_pos(self.reactor.monotonic())
            self.ctrl.reset_zone(e_pos)
            self._apply_rate(self.reactor.monotonic(), force=True)

    def _set_follow(self, multiplier):
        """Fixed multiplier on a synced buffer. Only the controller state and
        the step distance change: no flush."""
        self.ctrl.set_follow(multiplier)
        self._apply_rate(self.reactor.monotonic(), force=True)

    # --- events (reactor context: never raise) ---------------------------
    def _pin_event(self, key, eventtime, state):
        try:
            self.pin_states[key] = bool(state)
            if self.edge_log is not None and key in ("pos1", "pos2", "pos3"):
                self.edge_log.append((key, bool(state), eventtime))
            if self.report_events:
                labels = {
                    "inlet": ("absent", "present"),
                    "key_feed": ("released", "pressed"),
                    "key_retract": ("released", "pressed"),
                }
                off, on = labels.get(key, ("clear", "blocked"))
                self._respond("%s -> %s" % (key, on if state else off))
            if key in ("pos1", "pos2", "pos3"):
                direction = e_pos = 0
                if self.mcu is not None:
                    direction = self._extruder_direction(eventtime)
                    e_pos = self._extruder_pos(eventtime)
                if not self.synced:
                    # Unsynced: the extruder doesn't drive the buffer - just
                    # track states; the zone is re-derived on sync
                    self.ctrl.set_raw_state(key, state)
                    return
                self.ctrl.on_sensor(key, state, direction, e_pos)
                fault = self.ctrl.take_new_fault()
                if fault is not None:
                    self._fault(fault)
                self._apply_rate(eventtime)
            elif key == "inlet":
                self._update_leds()
                if state:
                    self._maybe_autoload(eventtime)
                    self.inlet_was_empty = False
                else:
                    self.inlet_was_empty = True
                if not state and self.synced and self.mcu is not None:
                    # arm/disarm now rather than waiting for the next tick
                    self.ctrl.set_faults_enabled(
                        self._faults_should_be_enabled(), self._extruder_pos(eventtime)
                    )
                    self.ctrl.raise_fault("runout: no filament at the buffer inlet")
                    fault = self.ctrl.take_new_fault()
                    if fault is not None:
                        self._fault(fault)
            elif key in ("key_feed", "key_retract"):
                self._button(key, state)
        except Exception:
            logging.exception("filament_buffer: error handling %s event", key)

    def _update(self, eventtime):
        try:
            if self.mcu is None:
                return eventtime + UPDATE_INTERVAL
            print_state = self._print_state()
            e_pos = self._extruder_pos(eventtime)
            enable = self._faults_should_be_enabled()
            if enable and not self.ctrl.faults_enabled:
                self._update_leds()
            self.ctrl.set_faults_enabled(enable, e_pos)
            if self.synced:
                fault = self.ctrl.on_progress(e_pos)
                if fault is not None:
                    self._fault(fault)
                if self.pending_rate_change or abs(
                    self.ctrl.effective() - self.applied_mult
                ) > 1e-6:
                    self._apply_rate(eventtime)
            # A fault stays latched (red LED) while the print is paused; it
            # clears when the print ends. Resuming re-arms detection fresh.
            if self.ctrl.fault and print_state not in ("printing", "paused"):
                self.ctrl.clear_fault()
                self._update_leds()
        except Exception:
            logging.exception("filament_buffer: error in update timer")
        return eventtime + UPDATE_INTERVAL

    def _fault(self, msg):
        self._update_leds()
        logging.warning("filament_buffer: FAULT %s", msg)
        if self.pause_pending or not self._is_printing():
            return
        if self.pause_resume.get_status(0)["is_paused"]:
            return
        self.pause_pending = True
        self.reactor.register_callback(lambda et, m=msg: self._pause(et, m))

    def _pause(self, eventtime, msg):
        try:
            self.gcode.respond_raw("!! buffer: FAULT - %s. Pausing the print." % msg)
            self.pause_resume.send_pause_command()
            self.reactor.pause(eventtime + 0.5)
            self.gcode.run_script("PAUSE")
        except Exception:
            logging.exception("filament_buffer: error while pausing")
        finally:
            self.pause_pending = False

    # --- buttons and autoload (only when not printing) -------------------
    def _button(self, key, state):
        if not state:
            if self.button_held == key:
                self.button_held = None
            return
        if self._is_printing():
            self._respond("%s ignored while printing" % key)
            return
        if self.loading:
            self.load_cancel = True
            return
        self.button_held = key
        if not self.button_loop_running:
            self.button_loop_running = True
            self.reactor.register_callback(self._button_loop)

    def _button_loop(self, eventtime):
        # One continuous move for as long as the button is held. FEED stops
        # by itself at pos3 (fully compressed), so holding it too long can't
        # push the tube out of its fitting.
        try:
            with self.gcode.get_mutex():
                key = self.button_held
                if key is None or self._is_printing():
                    return
                feed = key == "key_feed"
                moved, at_pos3 = self._idle_move(
                    JOG_MAX_MM if feed else -JOG_MAX_MM,
                    self.button_speed,
                    endstop_key="pos3" if feed else None,
                    abort=lambda: self.button_held != key or self._is_printing(),
                )
                if at_pos3:
                    self._respond("feed stopped at pos3 (slider fully compressed)")
        except Exception:
            logging.exception("filament_buffer: error in button move")
        finally:
            self.button_loop_running = False

    def _maybe_autoload(self, eventtime):
        if not self.autoload or self.synced or self.loading:
            return
        if not self.inlet_was_empty:
            return  # present at startup, not an insert
        if self._print_state() in ("printing", "paused"):
            return
        # Only into an empty path: the slider rests at pos1 with no filament
        if not self.pin_states.get("pos1"):
            return
        self.reactor.register_callback(self._autoload, eventtime + self.autoload_delay)

    def _autoload(self, eventtime):
        try:
            if not self.pin_states.get("inlet") or not self.pin_states.get("pos1"):
                return
            if self.synced or self.loading or self._print_state() in ("printing", "paused"):
                return
            self._respond("filament detected, loading it to the extruder")
            # the tip starts at the inlet, so this load measures the path
            self.fresh_insert = True
            self.gcode.run_script("BUFFER_LOAD")
        except Exception as e:
            logging.exception("filament_buffer: autoload")
            self.gcode.respond_raw("!! buffer: autoload: %s" % (e,))

    def _load_abort(self):
        if self.load_cancel or self._is_printing():
            return True
        if self.pin_states.get("inlet"):
            self.inlet_clear_since = None
            return False
        now = self.reactor.monotonic()
        if self.inlet_clear_since is None:
            self.inlet_clear_since = now
        return now - self.inlet_clear_since > LOAD_INLET_GRACE

    def _idle_move(self, dist, speed, endstop_key=None, abort=None):
        """Independent buffer move (caller ensures not printing). Unsyncs
        for the move and resyncs after. Returns (mm moved, endstop hit)."""
        was_synced = self.synced
        if was_synced:
            self._do_sync(False)
        try:
            return self.mover.move(
                dist,
                speed,
                self.move_accel,
                endstop=self.endstops.get(endstop_key),
                name=endstop_key or "",
                abort=abort,
            )
        finally:
            if was_synced:
                self._do_sync(True)
            else:
                self._motor_off()

    def _settle(self):
        """Let sensor reports from the end of a move arrive."""
        self.reactor.pause(self.reactor.monotonic() + SENSOR_SETTLE)

    # --- commands --------------------------------------------------------
    cmd_BUFFER_STATUS_help = "Report the filament buffer state"

    def cmd_BUFFER_STATUS(self, gcmd):
        c = self.ctrl
        lines = [
            "synced=%s zone=%s multiplier=%.3f trim=%.4f (applied x%.4f, %d trim updates)"
            % (
                self.synced,
                c.zone,
                c.multiplier(),
                c.trim,
                self.applied_mult,
                c.trim_updates,
            ),
            "sensors: "
            + " ".join(
                "%s=%s" % (k, "1" if v else "0") for k, v in sorted(self.pin_states.items())
            ),
            "rotation_distance base=%.4f active=%.4f | faults %s | fault: %s"
            % (
                self.base_rd,
                self.mcu_stepper.get_rotation_distance()[0],
                "ARMED" if c.faults_enabled else "off",
                c.fault or "none",
            ),
            "path inlet to gears %s | gears to nozzle %s"
            % (
                "%.0f mm" % self.path_mm if self.path_mm else "not measured",
                "%.0f mm" % self.nozzle_mm if self.nozzle_mm else "not measured",
            ),
        ]
        gcmd.respond_info("\n".join("buffer: " + l for l in lines))

    cmd_BUFFER_SYNC_help = "Sync the buffer motor to the extruder (toolhead must be stopped)"

    def cmd_BUFFER_SYNC(self, gcmd):
        if self.synced:
            gcmd.respond_info("buffer: already synced")
            return
        self._require_empty_queue(gcmd, "BUFFER_SYNC")
        self._do_sync(True)
        gcmd.respond_info(
            "buffer: synced to %s, zone=%s, x%.3f"
            % (self.extruder_name, self.ctrl.zone, self.applied_mult)
        )

    cmd_BUFFER_UNSYNC_help = (
        "Unsync the buffer motor from the extruder (while printing only with the"
        " toolhead stopped, e.g. after M400 in PRINT_END)"
    )

    def cmd_BUFFER_UNSYNC(self, gcmd):
        if not self.synced:
            gcmd.respond_info("buffer: already unsynced")
            return
        if self._is_printing():
            # PRINT_END still counts as printing; after an M400 the queue is
            # empty and unsyncing can't add a stop
            self._require_empty_queue(gcmd, "BUFFER_UNSYNC")
        self._do_sync(False)
        self._motor_off()
        gcmd.respond_info("buffer: unsynced")

    cmd_BUFFER_MOVE_help = (
        "Move the buffer motor on its own: DIST=<mm> [SPEED=]. Feeding stops"
        " early at pos3. Not while printing."
    )

    def cmd_BUFFER_MOVE(self, gcmd):
        self._require_not_printing(gcmd, "BUFFER_MOVE")
        dist = gcmd.get_float("DIST")
        speed = gcmd.get_float(
            "SPEED", self.button_speed, above=0.0, maxval=self.max_move_speed
        )
        if abs(dist) > JOG_MAX_MM:
            raise gcmd.error("BUFFER_MOVE: DIST limited to +/-%.0f mm" % JOG_MAX_MM)
        moved, at_pos3 = self._idle_move(
            dist, speed, endstop_key="pos3" if dist > 0.0 else None
        )
        if at_pos3:
            gcmd.respond_info(
                "buffer: stopped at pos3 after %.1f of %.1f mm" % (moved, dist)
            )

    cmd_BUFFER_LOAD_help = (
        "Feed filament from the inlet until the slider reaches pos2, which puts"
        " the tip against the extruder gears: [SPEED=] [MAX=]. TO=nozzle then"
        " feeds it through the hotend and purges: [PURGE=30] [TEMP=] (park the"
        " nozzle over a purge spot first). A buffer button cancels. Not while"
        " printing."
    )

    def cmd_BUFFER_LOAD(self, gcmd):
        self._require_not_printing(gcmd, "BUFFER_LOAD")
        measure = self.fresh_insert or bool(gcmd.get_int("MEASURE", 0, minval=0, maxval=1))
        self.fresh_insert = False
        to = gcmd.get("TO", "gears").lower()
        if to not in ("gears", "nozzle"):
            raise gcmd.error("BUFFER_LOAD: TO must be gears or nozzle")
        if self.synced:
            raise gcmd.error("BUFFER_LOAD: the buffer is synced; BUFFER_UNSYNC first")
        if not self.pin_states.get("inlet"):
            raise gcmd.error("BUFFER_LOAD: no filament at the buffer inlet")
        at_gears = self.pin_states.get("pos2") or self.pin_states.get("pos3")
        if at_gears:
            gcmd.respond_info("buffer: the tip is at the extruder gears (slider at pos2)")
            if to == "gears":
                return
        self.keep_motor = True
        try:
            if (at_gears or self._load_to_gears(gcmd, measure)) and to == "nozzle":
                self._load_to_nozzle(gcmd)
        finally:
            self.keep_motor = False
            self._motor_off()

    def _load_to_gears(self, gcmd, measure):
        speed = gcmd.get_float(
            "SPEED", self.load_speed, above=0.0, maxval=self.max_move_speed
        )
        max_mm = gcmd.get_float("MAX", self.load_max_mm, above=0.0, maxval=JOG_MAX_MM)
        at_rest = self.pin_states.get("pos1")
        self.loading, self.load_cancel = True, False
        self.inlet_clear_since = None
        start = self.reactor.monotonic()
        try:
            # slowly first, so the gear catches filament still being pushed in
            grab = min(self.load_grab_mm, max_mm)
            moved, hit = self._idle_move(
                grab, self.load_grab_speed, endstop_key="pos2", abort=self._load_abort
            )
            if not hit and not self._load_abort() and max_mm > moved:
                more, hit = self._idle_move(
                    max_mm - moved, speed, endstop_key="pos2", abort=self._load_abort
                )
                moved += more
        finally:
            self.loading = False
        secs = self.reactor.monotonic() - start
        if hit:
            self.last_load_mm = moved
            gcmd.respond_info(
                "buffer: loaded, pos2 reached after %.0f mm (%.0f s). The tip is at"
                " the extruder gears." % (moved, secs)
            )
            # From an empty path the slider starts relaxed, so everything fed
            # beyond the slack it now holds is the path itself
            path = moved - self.pos2_slack_mm
            if measure and at_rest and path >= MIN_PATH_MM:
                if self.path_mm and abs(path - self.path_mm) > PATH_AGREE_MM:
                    # e.g. a broken piece still in the tube ahead of the new tip
                    gcmd.respond_info(
                        "buffer: this load says the path is about %.0f mm, but it's"
                        " known to be %.0f mm. Something was already in the tube"
                        " ahead of the tip, so it wasn't saved" % (path, self.path_mm)
                    )
                else:
                    gcmd.respond_info(
                        "buffer: path from the inlet to the extruder gears is about"
                        " %.0f mm" % path
                    )
                    self._remember(gcmd, "path", path)
            return True
        if self.load_cancel:
            gcmd.respond_info("buffer: load cancelled after %.0f mm" % moved)
            return False
        if not self.pin_states.get("inlet"):
            raise gcmd.error(
                "BUFFER_LOAD: stopped after %.0f mm, the filament left the inlet" % moved
            )
        else:
            raise gcmd.error(
                "BUFFER_LOAD: pos2 not reached after %.0f mm. The buffer gear may not"
                " have gripped the filament, or the path is longer than"
                " load_max_mm." % moved
            )

    def _heat_for(self, gcmd, what):
        temp = gcmd.get_float("TEMP", None, minval=0.0, maxval=400.0)
        if temp is not None:
            gcmd.respond_info("buffer: heating to %.0f C" % temp)
            self.gcode.run_script_from_command("M109 S%.1f" % temp)
        if not self.extruder.get_heater().can_extrude:
            raise gcmd.error("%s: hotend too cold to extrude; heat it or pass TEMP=" % what)

    def _extrude_checked(self, gcmd, total, speed, what):
        """Synced extrusion in 10 mm pieces, stopping if the slider says the
        extruder isn't taking the filament or the buffer can't keep up."""
        run = self.gcode.run_script_from_command
        done = 0.0
        while done < total - 1e-6:
            piece = min(10.0, total - done)
            run("G1 E%.3f F%.1f\nM400" % (piece, speed * 60.0))
            done += piece
            problem = self._test_check()
            if problem:
                raise gcmd.error("%s: stopped after %.0f mm, %s" % (what, done, problem))

    def _load_to_nozzle(self, gcmd):
        """The tip is at the extruder gears. Grab it slowly, feed the
        gears-to-nozzle length plus a margin, then purge. Feeding too far
        only purges a little more; the margin covers the estimate."""
        what = "BUFFER_LOAD TO=nozzle"
        purge = gcmd.get_float("PURGE", 30.0, minval=0.0, maxval=300.0)
        self._heat_for(gcmd, what)
        nozzle = self.nozzle_mm or DEFAULT_NOZZLE_MM
        if self.nozzle_mm is None:
            gcmd.respond_info(
                "buffer: gears-to-nozzle length not measured yet, assuming %.0f mm"
                " (BUFFER_UNLOAD measures it)" % nozzle
            )
        run = self.gcode.run_script_from_command
        run("SAVE_GCODE_STATE NAME=_buffer_load\nM83\nM400")
        self._do_sync(True)
        try:
            self._extrude_checked(gcmd, 10.0, 2.0, what)
            self._extrude_checked(gcmd, nozzle + NOZZLE_MARGIN_MM - 10.0, 5.0, what)
            if purge > 0.0:
                self._extrude_checked(gcmd, purge, 3.0, what)
        finally:
            self._do_sync(False)
            run("RESTORE_GCODE_STATE NAME=_buffer_load")
        gcmd.respond_info(
            "buffer: loaded to the nozzle (%.0f mm through the hotend, %.0f mm purged)"
            % (nozzle + NOZZLE_MARGIN_MM, purge)
        )

    # --- unloading -------------------------------------------------------
    cmd_BUFFER_UNLOAD_help = (
        "Unload: retract the filament out of the hotend and extruder with the"
        " slider relaxed, so nothing presses the tip into the gears, prove the"
        " tip is free, then pull it back to the buffer. Hotend hot or TEMP=."
        " EJECT=1 pulls it out past the buffer gear. [MAX=] [TEST=] [PARK=]"
        " [PATH=]. Not while printing."
    )

    def cmd_BUFFER_UNLOAD(self, gcmd):
        self._require_not_printing(gcmd, "BUFFER_UNLOAD")
        eject = bool(gcmd.get_int("EJECT", 0, minval=0, maxval=1))
        test = gcmd.get_float("TEST", self.unload_test_mm, above=self.pos1_slack_mm)
        max_mm = gcmd.get_float(
            "MAX", (self.nozzle_mm or DEFAULT_NOZZLE_MM) + self.unload_overrun_mm,
            above=self.unload_fast_mm, maxval=500.0,
        )
        park = gcmd.get_float("PARK", self.park_mm, above=0.0)
        path = gcmd.get_float("PATH", self.path_mm, above=0.0)
        if self.synced:
            raise gcmd.error("BUFFER_UNLOAD: the buffer is synced; BUFFER_UNSYNC first")
        if not self.pin_states.get("inlet"):
            gcmd.respond_info(
                "buffer: no filament at the inlet, unloading what is left in the path"
            )
        self._heat_for(gcmd, "BUFFER_UNLOAD")
        self.unloading = self.keep_motor = True
        # filament the buffer sends back out of the inlet toward the spool,
        # which doesn't turn by itself: everything it retracts minus what it feeds
        back = 0.0
        try:
            # 1. Contact: the slider at the lower edge of pos2, a known slack
            back -= self._contact(gcmd, path)
            # 2. Relax: nothing may press on the filament at the gears
            relax = self.pos2_slack_mm - RELAX_LEAVE_MM
            self._idle_move(-relax, self.load_grab_speed)
            back += relax
            # 3. Retract with the buffer following, a hair faster
            self._retract_relaxed(max_mm)
            back += self.ctrl.trim * UNLOAD_FOLLOW * (max_mm - self.unload_ram_mm)
            # 4. The free test
            self._free_test(gcmd, test)
            back -= test
            # 5. Contact again. The feed it takes says where the tip was, so
            # the park needs no estimate, and how far the buffer carried the
            # tip past the gears gives the gears-to-nozzle length
            fed = self._contact(gcmd, path)
            back -= fed
            above = fed - self.pos2_slack_mm + test
            nozzle = max_mm - above / UNLOAD_FOLLOW
            if MIN_NOZZLE_MM <= nozzle <= max_mm - 5.0:
                gcmd.respond_info(
                    "buffer: extruder gears to nozzle is about %.0f mm" % nozzle
                )
                self._remember(gcmd, "nozzle", nozzle)
            elif nozzle < MIN_NOZZLE_MM:
                gcmd.respond_info("buffer: the filament wasn't through the hotend")
            # 6. Pull back from contact: the slack first, then the tip
            if path is None:
                pull = self.pos2_slack_mm + 100.0
                where = (
                    "the tip is free and about 100 mm above the extruder gears. The"
                    " path length isn't known yet (autoload measures it, or pass"
                    " PATH=), so it was not pulled further"
                )
            elif eject:
                pull = self.pos2_slack_mm + path + self.eject_margin_mm
                where = "the tip is out past the buffer gear"
            else:
                pull = self.pos2_slack_mm + path - park
                where = "the tip is parked about %.0f mm past the buffer inlet" % park
            pull = min(pull, JOG_MAX_MM)
            self._idle_move(-pull, self.load_speed)
            back += pull
            gcmd.respond_info(
                "buffer: unloaded, %s. About %.0f mm of filament went back out of"
                " the inlet toward the spool, which doesn't turn by itself: wind it"
                " back before loading again, or it tangles" % (where, back)
            )
        finally:
            self.unloading = self.keep_motor = False
            self._motor_off()

    def _contact(self, gcmd, path):
        """Bring the slider to the lower edge of pos2 against whatever holds the
        far end (the extruder, or a tip resting on the gears), so the slack it
        holds is known. Returns the filament fed to get there."""
        fed = 0.0
        if self.pin_states.get("pos2") or self.pin_states.get("pos3"):
            # past the edge: ease back until pos2 clears, then come up to it
            moved, _ = self._idle_move(
                -15.0, 5.0, abort=lambda: not self.pin_states.get("pos2")
            )
            fed += moved
            self._settle()
        moved, hit = self._idle_move(
            self.pos2_slack_mm + 20.0, self.load_grab_speed, endstop_key="pos2"
        )
        fed += moved
        if not hit:
            limit = (path + 50.0) if path else self.load_max_mm
            more, hit = self._idle_move(
                max(0.0, limit - fed), self.load_speed, endstop_key="pos2"
            )
            fed += more
        if not hit:
            raise gcmd.error(
                "BUFFER_UNLOAD: fed %.0f mm and the slider never compressed, so the"
                " filament never reached anything. Is it in the buffer gear?" % fed
            )
        return fed

    def _retract_relaxed(self, max_mm):
        """Synced at a fixed multiplier a hair above 1, so the slider stays
        relaxed. While the extruder holds the filament it sets the pace. Once
        the tip is out of the gears the buffer carries it up and away instead
        of a compressed spring pressing it back into gears that are still
        turning. One continuous retraction, fast out of the hot zone, then
        steady, so the soft tip never waits in the heatbreak."""
        run = self.gcode.run_script_from_command
        fast = min(self.unload_fast_mm, max_mm)
        run("SAVE_GCODE_STATE NAME=_buffer_unload\nM83\nM400")
        self._do_sync(True)
        self._set_follow(UNLOAD_FOLLOW)
        try:
            if self.unload_ram_mm > 0.0:
                # a little forward first, so the tip leaves from fresh melt
                run("G1 E%.3f F300\nM400" % self.unload_ram_mm)
            run(
                "G1 E-%.3f F%.1f\nG1 E-%.3f F%.1f\nM400"
                % (fast, self.unload_fast_speed * 60.0,
                   max_mm - fast, self.unload_speed * 60.0)
            )
        finally:
            self._do_sync(False)
            run("RESTORE_GCODE_STATE NAME=_buffer_unload")

    def _free_test(self, gcmd, test, what="BUFFER_UNLOAD"):
        """A free tip slides forward without compressing the slider. One held
        above the gears (a swollen tip at the PTFE, say) compresses it past
        the top of pos1. Checked before any long pull."""
        moved, hit = self._idle_move(test, self.load_grab_speed, endstop_key="pos2")
        self._settle()
        if hit or not self.pin_states.get("pos1"):
            raise gcmd.error(
                "%s: the tip is not free. Feeding %.0f mm compressed the slider, so"
                " something still holds it: a tip that won't go up past the gears,"
                " or one still in the extruder. Stopped before the long pull"
                % (what, test)
            )

    # --- calibration -----------------------------------------------------
    cmd_BUFFER_CALIBRATE_help = (
        "Measure the buffer's true rotation_distance against the extruder and"
        " apply it. Needs filament loaded through the extruder and a hot"
        " hotend (or TEMP=); extrudes about 15 mm per run plus a warm-up run."
        " [RUNS=3] [SPEED=4]"
        " [EXTRUDE_SPEED=2] [TEMP=]"
    )

    def _buffer_mm_at(self, eventtime):
        pt = self.mcu.estimated_print_time(eventtime)
        steps = self.mcu_stepper.get_past_mcu_position(pt)
        return steps * self.mcu_stepper.get_step_dist()

    def _edge_positions(self, log, position_at, label):
        """Position of the last edge of each (sensor, state) in an edge log."""
        edges = {}
        for key, state, et in log:
            edges[(key, state)] = pos = position_at(et)
            logging.info("filament_buffer: calibrate %s %s=%d at %.4f -> %.3f mm",
                         label, key, state, et, pos)
        return edges

    def _calibration_run(self, gcmd, speed, espeed, max_mm):
        run = self.gcode.run_script_from_command
        # Start at the top of the pos1 zone
        if not self.pin_states.get("pos1"):
            self._idle_move(-max_mm, speed, abort=lambda: self.pin_states.get("pos1"))
            self._settle()
            if not self.pin_states.get("pos1"):
                raise gcmd.error("BUFFER_CALIBRATE: the slider did not return to pos1")
        # Up: the buffer feeds while the extruder holds the filament
        self.edge_log = []
        moved, hit = self._idle_move(max_mm, speed, endstop_key="pos3")
        if hit:
            # a little past the edge, so the slider settling can't clear pos3
            self._idle_move(1.0, speed)
        self._settle()
        up, self.edge_log = self.edge_log, None
        if not hit:
            raise gcmd.error(
                "BUFFER_CALIBRATE: pos3 not reached after %.0f mm of buffer feed. Is"
                " the filament loaded through the extruder?" % moved
            )
        # Down: the extruder pulls while the buffer holds
        self.edge_log = []
        done = 0.0
        while not self.pin_states.get("pos1"):
            if done >= max_mm:
                self.edge_log = None
                raise gcmd.error(
                    "BUFFER_CALIBRATE: pos1 not reached after extruding %.0f mm" % done
                )
            run("G1 E0.5 F%.1f\nM400" % (espeed * 60.0))
            done += 0.5
        self._settle()
        down, self.edge_log = self.edge_log, None
        try:
            return calibration_result(
                self._edge_positions(up, self._buffer_mm_at, "buffer"),
                self._edge_positions(down, self._extruder_pos, "extruder"),
            )
        except (KeyError, ValueError):
            raise gcmd.error(
                "BUFFER_CALIBRATE: unexpected sensor sequence (up %s, down %s)"
                % ([(k, s) for k, s, t in up], [(k, s) for k, s, t in down])
            )

    def cmd_BUFFER_CALIBRATE(self, gcmd):
        self._require_not_printing(gcmd, "BUFFER_CALIBRATE")
        # the buffer has to hold while the extruder pulls, so it stays powered
        self.keep_motor = True
        try:
            self._calibrate(gcmd)
        finally:
            self.keep_motor = False
            self._motor_off()

    def _calibrate(self, gcmd):
        runs = gcmd.get_int("RUNS", 3, minval=1, maxval=10)
        speed = gcmd.get_float("SPEED", 4.0, above=0.0, maxval=20.0)
        espeed = gcmd.get_float("EXTRUDE_SPEED", 2.0, above=0.0, maxval=10.0)
        max_mm = gcmd.get_float("MAX", 60.0, minval=10.0, maxval=200.0)
        temp = gcmd.get_float("TEMP", None, minval=0.0, maxval=320.0)
        if not self.pin_states.get("inlet"):
            raise gcmd.error("BUFFER_CALIBRATE: no filament at the buffer inlet")
        if temp is not None:
            self.gcode.run_script_from_command("M109 S%.1f" % temp)
        if not self.extruder.get_heater().can_extrude:
            raise gcmd.error(
                "BUFFER_CALIBRATE: hotend too cold to extrude; heat it or pass TEMP="
            )
        run = self.gcode.run_script_from_command
        was_synced = self.synced
        if was_synced:
            self._do_sync(False)
        results = []
        # the extruder must hold the filament while the buffer pushes
        run("SET_STEPPER_ENABLE STEPPER=%s ENABLE=1" % self.extruder_name)
        run("SAVE_GCODE_STATE NAME=_buffer_cal\nM83")
        try:
            # The first cycle only conditions the path: after the motor has
            # been off the slider may sit deep in pos1 with slack in the tube
            self._calibration_run(gcmd, speed, espeed, max_mm)
            gcmd.respond_info("buffer calibrate: warm-up cycle done")
            for i in range(runs):
                ratio, geo = self._calibration_run(gcmd, speed, espeed, max_mm)
                results.append((ratio, geo))
                gcmd.respond_info(
                    "buffer calibrate %d/%d: %.4f mm per commanded mm, pos1-pos3"
                    " span %.2f mm" % (i + 1, runs, ratio, geo["span_mm"])
                )
        except Exception:
            self.edge_log = None
            run("RESTORE_GCODE_STATE NAME=_buffer_cal")
            if was_synced:
                self._do_sync(True)
            raise
        run("RESTORE_GCODE_STATE NAME=_buffer_cal")
        ratios = sorted(r for r, g in results)
        median = ratios[len(ratios) // 2]
        spread = (ratios[-1] - ratios[0]) / median
        if spread > CALIBRATE_SPREAD:
            if was_synced:
                self._do_sync(True)
            raise gcmd.error(
                "BUFFER_CALIBRATE: runs disagree by %.1f%% (gear slipping?);"
                " nothing changed" % (spread * 100.0)
            )
        old_rd = self.base_rd
        self.base_rd = old_rd * median
        self._reset_rate()
        if was_synced:
            self._do_sync(True)
        geo = results[-1][1]
        lines = [
            "rotation_distance %.4f -> %.4f (applied until restart; put"
            " rotation_distance: %.3f in [extruder_stepper %s])"
            % (old_rd, self.base_rd, self.base_rd, self.stepper_name),
            "spread between runs %.2f%%" % (spread * 100.0),
        ]
        if "gap_mm" in geo:
            lines.append(
                "slider: %.1f mm from leaving pos1 to pos2, %.1f mm from pos2 to"
                " pos3 (band_mm), pos2 %s pos3"
                % (geo["gap_mm"], geo["band_mm"],
                   "overlaps" if geo["overlap"] else "is separate from")
            )
        gcmd.respond_info("\n".join("buffer calibrate: " + l for l in lines))

    cmd_BUFFER_SET_help = (
        "Change buffer tuning at runtime: [M_POS1=] [M_APPROACH_BELOW=]"
        " [M_BELOW=] [M_TARGET=] [M_ABOVE=] [M_APPROACH_ABOVE=] [M_POS3=]"
        " [TENSION_FAULT_MM=] [COMPRESSION_FAULT_MM=] [TRIM=] [BAND_MM=]"
        " [REPORT_EVENTS=0|1] [ROTATION_DISTANCE=] (not while printing)"
    )

    def cmd_BUFFER_SET(self, gcmd):
        c = self.ctrl
        rd = gcmd.get_float("ROTATION_DISTANCE", None, above=0.0)
        if rd is not None:
            self._require_not_printing(gcmd, "BUFFER_SET ROTATION_DISTANCE")
            self.base_rd = rd
            if self.synced:
                self.mcu_stepper.set_rotation_distance(self.base_rd / self.applied_mult)
            else:
                self._reset_rate()
        c.band_mm = gcmd.get_float("BAND_MM", c.band_mm, minval=0.0)
        c.set_multipliers(
            m_pos1=gcmd.get_float("M_POS1", None, above=1.0),
            m_approach_below=gcmd.get_float("M_APPROACH_BELOW", None, above=1.0),
            m_below=gcmd.get_float("M_BELOW", None, above=1.0),
            m_target=gcmd.get_float("M_TARGET", None, above=0.0, below=1.0),
            m_above=gcmd.get_float("M_ABOVE", None, above=0.0, below=1.0),
            m_approach_above=gcmd.get_float(
                "M_APPROACH_ABOVE", None, above=0.0, below=1.0
            ),
            m_pos3=gcmd.get_float("M_POS3", None, above=0.0, below=1.0),
        )
        c.tension_fault_mm = gcmd.get_float(
            "TENSION_FAULT_MM", c.tension_fault_mm, above=0.0
        )
        c.compression_fault_mm = gcmd.get_float(
            "COMPRESSION_FAULT_MM", c.compression_fault_mm, above=0.0
        )
        trim = gcmd.get_float("TRIM", None, above=0.5, below=1.5)
        if trim is not None:
            c.trim = trim
        self.report_events = bool(
            gcmd.get_int("REPORT_EVENTS", int(self.report_events), minval=0, maxval=1)
        )
        self._apply_rate(self.reactor.monotonic())
        self.cmd_BUFFER_STATUS(gcmd)

    # --- statistics ------------------------------------------------------
    def _reset_stats(self):
        self.ctrl.reset_stats(self._extruder_pos(self.reactor.monotonic()))
        self.rate_changes_at_reset = self.rate_changes

    def _stats_lines(self):
        st = self.ctrl.stats
        if st["start_e"] is None:
            return ["no statistics yet (BUFFER_STATS RESET=1 starts them)"]
        order = (ZONE_POS1, ZONE_BELOW, ZONE_POS2, ZONE_ABOVE, ZONE_POS3, ZONE_UNKNOWN)
        total = sum(st["zone_mm"].values())
        rc = self.rate_changes - self.rate_changes_at_reset
        lines = [
            "%.1f mm extruded since reset | %d rate changes (%.1f per 100 mm)"
            % (total, rc, 100.0 * rc / total if total > 0 else 0.0),
            "time in zone: "
            + " | ".join(
                "%s %.1f%%" % (z, 100.0 * st["zone_mm"][z] / total if total > 0 else 0.0)
                for z in order
            ),
            "zone entries: "
            + " ".join("%s=%d" % (z, st["zone_entries"][z]) for z in order),
            "first reached pos2 after: %s | trim %.4f -> %.4f (%d learning updates)"
            % (
                "%.1f mm" % st["first_pos2_mm"]
                if st["first_pos2_mm"] is not None
                else "not yet",
                st["trim_start"],
                self.ctrl.trim,
                self.ctrl.trim_updates - st["trim_updates_start"],
            ),
        ]
        return lines

    cmd_BUFFER_STATS_help = "Buffer statistics since the last reset: [RESET=1]"

    def cmd_BUFFER_STATS(self, gcmd):
        if gcmd.get_int("RESET", 0, minval=0, maxval=1):
            self._reset_stats()
            gcmd.respond_info("buffer: statistics reset")
            return
        gcmd.respond_info("\n".join("buffer: " + l for l in self._stats_lines()))

    # --- automated synced-extrusion test -----------------------------------
    cmd_BUFFER_TEST_EXTRUDE_help = (
        "Automated synced-extrusion test. Needs: filament loaded through the"
        " buffer into the extruder, printer homed with the nozzle >= MIN_Z"
        " above the bed (over a cup), hotend hot or TEMP=. Options: LENGTH=300"
        " SEGMENT=10 SPEEDS=1.5,3,5 RETRACT=1.0 RETRACT_SPEED=35 TRAVEL_MS=300"
        " CHECK_EVERY=25 MIN_Z=20 TEMP= DRY_RUN=1. CHECK_Z=0 skips the homing"
        " and height check when you know the nozzle is clear of the bed."
    )

    def _test_check(self):
        """Abort conditions, checked in real time between test segments."""
        c = self.ctrl
        if not self.pin_states.get("inlet"):
            return "filament left the buffer inlet"
        if c.sensors["pos1"] and c.sensors["pos3"]:
            return "impossible sensor state (pos1 and pos3 both blocked)"
        e_pos = self._extruder_pos(self.reactor.monotonic())
        net = e_pos - (c.zone_entry_e if c.zone_entry_e is not None else e_pos)
        if c.zone == ZONE_POS3 and net > TEST_POS3_ABORT_MM:
            return (
                "slider held at pos3 for %.0f mm: the extruder is not consuming"
                " filament (not loaded through to the nozzle, or nozzle blocked)" % net
            )
        if c.zone == ZONE_POS1 and net > TEST_POS1_ABORT_MM:
            return (
                "slider held at pos1 for %.0f mm: the buffer is not keeping up"
                " (gear not gripping, tangle, or motor)" % net
            )
        return None

    def _test_verdict(self, aborted):
        if aborted:
            return ["RESULT: ABORTED - " + aborted]
        st = self.ctrl.stats
        first = st["first_pos2_mm"]
        if first is None:
            return ["RESULT: CHECK - the slider never reached pos2"]
        after = {
            z: st["zone_mm"][z] - st["zone_mm_at_pos2"][z] for z in st["zone_mm"]
        }
        entries = {
            z: st["zone_entries"][z] - st["zone_entries_at_pos2"][z]
            for z in st["zone_entries"]
        }
        total_after = sum(after.values())
        if total_after < 20.0:
            return [
                "RESULT: CHECK - reached pos2 after %.0f mm but too little"
                " extrusion after that to judge (use a larger LENGTH)" % first
            ]
        hard = after[ZONE_POS1] + after[ZONE_POS3]
        held = after[ZONE_POS2] + after[ZONE_BELOW] + after[ZONE_ABOVE]
        lines = [
            "after reaching pos2: %.0f mm extruded, %.1f%% at pos2 or just beside it,"
            " %d pos3 entries, %d pos1 entries"
            % (total_after, 100.0 * held / total_after, entries[ZONE_POS3], entries[ZONE_POS1])
        ]
        ok = first <= 200.0 and hard / total_after < 0.05 and entries[ZONE_POS3] == 0
        lines.append(
            "RESULT: %s"
            % (
                "PASS"
                if ok
                else "CHECK - see the numbers above (expected: pos2 within 200 mm,"
                " no pos3 entries, <5%% at pos1/pos3)"
            )
        )
        return lines

    def cmd_BUFFER_TEST_EXTRUDE(self, gcmd):
        self._require_not_printing(gcmd, "BUFFER_TEST_EXTRUDE")
        length = gcmd.get_float("LENGTH", 300.0, above=0.0, maxval=2000.0)
        segment = gcmd.get_float("SEGMENT", 10.0, minval=1.0, maxval=50.0)
        retract = gcmd.get_float("RETRACT", 1.0, minval=0.0, maxval=2.0)
        rspeed = gcmd.get_float("RETRACT_SPEED", 35.0, above=0.0, maxval=60.0)
        travel_ms = gcmd.get_int("TRAVEL_MS", 300, minval=0, maxval=5000)
        check_every = gcmd.get_float("CHECK_EVERY", 25.0, minval=5.0)
        min_z = gcmd.get_float("MIN_Z", 20.0, minval=0.0)
        check_z = gcmd.get_int("CHECK_Z", 1, minval=0, maxval=1)
        temp = gcmd.get_float("TEMP", None, minval=0.0, maxval=320.0)
        dry_run = gcmd.get_int("DRY_RUN", 0, minval=0, maxval=1)
        try:
            speeds = [float(s) for s in gcmd.get("SPEEDS", "1.5,3,5").split(",")]
        except ValueError:
            raise gcmd.error("SPEEDS must be a comma separated list of mm/s")
        if not speeds or min(speeds) <= 0.0 or max(speeds) > 12.0:
            raise gcmd.error("SPEEDS must be within 0 < speed <= 12 mm/s")
        nseg = max(1, int(math.ceil(length / len(speeds) / segment)))
        plan = [sp for sp in speeds for _ in range(nseg)]
        est = sum(segment / sp for sp in plan)
        if retract > 0.0:
            est += len(plan) * (2.0 * retract / rspeed + travel_ms / 1000.0)
        summary = (
            "%d segments of %.1f mm (%.0f mm) at %s mm/s, %.1f mm retraction at"
            " %.0f mm/s after each, checks every %.0f mm, ~%.0f s"
            % (
                len(plan),
                segment,
                len(plan) * segment,
                "/".join("%g" % s for s in speeds),
                retract,
                rspeed,
                check_every,
                est,
            )
        )
        if dry_run:
            gcmd.respond_info("buffer test (dry run, nothing moves): " + summary)
            return
        # Preconditions - refuse rather than guess
        eventtime = self.reactor.monotonic()
        if not self.pin_states.get("inlet"):
            raise gcmd.error("BUFFER_TEST_EXTRUDE: no filament at the buffer inlet")
        if not check_z:
            gcmd.respond_info("buffer test: CHECK_Z=0, not checking the nozzle height")
        elif "z" not in self.toolhead.get_status(eventtime)["homed_axes"]:
            raise gcmd.error(
                "BUFFER_TEST_EXTRUDE: home the printer first (Z must be known), or pass"
                " CHECK_Z=0 if the nozzle is clear of the bed"
            )
        elif self.toolhead.get_position()[2] < min_z:
            raise gcmd.error(
                "BUFFER_TEST_EXTRUDE: raise the nozzle to Z >= %.0f first (over a cup)" % min_z
            )
        if temp is not None:
            gcmd.respond_info("buffer test: heating to %.0f C" % temp)
            self.gcode.run_script_from_command("M109 S%.1f" % temp)
        if not self.extruder.get_heater().can_extrude:
            raise gcmd.error(
                "BUFFER_TEST_EXTRUDE: hotend too cold to extrude - heat it or pass TEMP="
            )
        run = self.gcode.run_script_from_command
        gcmd.respond_info("buffer test: " + summary)
        run("SAVE_GCODE_STATE NAME=_buffer_test\nM83\nM400")
        was_synced = self.synced
        if not was_synced:
            self._do_sync(True)
        self._reset_stats()
        aborted = None
        done, next_check = 0.0, check_every
        try:
            for sp in plan:
                run("G1 E%.3f F%.1f" % (segment, sp * 60.0))
                if retract > 0.0:
                    run(
                        "G1 E-%.3f F%.1f\nG4 P%d\nG1 E%.3f F%.1f"
                        % (retract, rspeed * 60.0, travel_ms, retract, rspeed * 60.0)
                    )
                done += segment
                if done >= next_check:
                    next_check += check_every
                    run("M400")
                    aborted = self._test_check()
                    if aborted:
                        break
                    gcmd.respond_info(
                        "buffer test: %.0f/%.0f mm | zone %s x%.3f | trim %.4f"
                        % (done, len(plan) * segment, self.ctrl.zone,
                           self.ctrl.multiplier(), self.ctrl.trim)
                    )
            run("M400")
            if aborted is None:
                aborted = self._test_check()
        finally:
            lines = self._stats_lines() + self._test_verdict(aborted)
            if not was_synced and self.synced:
                self._do_sync(False)
                self._motor_off()
            run("RESTORE_GCODE_STATE NAME=_buffer_test")
        gcmd.respond_info("\n".join("buffer test: " + l for l in lines))

    # --- path slack and feed speed tests ----------------------------------
    cmd_BUFFER_TEST_SLACK_help = (
        "Measure the slack in the filament path: with the extruder holding the"
        " filament, drive the slider from pos1 to pos3 and back CYCLES times and"
        " report where each sensor trips each way. [CYCLES=5] [SPEED=3]. Needs"
        " filament loaded through the extruder. Not while printing."
    )

    def _last_edges(self, log):
        edges = {}
        for key, state, et in log:
            edges[(key, state)] = self._buffer_mm_at(et)
        return edges

    def cmd_BUFFER_TEST_SLACK(self, gcmd):
        self._require_not_printing(gcmd, "BUFFER_TEST_SLACK")
        cycles = gcmd.get_int("CYCLES", 5, minval=1, maxval=20)
        speed = gcmd.get_float("SPEED", 3.0, above=0.0, maxval=20.0)
        if self.synced:
            raise gcmd.error("BUFFER_TEST_SLACK: the buffer is synced; BUFFER_UNSYNC first")
        if not self.pin_states.get("inlet"):
            raise gcmd.error("BUFFER_TEST_SLACK: no filament at the buffer inlet")
        run = self.gcode.run_script_from_command
        # the extruder gears hold the far end still
        run("SET_STEPPER_ENABLE STEPPER=%s ENABLE=1" % self.extruder_name)
        at_pos1 = lambda: self.pin_states.get("pos1")
        runs = []
        self.keep_motor = True
        try:
            # start a little below the top of pos1, still compressed
            if not at_pos1():
                self._idle_move(-40.0, speed, abort=at_pos1)
            self._idle_move(-3.0, speed)
            self._settle()
            if not at_pos1():
                raise gcmd.error("BUFFER_TEST_SLACK: the slider did not come down to pos1")
            for i in range(cycles):
                self.edge_log = []
                moved, hit = self._idle_move(40.0, speed, endstop_key="pos3")
                if not hit:
                    raise gcmd.error(
                        "BUFFER_TEST_SLACK: pos3 not reached after %.0f mm. Is the"
                        " filament loaded through the extruder?" % moved
                    )
                self._idle_move(1.0, speed)
                self._settle()
                up, self.edge_log = self.edge_log, []
                self._idle_move(-40.0, speed, abort=at_pos1)
                self._idle_move(-3.0, speed)
                self._settle()
                down, self.edge_log = self.edge_log, None
                try:
                    runs.append(slack_result(self._last_edges(up), self._last_edges(down)))
                except KeyError:
                    raise gcmd.error(
                        "BUFFER_TEST_SLACK: unexpected sensor sequence (up %s, down %s)"
                        % ([(k, s) for k, s, t in up], [(k, s) for k, s, t in down])
                    )
        finally:
            self.edge_log = None
            self.keep_motor = False
            self._motor_off()
        lines = ["%d cycles at %.1f mm/s, buffer mm (mean, sd):" % (len(runs), speed)]
        for name in ("pos1", "pos2", "pos3"):
            vals = [d[name] for d, su, sd_ in runs if name in d]
            if vals:
                lines.append(
                    "dead band at %s: %.2f, %.2f (up minus down)" % ((name,) + _mean_sd(vals))
                )
        lines.append("span pos1 to pos3 going up: %.2f, %.2f" % _mean_sd([su for d, su, sd_ in runs]))
        lines.append("span pos3 to pos1 coming down: %.2f, %.2f" % _mean_sd([sd_ for d, su, sd_ in runs]))
        gcmd.respond_info("\n".join("buffer slack: " + l for l in lines))

    cmd_BUFFER_TEST_SPEED_help = (
        "Find how fast the buffer feeds without skipping: round trips from contact"
        " at the extruder gears, out DIST and back, at each speed. A skipping"
        " motor comes back short or long. [SPEEDS=30,45,60,80,100] [DIST=80]"
        " [CYCLES=2]. Needs the tip free above the gears with DIST of room (after"
        " a short BUFFER_UNLOAD). Each trip sends DIST toward the spool and takes"
        " it back. Not while printing."
    )

    def cmd_BUFFER_TEST_SPEED(self, gcmd):
        self._require_not_printing(gcmd, "BUFFER_TEST_SPEED")
        dist = gcmd.get_float("DIST", 80.0, minval=10.0, maxval=300.0)
        cycles = gcmd.get_int("CYCLES", 2, minval=1, maxval=10)
        try:
            speeds = [float(v) for v in gcmd.get("SPEEDS", "30,45,60,80,100").split(",")]
        except ValueError:
            raise gcmd.error("SPEEDS must be a comma separated list of mm/s")
        if not speeds or min(speeds) <= 0.0 or max(speeds) > TEST_SPEED_MAX:
            raise gcmd.error("SPEEDS must be within 0 < speed <= %.0f mm/s" % TEST_SPEED_MAX)
        if self.synced:
            raise gcmd.error("BUFFER_TEST_SPEED: the buffer is synced; BUFFER_UNSYNC first")
        if not self.pin_states.get("inlet"):
            raise gcmd.error("BUFFER_TEST_SPEED: no filament at the buffer inlet")
        self.gcode.run_script_from_command(
            "SET_STEPPER_ENABLE STEPPER=%s ENABLE=1" % self.extruder_name
        )
        trip = self.pos2_slack_mm + dist
        lines = []
        self.keep_motor = True
        try:
            # the tip must be free: a short pull and the free test bound any
            # slip to a few cm if it's still in the extruder after all
            self._contact(gcmd, self.path_mm)
            test = self.unload_test_mm
            self._idle_move(-(self.pos2_slack_mm + test + 3.0), self.load_grab_speed)
            self._free_test(gcmd, test, "BUFFER_TEST_SPEED")
            self._contact(gcmd, self.path_mm)
            for v in speeds:
                errs = []
                for i in range(cycles):
                    self._idle_move(-trip, v)
                    moved, hit = self._idle_move(trip + 40.0, v, endstop_key="pos2")
                    errs.append(moved - trip if hit else None)
                    if not hit or abs(moved - trip) > 10.0:
                        break
                ok = [e for e in errs if e is not None]
                lines.append(
                    "%5.0f mm/s: back at contact %s"
                    % (v, ", ".join("%+.1f mm" % e if e is not None else "never" for e in errs))
                )
                if len(ok) < len(errs) or any(abs(e) > 10.0 for e in ok):
                    lines.append("stopped: the motor lost steps at %.0f mm/s" % v)
                    break
        finally:
            self.keep_motor = False
            self._motor_off()
        gcmd.respond_info(
            "\n".join(
                ["buffer speed: round trips of %.0f mm (contact to %.0f mm up and back)" % (trip, dist)]
                + ["buffer speed: " + l for l in lines]
            )
        )

    def get_status(self, eventtime):
        c = self.ctrl
        status = {
            "synced": self.synced,
            "zone": c.zone,
            "multiplier": c.multiplier(),
            "trim": c.trim,
            "applied_multiplier": self.applied_mult,
            "fault": c.fault or "",
            "faults_armed": c.faults_enabled,
            "base_rotation_distance": self.base_rd or 0.0,
            "loading": self.loading,
            "unloading": self.unloading,
            "last_load_mm": self.last_load_mm or 0.0,
            "path_mm": self.path_mm or 0.0,
            "nozzle_mm": self.nozzle_mm or 0.0,
        }
        status.update(self.pin_states)
        return status


def load_config(config):
    return FilamentBuffer(config)
