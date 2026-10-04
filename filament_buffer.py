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
import logging
import math

UPDATE_INTERVAL = 0.1  # seconds between extruder position samples
DIRECTION_WINDOW = 0.3  # seconds of extruder history used for direction
DIRECTION_DEADBAND = 0.05  # mm of net extruder motion treated as "stopped"
MIN_RATE_CHANGE_INTERVAL = 0.2  # seconds between applied rate changes
TEST_POS3_ABORT_MM = 15.0  # BUFFER_TEST_EXTRUDE: held at pos3 this long -> abort
TEST_POS1_ABORT_MM = 40.0  # BUFFER_TEST_EXTRUDE: held at pos1 this long -> abort

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
        tension_fault_mm=25.0,
        compression_fault_mm=15.0,
        trim_limit=0.05,
        trim_gain=0.5,
        trim_nudge=0.01,
        hover_stall_mm=60.0,
        debounce_mm=0.3,
        up_stay_flip_mm=300.0,
    ):
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

    # --- state -----------------------------------------------------------
    def multiplier(self):
        z = self.zone
        if z == ZONE_POS1:
            return self.m_pos1
        if z == ZONE_POS3:
            return self.m_pos3
        if z == ZONE_POS2:
            return self.m_target
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
        if s["pos1"] and s["pos3"]:
            zone = ZONE_UNKNOWN
        elif s["pos1"]:
            zone = ZONE_POS1
        elif s["pos3"]:
            zone = ZONE_POS3
        elif s["pos2"]:
            zone = ZONE_POS2
        else:
            zone = ZONE_UNKNOWN
        self._enter(zone, e_pos, approach=False, learn=False)
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
        if learn:
            # Hard sensor hits: correct the belief, and the trim only when
            # the correct-side hover rate failed to hold the slider
            if zone == ZONE_POS3:
                if prev == ZONE_ABOVE:
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
            # Auto-trim from clean hover cycles at either edge of pos2
            if prev == ZONE_POS2 and zone in (ZONE_BELOW, ZONE_ABOVE):
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
        self.zone = zone
        self.approach = approach
        self.zone_entry_e = e_pos

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
                zone = ZONE_POS2
            else:
                zone, approach = ZONE_ABOVE, True
        elif state:  # pos2 rising
            zone = ZONE_POS2
        elif s["pos1"]:
            zone = ZONE_POS1
        elif s["pos3"]:
            zone = ZONE_POS3
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
        self.button_step = config.getfloat("button_step", 10.0, above=0.0)
        self.button_speed = config.getfloat("button_speed", 20.0, above=0.0)
        self.max_move_speed = config.getfloat("max_move_speed", 60.0, above=0.0)
        self.move_accel = config.getfloat("move_accel", 500.0, minval=0.0)
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
        self.ok_led_name = config.get("ok_led", None)
        self.fault_led_name = config.get("fault_led", None)
        self.printer.load_object(config, "pause_resume")
        self.printer.load_object(config, "force_move")
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
        self.toolhead = self.extruder = self.mcu = None
        self.es = self.mcu_stepper = None
        self.base_rd = None
        self.print_stats = self.pause_resume = self.force_move = None
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
            "BUFFER_SET",
            "BUFFER_TEST_EXTRUDE",
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
        self.force_move = self.printer.lookup_object("force_move")
        for led in (self.ok_led_name, self.fault_led_name):
            if led:
                self.leds[led] = self.printer.lookup_object("output_pin " + led)
        self.synced = bool(self.es.motion_queue)

    def _handle_ready(self):
        # extruder_stepper applies its configured sync on connect; read the
        # result here so config section order can't matter
        self.synced = bool(self.es.motion_queue)
        self._update_leds()
        self.reactor.update_timer(self.update_timer, self.reactor.NOW)

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
        if sync:
            e_pos = self._extruder_pos(self.reactor.monotonic())
            self.ctrl.reset_zone(e_pos)
            self._apply_rate(self.reactor.monotonic(), force=True)

    # --- events (reactor context: never raise) ---------------------------
    def _pin_event(self, key, eventtime, state):
        try:
            self.pin_states[key] = bool(state)
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

    # --- buttons (only when not printing) --------------------------------
    def _button(self, key, state):
        if not state:
            if self.button_held == key:
                self.button_held = None
            return
        if self._is_printing():
            self._respond("%s ignored while printing" % key)
            return
        self.button_held = key
        if not self.button_loop_running:
            self.button_loop_running = True
            self.reactor.register_callback(self._button_loop)

    def _button_loop(self, eventtime):
        try:
            while self.button_held is not None and not self._is_printing():
                step = self.button_step
                if self.button_held == "key_retract":
                    step = -step
                self.gcode.run_script(
                    "BUFFER_MOVE DIST=%.3f SPEED=%.3f" % (step, self.button_speed)
                )
        except Exception:
            logging.exception("filament_buffer: error in button move")
        finally:
            self.button_loop_running = False

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

    cmd_BUFFER_UNSYNC_help = "Unsync the buffer motor from the extruder (not while printing)"

    def cmd_BUFFER_UNSYNC(self, gcmd):
        if not self.synced:
            gcmd.respond_info("buffer: already unsynced")
            return
        self._require_not_printing(gcmd, "BUFFER_UNSYNC")
        self._do_sync(False)
        gcmd.respond_info("buffer: unsynced")

    cmd_BUFFER_MOVE_help = (
        "Move the buffer motor on its own: DIST=<mm> [SPEED=] [ACCEL=]."
        " Not while printing."
    )

    def cmd_BUFFER_MOVE(self, gcmd):
        self._require_not_printing(gcmd, "BUFFER_MOVE")
        dist = gcmd.get_float("DIST")
        speed = gcmd.get_float(
            "SPEED", self.button_speed, above=0.0, maxval=self.max_move_speed
        )
        accel = gcmd.get_float("ACCEL", self.move_accel, minval=0.0)
        if abs(dist) > 2000.0:
            raise gcmd.error("BUFFER_MOVE: DIST limited to +/-2000 mm")
        was_synced = self.synced
        if was_synced:
            self._do_sync(False)
        try:
            self.force_move.manual_move(self.mcu_stepper, dist, speed, accel)
        finally:
            if was_synced:
                self._do_sync(True)

    cmd_BUFFER_SET_help = (
        "Change buffer tuning at runtime: [M_POS1=] [M_APPROACH_BELOW=]"
        " [M_BELOW=] [M_TARGET=] [M_ABOVE=] [M_APPROACH_ABOVE=] [M_POS3=]"
        " [TENSION_FAULT_MM=] [COMPRESSION_FAULT_MM=] [TRIM=] [REPORT_EVENTS=0|1]"
    )

    def cmd_BUFFER_SET(self, gcmd):
        c = self.ctrl
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
        " CHECK_EVERY=25 MIN_Z=20 TEMP= DRY_RUN=1"
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
        if "z" not in self.toolhead.get_status(eventtime)["homed_axes"]:
            raise gcmd.error("BUFFER_TEST_EXTRUDE: home the printer first (Z must be known)")
        if self.toolhead.get_position()[2] < min_z:
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
            run("RESTORE_GCODE_STATE NAME=_buffer_test")
        gcmd.respond_info("\n".join("buffer test: " + l for l in lines))

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
        }
        status.update(self.pin_states)
        return status


def load_config(config):
    return FilamentBuffer(config)
