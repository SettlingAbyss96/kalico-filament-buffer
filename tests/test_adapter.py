"""Tests for the Kalico adapter (FilamentBuffer) using stand-in printer objects.

The key test is the hard rule: while printing, nothing the plugin does may
touch the toolhead's motion queue (flush_step_generation, get_last_move_time,
wait_moves, independent moves). Only the buffer's rotation distance may change.
Independent moves go through BufferMover, replaced here by FakeMover; the real
one is exercised on the printer.

Run from the repo root:  python -m unittest discover -s tests -v
"""
import contextlib
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import filament_buffer as fb  # noqa: E402

REQUIRED = object()


class CommandError(Exception):
    pass


class FakeReactor:
    NOW = 0.0
    NEVER = 9e99

    def __init__(self):
        self.now = 100.0
        self.callbacks = []

    def monotonic(self):
        return self.now

    def register_timer(self, callback, waketime=None):
        return callback

    def update_timer(self, timer, waketime):
        pass

    def register_callback(self, callback, waketime=None):
        self.callbacks.append(callback)

    def pause(self, waketime):
        pass


class FakeGcode:
    def __init__(self):
        self.commands = {}
        self.responses = []
        self.scripts = []

    def register_command(self, name, func, desc=None):
        self.commands[name] = func

    def respond_info(self, msg):
        self.responses.append(msg)

    def respond_raw(self, msg):
        self.responses.append(msg)

    def run_script(self, script):
        self.scripts.append(script)

    def run_script_from_command(self, script):
        self.scripts.append(script)

    def get_mutex(self):
        return contextlib.nullcontext()


class FakeButtons:
    def __init__(self):
        self.pins = {}

    def register_buttons(self, pins, callback):
        self.pins[pins[0]] = callback


class FakeEndstop:
    def __init__(self, pin):
        self.pin = pin
        self.steppers = []
        self.triggered = False

    def add_stepper(self, stepper):
        self.steppers.append(stepper)

    def query_endstop(self, print_time):
        return self.triggered


class FakePins:
    def __init__(self):
        self.multi_use = set()
        self.endstops = {}

    def allow_multi_use_pin(self, pin):
        self.multi_use.add(pin)

    def setup_pin(self, pin_type, pin):
        assert pin_type == "endstop"
        assert pin.lstrip("^~! ") in self.multi_use, "shared without allow_multi_use_pin"
        self.endstops[pin] = FakeEndstop(pin)
        return self.endstops[pin]


class FakeMover:
    """Stands in for BufferMover: records each independent move."""

    def __init__(self, printer, config, stepper):
        self.stepper = stepper
        self.moves = []
        self.results = []  # scripted (moved, triggered); default full move

    def move(self, dist, speed, accel, endstop=None, name="sensor", abort=None):
        self.moves.append(
            {"dist": dist, "speed": speed, "endstop": name if endstop else None,
             "abort": abort}
        )
        if self.results:
            return self.results.pop(0)
        return dist, False


fb.BufferMover = FakeMover


class FakeLookahead:
    def __init__(self):
        self.last = None

    def get_last(self):
        return self.last


class FakeToolhead:
    """Records every call that would touch the motion queue."""

    def __init__(self):
        self.lookahead = FakeLookahead()
        self.calls = []
        self.homed_axes = "xyz"

    def flush_step_generation(self):
        self.calls.append("flush_step_generation")

    def get_last_move_time(self):
        self.calls.append("get_last_move_time")
        return 0.0

    def wait_moves(self):
        self.calls.append("wait_moves")

    def get_status(self, eventtime):
        return {"homed_axes": self.homed_axes}

    def get_position(self):
        return [0.0, 0.0, 50.0, 0.0]


class FakeStepper:
    def __init__(self, rd):
        self.rd = rd
        self.rd_history = []

    def get_rotation_distance(self):
        return self.rd, 200

    def set_rotation_distance(self, rd):
        self.rd = rd
        self.rd_history.append(rd)


class FakeExtruderStepper:
    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.stepper = FakeStepper(6.3)
        self.motion_queue = None

    def sync_to_extruder(self, name):
        self.toolhead.flush_step_generation()  # the real one flushes too
        self.motion_queue = name or None


class FakeHeater:
    can_extrude = True


class FakeExtruder:
    def __init__(self):
        self.e = 0.0

    def find_past_position(self, print_time):
        return self.e

    def get_heater(self):
        return FakeHeater()


class FakeMcu:
    def estimated_print_time(self, eventtime):
        return eventtime


class FakePrintStats:
    def __init__(self):
        self.state = "standby"

    def get_status(self, eventtime):
        return {"state": self.state}


class FakePauseResume:
    def __init__(self):
        self.paused = False
        self.pause_commands = 0

    def get_status(self, eventtime):
        return {"is_paused": self.paused}

    def send_pause_command(self):
        self.pause_commands += 1


class FakeGcrq:
    def __init__(self):
        self.values = []

    def send_async_request(self, value, print_time=None):
        self.values.append(value)


class FakeOutputPin:
    def __init__(self):
        self.gcrq = FakeGcrq()


class FakePrinter:
    def __init__(self):
        self.reactor = FakeReactor()
        self.toolhead = FakeToolhead()
        self.objects = {
            "gcode": FakeGcode(),
            "buttons": FakeButtons(),
            "toolhead": self.toolhead,
            "mcu": FakeMcu(),
            "extruder": FakeExtruder(),
            "print_stats": FakePrintStats(),
            "pause_resume": FakePauseResume(),
            "pins": FakePins(),
            "output_pin buffer_led_run": FakeOutputPin(),
            "output_pin buffer_led_err": FakeOutputPin(),
        }
        es_wrapper = type("PES", (), {})()
        es_wrapper.extruder_stepper = FakeExtruderStepper(self.toolhead)
        self.objects["extruder_stepper buffer"] = es_wrapper
        self.handlers = {}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=REQUIRED):
        if name in self.objects:
            return self.objects[name]
        if default is REQUIRED:
            raise KeyError(name)
        return default

    def load_object(self, config, name):
        return self.objects.setdefault(name, object())

    def register_event_handler(self, event, callback):
        self.handlers.setdefault(event, []).append(callback)

    def send_event(self, event):
        for cb in self.handlers.get(event, []):
            cb()


class FakeConfig:
    error = CommandError

    def __init__(self, printer, values):
        self.printer = printer
        self.values = values

    def get_printer(self):
        return self.printer

    def get_name(self):
        return "filament_buffer"

    def get(self, option, default=REQUIRED):
        if option in self.values:
            return self.values[option]
        if default is REQUIRED:
            raise CommandError("missing option " + option)
        return default

    def getfloat(self, option, default=REQUIRED, **kw):
        val = self.get(option, default)
        return None if val is None else float(val)

    def getint(self, option, default=REQUIRED, **kw):
        val = self.get(option, default)
        return None if val is None else int(val)

    def getboolean(self, option, default=REQUIRED):
        val = self.get(option, default)
        if isinstance(val, str):
            return val.lower() in ("1", "true", "yes")
        return bool(val)


class FakeGcmd:
    error = CommandError

    def __init__(self, **params):
        self.params = {k.upper(): str(v) for k, v in params.items()}
        self.responses = []

    def get(self, name, default=REQUIRED):
        if name in self.params:
            return self.params[name]
        if default is REQUIRED:
            raise CommandError("missing " + name)
        return default

    def get_float(self, name, default=REQUIRED, **kw):
        val = self.get(name, default)
        return None if val is None else float(val)

    def get_int(self, name, default=REQUIRED, **kw):
        val = self.get(name, default)
        return None if val is None else int(val)

    def respond_info(self, msg):
        self.responses.append(msg)


BASE_CONFIG = {
    "extruder_stepper": "buffer",
    "extruder": "extruder",
    "pos1_pin": "buffer:PB4",
    "pos2_pin": "buffer:PB3",
    "pos3_pin": "buffer:PB2",
    "inlet_pin": "!buffer:PB7",
    "feed_button_pin": "!buffer:PB12",
    "retract_button_pin": "!buffer:PB13",
    "ok_led": "buffer_led_run",
    "fault_led": "buffer_led_err",
    "report_events": "False",
}


def make_buffer(extra=None):
    printer = FakePrinter()
    values = dict(BASE_CONFIG)
    values.update(extra or {})
    buf = fb.FilamentBuffer(FakeConfig(printer, values))
    printer.send_event("klippy:connect")
    printer.send_event("klippy:ready")
    return printer, buf


class TestConfig(unittest.TestCase):
    def test_config_defaults_match_the_simulated_controller(self):
        _, buf = make_buffer()
        ref = fb.FeedController()
        for attr in (
            "m_pos1", "m_approach_below", "m_below", "m_target", "m_above",
            "m_approach_above", "m_pos3", "tension_fault_mm",
            "compression_fault_mm", "trim_limit", "trim_gain", "trim_nudge",
            "hover_stall_mm", "debounce_mm", "up_stay_flip_mm", "overlap", "band_mm",
        ):
            self.assertEqual(getattr(buf.ctrl, attr), getattr(ref, attr), attr)

    def test_commands_and_pins_registered(self):
        printer, _ = make_buffer()
        cmds = printer.objects["gcode"].commands
        for name in ("BUFFER_STATUS", "BUFFER_STATS", "BUFFER_SYNC", "BUFFER_UNSYNC",
                     "BUFFER_MOVE", "BUFFER_LOAD", "BUFFER_CALIBRATE", "BUFFER_SET",
                     "BUFFER_TEST_EXTRUDE"):
            self.assertIn(name, cmds)
        self.assertEqual(len(printer.objects["buttons"].pins), 6)

    def test_pos2_and_pos3_double_as_endstops_on_the_buffer_motor(self):
        printer, buf = make_buffer()
        endstops = printer.objects["pins"].endstops
        self.assertEqual(sorted(endstops), ["buffer:PB2", "buffer:PB3"])
        for es in endstops.values():
            self.assertEqual(es.steppers, [buf.mcu_stepper])

    def test_bad_sensor_layout_is_a_config_error(self):
        with self.assertRaises(CommandError):
            make_buffer({"sensor_layout": "sideways"})

    def test_missing_required_pin_is_an_error(self):
        printer = FakePrinter()
        values = dict(BASE_CONFIG)
        del values["pos2_pin"]
        with self.assertRaises(CommandError):
            fb.FilamentBuffer(FakeConfig(printer, values))


class TestHardRule(unittest.TestCase):
    def setUp(self):
        self.printer, self.buf = make_buffer()
        self.toolhead = self.printer.toolhead
        self.pins = self.printer.objects["buttons"].pins
        self.extruder = self.printer.objects["extruder"]
        self.stepper = self.buf.mcu_stepper

    def press(self, pin, state):
        self.printer.reactor.now += 0.05
        self.pins[pin](self.printer.reactor.now, state)

    def tick(self, de):
        self.extruder.e += de
        self.printer.reactor.now += 0.1
        self.buf._update(self.printer.reactor.now)

    def test_mid_print_actions_never_touch_the_toolhead(self):
        # Sync while idle (allowed - the queue is empty), then start printing
        self.press("!buffer:PB7", 1)  # filament present
        self.press("buffer:PB4", 1)  # slider at rest (pos1)
        self.printer.objects["gcode"].commands["BUFFER_SYNC"](FakeGcmd())
        self.printer.objects["print_stats"].state = "printing"
        self.toolhead.lookahead.last = object()  # moves are queued now
        self.toolhead.calls.clear()
        rd_changes_before = len(self.stepper.rd_history)
        # A print: extrusion with retractions while the slider climbs to pos2
        # and hovers around its lower edge
        for i in range(400):
            self.tick(0.6)
            if i % 7 == 0:
                self.tick(-1.0)
                self.tick(1.0)
            if i == 10:
                self.press("buffer:PB4", 0)
            if i in (60, 140, 220, 300):
                self.press("buffer:PB3", 1)
            if i in (100, 180, 260, 340):
                self.press("buffer:PB3", 0)
        self.assertEqual(self.toolhead.calls, [], "the plugin touched the motion queue mid-print")
        self.assertGreater(len(self.stepper.rd_history), rd_changes_before,
                           "expected rate changes during the print")
        self.assertEqual(self.buf.mover.moves, [])

    def test_commands_that_would_stop_the_toolhead_are_refused_while_printing(self):
        cmds = self.printer.objects["gcode"].commands
        self.printer.objects["print_stats"].state = "printing"
        self.toolhead.lookahead.last = object()
        for name, params in (("BUFFER_MOVE", {"dist": 10}), ("BUFFER_UNSYNC", {}),
                             ("BUFFER_SYNC", {}), ("BUFFER_TEST_EXTRUDE", {"dry_run": 1}),
                             ("BUFFER_LOAD", {}), ("BUFFER_CALIBRATE", {}),
                             ("BUFFER_SET", {"rotation_distance": 6.3})):
            # UNSYNC only does anything when synced; SYNC only when unsynced
            self.buf.synced = name == "BUFFER_UNSYNC"
            with self.assertRaises(CommandError, msg=name):
                cmds[name](FakeGcmd(**params))
        self.assertEqual(self.toolhead.calls, [])
        self.assertEqual(self.buf.mover.moves, [])
        # Already synced: BUFFER_SYNC is a no-op and must not touch the toolhead
        self.buf.synced = True
        cmds["BUFFER_SYNC"](FakeGcmd())
        self.assertEqual(self.toolhead.calls, [])

    def test_unsync_at_print_end_needs_the_toolhead_stopped(self):
        cmds = self.printer.objects["gcode"].commands
        cmds["BUFFER_SYNC"](FakeGcmd())
        self.printer.objects["print_stats"].state = "printing"
        self.toolhead.lookahead.last = object()
        with self.assertRaises(CommandError):
            cmds["BUFFER_UNSYNC"](FakeGcmd())
        self.toolhead.lookahead.last = None  # after M400 in PRINT_END
        cmds["BUFFER_UNSYNC"](FakeGcmd())
        self.assertFalse(self.buf.synced)

    def test_buttons_ignored_while_printing(self):
        self.printer.objects["print_stats"].state = "printing"
        self.press("!buffer:PB12", 1)
        self.assertIsNone(self.buf.button_held)
        self.assertEqual(self.printer.reactor.callbacks, [])

    def test_no_autoload_while_printing_or_paused(self):
        self.press("buffer:PB4", 1)  # slider at rest: empty path
        for state in ("printing", "paused"):
            self.printer.objects["print_stats"].state = state
            self.press("!buffer:PB7", 1)
            self.press("!buffer:PB7", 0)
        self.assertEqual(self.printer.reactor.callbacks, [])

    def test_fault_pauses_only_while_printing(self):
        self.press("!buffer:PB7", 1)
        self.printer.objects["gcode"].commands["BUFFER_SYNC"](FakeGcmd())
        # not printing: runout is not a fault
        self.press("!buffer:PB7", 0)
        self.assertEqual(self.printer.reactor.callbacks, [])
        # printing: runout pauses (via a reactor callback, like Kalico's runout sensor)
        self.press("!buffer:PB7", 1)
        self.printer.objects["print_stats"].state = "printing"
        self.tick(1.0)
        self.press("!buffer:PB7", 0)
        self.assertEqual(len(self.printer.reactor.callbacks), 1)


class TestIdleMoves(unittest.TestCase):
    def setUp(self):
        self.printer, self.buf = make_buffer()
        self.pins = self.printer.objects["buttons"].pins
        self.cmds = self.printer.objects["gcode"].commands
        self.mover = self.buf.mover

    def press(self, pin, state):
        self.printer.reactor.now += 0.05
        self.pins[pin](self.printer.reactor.now, state)

    def run_callbacks(self):
        callbacks, self.printer.reactor.callbacks = self.printer.reactor.callbacks, []
        for cb in callbacks:
            cb(self.printer.reactor.now)

    def test_inserting_filament_into_an_empty_path_starts_a_load(self):
        self.press("buffer:PB4", 1)  # slider at rest
        self.press("!buffer:PB7", 1)  # filament inserted
        self.assertEqual(len(self.printer.reactor.callbacks), 1)
        self.run_callbacks()
        self.assertIn("BUFFER_LOAD", self.printer.objects["gcode"].scripts)

    def test_no_autoload_when_the_path_is_not_empty_or_disabled(self):
        self.press("!buffer:PB7", 1)  # pos1 not blocked: filament already in
        self.assertEqual(self.printer.reactor.callbacks, [])
        printer, buf = make_buffer({"autoload": "False"})
        pins = printer.objects["buttons"].pins
        pins["buffer:PB4"](100.1, 1)
        pins["!buffer:PB7"](100.2, 1)
        self.assertEqual(printer.reactor.callbacks, [])

    def test_load_feeds_slowly_then_fast_and_stops_at_pos2(self):
        self.press("buffer:PB4", 1)
        self.press("!buffer:PB7", 1)
        self.mover.results = [(20.0, False), (845.0, True)]
        gcmd = FakeGcmd()
        self.cmds["BUFFER_LOAD"](gcmd)
        first, second = self.mover.moves
        self.assertEqual((first["endstop"], second["endstop"]), ("pos2", "pos2"))
        self.assertLess(first["speed"], second["speed"])
        self.assertIn("865 mm", gcmd.responses[-1])
        self.assertEqual(self.buf.last_load_mm, 865.0)

    def test_load_refusals(self):
        with self.assertRaises(CommandError):
            self.cmds["BUFFER_LOAD"](FakeGcmd())  # no filament at the inlet
        self.press("!buffer:PB7", 1)
        self.buf.synced = True
        with self.assertRaises(CommandError):
            self.cmds["BUFFER_LOAD"](FakeGcmd())
        self.buf.synced = False
        self.mover.results = [(20.0, False), (1480.0, False)]
        with self.assertRaises(CommandError):  # pos2 never reached
            self.cmds["BUFFER_LOAD"](FakeGcmd())

    def test_a_button_cancels_a_load(self):
        self.press("!buffer:PB7", 1)
        self.buf.loading = True
        self.press("!buffer:PB13", 1)
        self.assertTrue(self.buf.load_cancel)
        self.assertTrue(self.buf._load_abort())
        self.assertEqual(self.printer.reactor.callbacks, [])

    def test_feed_button_moves_until_released_and_stops_at_pos3(self):
        self.press("!buffer:PB12", 1)
        self.run_callbacks()
        (move,) = self.mover.moves
        self.assertGreater(move["dist"], 0.0)
        self.assertEqual(move["endstop"], "pos3")
        self.assertFalse(move["abort"]())
        self.press("!buffer:PB12", 0)
        self.assertTrue(move["abort"]())

    def test_retract_button_has_no_endstop(self):
        self.press("!buffer:PB13", 1)
        self.run_callbacks()
        (move,) = self.mover.moves
        self.assertLess(move["dist"], 0.0)
        self.assertIsNone(move["endstop"])

    def test_buffer_move_feeding_stops_at_pos3(self):
        self.cmds["BUFFER_MOVE"](FakeGcmd(dist=30))
        self.cmds["BUFFER_MOVE"](FakeGcmd(dist=-30))
        self.assertEqual([m["endstop"] for m in self.mover.moves], ["pos3", None])

    def test_set_rotation_distance_when_idle(self):
        self.cmds["BUFFER_SET"](FakeGcmd(rotation_distance=6.3))
        self.assertEqual(self.buf.base_rd, 6.3)
        self.assertEqual(self.buf.mcu_stepper.rd, 6.3)

    def test_extrude_test_requires_homing_unless_check_z_0(self):
        self.press("!buffer:PB7", 1)
        self.printer.toolhead.homed_axes = ""
        with self.assertRaises(CommandError) as ctx:
            self.cmds["BUFFER_TEST_EXTRUDE"](FakeGcmd())
        self.assertIn("CHECK_Z=0", str(ctx.exception))
        gcmd = FakeGcmd(check_z=0, length=30)
        self.cmds["BUFFER_TEST_EXTRUDE"](gcmd)
        self.assertTrue(any("RESULT" in r for r in gcmd.responses))


class TestCalibrationMath(unittest.TestCase):
    def test_ratio_and_geometry_from_edges(self):
        # numbers from the measurement on the real buffer (2026-10-04)
        up = {("pos1", False): 0.75, ("pos2", True): 21.75, ("pos3", True): 31.75}
        down = {("pos3", False): 0.12, ("pos2", False): 4.38, ("pos1", True): 14.12}
        ratio, geo = fb.calibration_result(up, down)
        self.assertAlmostEqual(ratio, 14.0 / 31.0)
        self.assertAlmostEqual(geo["gap_mm"], 21.0 * ratio)
        self.assertAlmostEqual(geo["band_mm"], 10.0 * ratio)
        self.assertTrue(geo["overlap"])

    def test_edges_out_of_order_are_rejected(self):
        up = {("pos1", False): 5.0, ("pos3", True): 4.0}
        down = {("pos3", False): 0.0, ("pos1", True): 10.0}
        with self.assertRaises(ValueError):
            fb.calibration_result(up, down)


if __name__ == "__main__":
    unittest.main()
