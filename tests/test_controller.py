"""Offline test suite for the filament buffer FeedController.

A physical model of the buffer slider drives the real controller code:
- x = slider position in filament-mm (0 = pos1 rest end, grows as the spring
  compresses). Feeding more than the extruder consumes raises x.
- Hall sensors are windows on x with hysteresis. The default geometry is the
  one measured on a Mellow LLL Buffer Plus: pos1 blocked over the first 19 mm,
  a 9.6 mm gap, then pos2 blocked from 28.6 mm up past pos3 (33.0 mm), which
  stays blocked to the end stop.
- The buffer motor is synced to the commanded extruder motion; rate changes
  take effect after a latency (Kalico step-generation window).
Run from the repo root:  python -m unittest discover -s tests -v
"""
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from filament_buffer import (  # noqa: E402
    FeedController,
    ZONE_POS1,
    ZONE_POS2,
    ZONE_POS3,
)


class Sensor:
    """Blocked while lo <= x <= hi; once blocked, clears only beyond +/- hyst."""

    def __init__(self, lo, hi, hyst):
        self.lo, self.hi, self.hyst = lo, hi, hyst
        self.state = False

    def update(self, x):
        if self.state:
            new = (self.lo - self.hyst) <= x <= (self.hi + self.hyst)
        else:
            new = self.lo <= x <= self.hi
        changed = new != self.state
        self.state = new
        return changed


# (pos1 top, pos2 bottom, pos2 top, pos3 bottom) in filament mm, and end stop
LLL_PLUS = ((19.0, 28.6, 36.6, 33.0), 40.0)  # measured; pos2 overlaps pos3
SEPARATE = ((2.0, 13.0, 15.0, 26.0), 30.0)  # three separate sensor windows


class BufferSim:
    def __init__(
        self,
        ctrl,
        ratio_err=0.0,
        x0=0.0,
        latency=0.25,
        dt=0.005,
        faults=True,
        x_max=LLL_PLUS[1],
        geometry=LLL_PLUS[0],
        hyst=0.3,
        x_noise=0.0,
        seed=0,
    ):
        self.ctrl = ctrl
        self.ratio_err = ratio_err
        self.latency = latency
        self.dt = dt
        self.x_max = x_max
        self.x_noise = x_noise
        self.rnd = random.Random(seed)
        p1, p2a, p2b, p3 = geometry
        self.p1_top, self.p2_lo, self.p3_lo = p1, p2a, p3
        # where a settled slider should be: around the lower edge of pos2
        self.hover_band = (p2a - 2.0, min(p2a + 3.0, p3 - 0.5))
        self.sensors = {
            "pos1": Sensor(-1e9, p1, hyst),
            "pos2": Sensor(p2a, p2b, hyst),
            "pos3": Sensor(p3, 1e9, hyst),
        }
        self.x = x0
        self.E = 0.0
        self.t = 0.0
        self.hist = [(0.0, 0.0)]
        self.slip = False
        self.clog = False
        self.fault = None
        self.fault_E = None
        self.rate_changes = 0
        self.max_x = x0
        self.ground_mm = 0.0  # mm pushed against the hard end (grinding)
        # Tug: the buffer pulling filament back harder than the extruder
        # releases it while the spring is fully relaxed (slack already 0).
        self.tug_total = 0.0
        self.tug_max = 0.0
        self._tug_run = 0.0
        self.samples = []  # (E, x) every progress tick
        self.retract_moves = []  # (E_start, x_before, x_after) per retraction
        for name, s in self.sensors.items():
            s.update(x0)
            ctrl.set_raw_state(name, s.state)
        ctrl.reset_zone(0.0)
        ctrl.set_faults_enabled(faults, 0.0)
        self.target = ctrl.effective()
        self.applied = self.target
        self.pending = []
        self.next_tick = 0.1

    def _direction(self):
        t_past = self.t - 0.3
        e_past = self.hist[0][1]
        for t, e in reversed(self.hist):
            if t <= t_past:
                e_past = e
                break
        diff = self.E - e_past
        return 1 if diff > 0.05 else (-1 if diff < -0.05 else 0)

    def _note_rate(self):
        m = self.ctrl.effective()
        if abs(m - self.target) > 1e-9:
            self.target = m
            self.pending.append((self.t + self.latency, m))
            self.rate_changes += 1

    def run(self, v, duration):
        steps = max(1, int(round(duration / self.dt)))
        x_before = self.x
        e_before = self.E
        for _ in range(steps):
            self.step(v)
            if self.fault:
                return
        if v < 0.0:
            self.retract_moves.append((e_before, x_before, self.x))

    def step(self, v):
        dE = v * self.dt
        self.E += dE
        self.t += self.dt
        self.hist.append((self.t, self.E))
        if len(self.hist) > 400:
            del self.hist[:200]
        while self.pending and self.pending[0][0] <= self.t:
            self.applied = self.pending.pop(0)[1]
        feed = 0.0 if self.slip else self.applied * (1.0 + self.ratio_err) * dE
        consume = 0.0 if self.clog else dE
        x = self.x + feed - consume
        if x > self.x_max:
            self.ground_mm += x - self.x_max
        if x < 0.0:
            self._tug_run += -x
            self.tug_total += -x
            self.tug_max = max(self.tug_max, self._tug_run)
        else:
            self._tug_run = 0.0
        self.x = min(self.x_max, max(0.0, x))
        self.max_x = max(self.max_x, self.x)
        x_seen = self.x + (self.rnd.gauss(0.0, self.x_noise) if self.x_noise else 0.0)
        for name in ("pos1", "pos2", "pos3"):
            s = self.sensors[name]
            if s.update(x_seen):
                self.ctrl.on_sensor(name, s.state, self._direction(), self.E)
                self._note_rate()
                msg = self.ctrl.take_new_fault()
                if msg and not self.fault:
                    self.fault, self.fault_E = msg, self.E
        if self.t >= self.next_tick:
            self.next_tick += 0.1
            msg = self.ctrl.on_progress(self.E)
            self._note_rate()
            self.samples.append((self.E, self.x))
            if msg and not self.fault:
                self.fault, self.fault_E = msg, self.E


def print_profile(total_mm, seed, max_flow=12.0, max_retract=1.0):
    """Random print: extrusion segments separated by travels. Retractions
    are 0.2..max_retract mm at 25..45 mm/s in several real-world patterns:
    retract-travel-unretract, layer change with a z-hop pause, quick
    back-to-back retractions, and the occasional extra prime."""
    rnd = random.Random(seed)
    E = 0.0
    while E < total_mm:
        flow = rnd.uniform(0.5, max_flow)
        dur = rnd.uniform(0.3, 8.0)
        yield flow, dur
        E += flow * dur
        r = rnd.random()
        if r < 0.6:
            n = 3 if rnd.random() < 0.1 else 1  # sometimes rapid repeats
            for _ in range(n):
                dist = rnd.uniform(0.2, max_retract)
                speed = rnd.uniform(25.0, 45.0)
                yield -speed, dist / speed
                yield 0.0, rnd.uniform(0.02, 2.0)  # travel / z-hop
                prime = 0.02 if rnd.random() < 0.1 else 0.0
                yield speed, (dist + prime) / speed
                if n > 1:
                    yield rnd.uniform(0.5, max_flow), rnd.uniform(0.05, 0.3)
        elif r < 0.7:
            # layer change: retract, longer pause, unretract
            dist = max_retract
            yield -35.0, dist / 35.0
            yield 0.0, rnd.uniform(1.0, 6.0)
            yield 35.0, dist / 35.0


def run_print(sim, total_mm=3000.0, seed=1, until=None, max_flow=12.0, max_retract=1.0):
    for v, dur in print_profile(total_mm, seed, max_flow, max_retract):
        if until is not None and sim.E >= until:
            return
        sim.run(v, dur)
        if sim.fault:
            return


def new_ctrl(**kw):
    return FeedController(**kw)


def separate_sim(ctrl_kw=None, **kw):
    """Sim and controller for the separate-windows sensor layout."""
    ctrl = new_ctrl(overlap=False, **(ctrl_kw or {}))
    return BufferSim(ctrl, geometry=SEPARATE[0], x_max=SEPARATE[1], **kw)


class TestZoneLogic(unittest.TestCase):
    def test_debounce_ignores_flicker_and_keeps_edge_position(self):
        c = new_ctrl()  # debounce 0.3 mm of extruder travel
        c.reset_zone(0.0)
        c.on_progress(0.0)
        c.on_sensor("pos2", True, +1, 10.0)
        c.on_sensor("pos2", False, +1, 10.05)  # flicker: never counts
        c.on_progress(11.0)
        self.assertEqual(c.zone, "below")  # nothing blocked: the pos1-pos2 gap
        c.on_sensor("pos2", True, +1, 12.0)
        c.on_progress(12.1)  # only 0.1 mm since the edge: not yet
        self.assertEqual(c.zone, "below")
        c.on_progress(12.5)
        self.assertEqual(c.zone, ZONE_POS2)
        self.assertEqual(c.zone_entry_e, 12.0)  # timed from the real edge

    def test_pos2_exit_side_inferred_from_multiplier(self):
        c = new_ctrl(debounce_mm=0.0, overlap=False)
        c.set_raw_state("pos2", True)
        c.reset_zone(0.0)
        self.assertEqual(c.zone, ZONE_POS2)
        # inside pos2 m<1: extruding forward means the slider drifts down
        c.on_sensor("pos2", False, +1, 10.0)
        self.assertEqual(c.zone, "below")
        c.on_sensor("pos2", True, +1, 20.0)
        # while retracting the relative motion flips
        c.on_sensor("pos2", False, -1, 21.0)
        self.assertEqual(c.zone, "above")

    def test_overlap_pos2_exit_is_always_below(self):
        c = new_ctrl(debounce_mm=0.0)
        c.set_raw_state("pos2", True)
        c.reset_zone(0.0)
        c.drift_up = True  # a belief that would say "above" without overlap
        c.on_sensor("pos2", False, -1, 10.0)
        self.assertEqual(c.zone, "below")
        self.assertFalse(c.approach)

    def test_overlap_descent_from_pos3_relieves_until_pos2_edge(self):
        c = new_ctrl(debounce_mm=0.0)
        c.set_raw_state("pos2", True)
        c.set_raw_state("pos3", True)
        c.reset_zone(0.0)
        self.assertEqual(c.zone, ZONE_POS3)
        c.on_sensor("pos3", False, +1, 5.0)
        self.assertEqual((c.zone, c.approach), (ZONE_POS2, True))
        self.assertEqual(c.multiplier(), c.m_approach_above)
        c.on_sensor("pos2", False, +1, 30.0)
        self.assertEqual((c.zone, c.approach), ("below", False))
        self.assertEqual(c.multiplier(), c.m_below)
        self.assertIsNone(c.cycle_pos2_mm, "a descent from pos3 is not a hover cycle")

    def test_pos2_rising_with_pos3_blocked_stays_pos3(self):
        # top of travel: the magnet has passed pos2 and comes back over it
        c = new_ctrl(debounce_mm=0.0)
        c.set_raw_state("pos3", True)
        c.reset_zone(0.0)
        c.on_sensor("pos2", True, +1, 1.0)
        self.assertEqual(c.zone, ZONE_POS3)

    def test_rise_through_pos2_band_sets_trim_in_one_step(self):
        c = new_ctrl(debounce_mm=0.0)
        c.reset_zone(0.0)  # nothing blocked: below pos2
        c.on_sensor("pos2", True, +1, 10.0)  # enters at the lower edge
        # +4% feed error: at m_target the slider rises 0.99 * 1.04 - 1 of the
        # extrusion, so it needs this much to cross the 4.4 mm band
        c.on_sensor("pos3", True, +1, 10.0 + c.band_mm / (0.99 * 1.04 - 1.0))
        self.assertAlmostEqual(c.trim * 1.04, 1.0, places=3)

    def test_fast_rise_is_a_disturbance_not_a_ratio_error(self):
        c = new_ctrl(debounce_mm=0.0)
        c.reset_zone(0.0)
        c.on_sensor("pos2", True, +1, 10.0)
        c.on_sensor("pos3", True, +1, 20.0)  # 4.4 mm in 10 mm: far too fast
        self.assertAlmostEqual(c.trim, 1.0 - c.trim_nudge)

    def test_overlap_nothing_blocked_at_sync_is_below_pos2(self):
        c = new_ctrl()
        c.reset_zone(0.0)
        self.assertEqual((c.zone, c.approach), ("below", True))
        self.assertEqual(new_ctrl(overlap=False).zone, "unknown")

    def test_impossible_state_faults_when_armed(self):
        c = new_ctrl(debounce_mm=0.0)
        c.set_faults_enabled(True, 0.0)
        c.on_sensor("pos1", True, +1, 1.0)
        c.on_sensor("pos3", True, +1, 2.0)
        self.assertIsNotNone(c.fault)

    def test_no_faults_when_not_armed(self):
        c = new_ctrl()
        c.on_sensor("pos1", True, +1, 0.0)
        self.assertIsNone(c.on_progress(500.0))
        self.assertIsNone(c.fault)


class TestSimulatedPrints(unittest.TestCase):
    def assert_hovers_at_pos2(self, sim, settle_mm=400.0):
        late = [x for e, x in sim.samples if e > settle_mm]
        self.assertTrue(late, "no samples after settling")
        lo, hi = sim.hover_band
        near = sum(1 for x in late if lo <= x <= hi) / len(late)
        self.assertGreater(near, 0.95, "slider near pos2 only %.0f%% of the time" % (near * 100))

    def test_nominal_print_holds_pos2_no_faults(self):
        for seed in range(5):
            sim = BufferSim(new_ctrl(), ratio_err=0.0, x0=0.0)
            run_print(sim, 3000.0, seed)
            self.assertIsNone(sim.fault, "seed %d: %s" % (seed, sim.fault))
            self.assert_hovers_at_pos2(sim)
            self.assertLess(sim.max_x, sim.p3_lo, "seed %d reached pos3" % seed)
            per_100mm = sim.rate_changes / (sim.E / 100.0)
            self.assertLess(per_100mm, 6.0, "too many rate changes: %.1f/100mm" % per_100mm)

    def test_ratio_errors_are_trimmed_out(self):
        # Any trim that leaves the remaining error inside (-delta, +eps) is a
        # correct outcome: the slider hovers at pos2. Errors near +eps are the
        # best case (the slider barely moves, almost no rate changes).
        c0 = new_ctrl()
        eps, delta = 1.0 - c0.m_target, c0.m_below - 1.0
        for err in (-0.04, -0.02, 0.0, 0.02, 0.04):
            sim = BufferSim(new_ctrl(), ratio_err=err, x0=0.0)
            run_print(sim, 5000.0, seed=7)
            self.assertIsNone(sim.fault, "err %+.2f: %s" % (err, sim.fault))
            remaining = sim.ctrl.trim * (1.0 + err) - 1.0
            self.assertTrue(
                -delta < remaining <= eps + 0.002,
                "err %+.2f: remaining error %+.4f outside the stable band" % (err, remaining),
            )
            self.assert_hovers_at_pos2(sim, settle_mm=2500.0)
            late_hits = [
                e for e, x in sim.samples
                if e > 2500.0 and (x < sim.p1_top + 0.3 or x > sim.p3_lo - 0.3)
            ]
            self.assertFalse(late_hits, "err %+.2f: slider reached pos1/pos3 after settling" % err)

    def test_high_flow_no_false_faults(self):
        sim = BufferSim(new_ctrl(), x0=0.0)
        run_print(sim, 3000.0, seed=3, max_flow=15.0)
        self.assertIsNone(sim.fault)

    def test_slipping_gear_pauses_promptly(self):
        c = new_ctrl()
        sim = BufferSim(c, x0=0.0)
        run_print(sim, 1000.0, seed=2)
        self.assertIsNone(sim.fault)
        sim.slip = True
        e_slip = sim.E
        run_print(sim, 3000.0, seed=4)
        self.assertIsNotNone(sim.fault)
        self.assertIn("tension", sim.fault)
        # from slip: drain the slack down to pos1 + latency + tension_fault_mm
        slack = sim.hover_band[1] - sim.p1_top
        self.assertLess(sim.fault_E - e_slip, slack + c.tension_fault_mm + 5.0)

    def test_clog_pauses_and_limits_grinding(self):
        c = new_ctrl()
        sim = BufferSim(c, x0=0.0)
        run_print(sim, 1000.0, seed=5)
        self.assertIsNone(sim.fault)
        sim.clog = True
        e_clog = sim.E
        run_print(sim, 3000.0, seed=6)
        self.assertIsNotNone(sim.fault)
        self.assertIn("compression", sim.fault)
        rise = sim.p3_lo - sim.hover_band[0]
        self.assertLess(sim.fault_E - e_clog, rise + c.compression_fault_mm + 5.0)
        # filament pushed into the jammed path beyond the slider's end stop
        self.assertLess(sim.ground_mm, 12.0, "pushed %.1f mm against the stop" % sim.ground_mm)

    def test_filament_ratio_changes_mid_print(self):
        sim = BufferSim(new_ctrl(), ratio_err=-0.02, x0=0.0)
        run_print(sim, 2500.0, seed=11)
        self.assertIsNone(sim.fault)
        sim.ratio_err = +0.03  # e.g. a different filament grips differently
        run_print(sim, 6000.0, seed=12)
        self.assertIsNone(sim.fault, sim.fault)
        self.assertAlmostEqual(sim.ctrl.trim * 1.03, 1.0, delta=0.012)
        late = [x for e, x in sim.samples if e > 6500.0]
        lo, hi = sim.hover_band
        self.assertGreater(sum(1 for x in late if lo <= x <= hi) / len(late), 0.95)

    def test_sensor_noise_near_edges(self):
        sim = BufferSim(new_ctrl(), ratio_err=0.01, x0=0.0, x_noise=0.08, seed=5)
        run_print(sim, 4000.0, seed=13)
        self.assertIsNone(sim.fault, sim.fault)
        self.assert_hovers_at_pos2(sim)
        self.assertLess(sim.rate_changes / (sim.E / 100.0), 10.0)

    def test_slow_rate_application(self):
        sim = BufferSim(new_ctrl(), ratio_err=-0.01, x0=0.0, latency=0.5)
        run_print(sim, 4000.0, seed=14, max_flow=15.0)
        self.assertIsNone(sim.fault, sim.fault)
        self.assert_hovers_at_pos2(sim)

    def test_other_sensor_geometries(self):
        # (pos1 top, pos2 bottom, pos2 top, pos3 bottom) in filament mm
        cases = [
            (False, (1.0, 8.0, 9.5, 16.0), 20.0),
            (False, (3.0, 18.0, 22.0, 34.0), 38.0),
            (False, (2.0, 10.0, 10.8, 20.0), 24.0),
            (True, (8.0, 16.0, 26.0, 20.0), 30.0),
            (True, (25.0, 34.0, 46.0, 41.0), 48.0),
        ]
        for overlap, geo, x_max in cases:
            for err in (-0.03, 0.03):
                sim = BufferSim(new_ctrl(overlap=overlap), ratio_err=err, x0=0.0,
                                geometry=geo, x_max=x_max)
                run_print(sim, 4000.0, seed=15)
                self.assertIsNone(sim.fault, "geo %s err %+.2f: %s" % (geo, err, sim.fault))
                late = [x for e, x in sim.samples if e > 2000.0]
                lo, hi = sim.hover_band
                band = sum(1 for x in late if lo <= x <= hi) / len(late)
                self.assertGreater(band, 0.95, "geo %s err %+.2f: %.0f%%" % (geo, err, band * 100))

    def test_separate_sensor_layout_still_holds_pos2(self):
        for err in (-0.03, 0.0, 0.03):
            sim = separate_sim(ratio_err=err, x0=0.0)
            run_print(sim, 5000.0, seed=9)
            self.assertIsNone(sim.fault, "err %+.2f: %s" % (err, sim.fault))
            self.assert_hovers_at_pos2(sim, settle_mm=2500.0)

    def test_soak_many_prints_no_false_faults(self):
        for seed in range(20, 30):
            err = random.Random(seed).uniform(-0.04, 0.04)
            sim = BufferSim(new_ctrl(), ratio_err=err, x0=0.0)
            run_print(sim, 8000.0, seed)
            self.assertIsNone(sim.fault, "seed %d err %+.3f: %s" % (seed, err, sim.fault))

    def test_retractions_barely_move_the_slider_when_settled(self):
        # Retractions of up to 1 mm happen constantly in real prints. The
        # buffer is synced, so it follows a retraction at m (~1 when hovering):
        # the slider should move only by |m - 1| * retraction.
        for err in (-0.04, 0.0, 0.04):
            sim = BufferSim(new_ctrl(), ratio_err=err, x0=0.0)
            run_print(sim, 5000.0, seed=1, max_retract=1.0)
            self.assertIsNone(sim.fault)
            settled = [abs(b - a) for e, a, b in sim.retract_moves if e > 500.0]
            self.assertGreater(len(settled), 50)
            self.assertLess(max(settled), 0.05, "err %+.2f: a retraction moved the slider %.3f mm" % (err, max(settled)))
            self.assertEqual(sim.tug_total, 0.0, "buffer pulled against the extruder")

    def test_retraction_heavy_print(self):
        # Small parts: very frequent retractions at the 1 mm maximum
        def profile(total_mm, seed):
            rnd = random.Random(seed)
            E = 0.0
            while E < total_mm:
                flow, dur = rnd.uniform(1.0, 15.0), rnd.uniform(0.05, 0.6)
                yield flow, dur
                E += flow * dur
                speed = rnd.uniform(25.0, 45.0)
                yield -speed, 1.0 / speed
                yield 0.0, rnd.uniform(0.02, 0.4)
                yield speed, 1.0 / speed
        for err in (-0.03, 0.03):
            sim = BufferSim(new_ctrl(), ratio_err=err, x0=0.0)
            for v, dur in profile(4000.0, 21):
                sim.run(v, dur)
                if sim.fault:
                    break
            self.assertIsNone(sim.fault, "err %+.2f: %s" % (err, sim.fault))
            self.assertGreater(len(sim.retract_moves), 1000)
            self.assert_hovers_at_pos2(sim, settle_mm=2000.0)

    def test_print_starting_with_retraction_at_rest(self):
        # Worst case: spring fully relaxed (pos1, slack 0) and the very first
        # move is a 1 mm retraction. The buffer pulls back m_pos1 x 1 mm while
        # the extruder releases only 1 mm: a small, bounded tug.
        c = new_ctrl()
        sim = BufferSim(c, x0=0.0)
        sim.run(-35.0, 1.0 / 35.0)
        retracted = -sim.E  # the sim's time step rounds 1 mm to ~1.05 mm
        sim.run(35.0, 1.0 / 35.0)
        run_print(sim, 500.0, seed=3)
        self.assertIsNone(sim.fault)
        self.assertGreater(retracted, 0.99)
        self.assertLessEqual(sim.tug_max, (c.m_pos1 - 1.0) * retracted + 1e-6)

    def test_statistics_account_for_all_extrusion(self):
        c = new_ctrl()
        sim = BufferSim(c, x0=0.0)
        c.reset_stats(0.0)
        run_print(sim, 2000.0, seed=4)
        st = c.stats
        forward = sum(st["zone_mm"].values())
        # forward extrusion includes unretracts; it must at least cover net E
        self.assertGreaterEqual(forward + 0.5, sim.E)
        self.assertLess(forward, sim.E * 1.25)
        self.assertIsNotNone(st["first_pos2_mm"])
        self.assertLess(st["first_pos2_mm"], 150.0)
        self.assertGreater(st["zone_entries"]["pos2"], 3)
        self.assertGreater(st["zone_mm"]["pos2"] / forward, 0.5)

    def test_starting_at_rest_reaches_pos2_quickly(self):
        sim = BufferSim(new_ctrl(), x0=0.0)
        run_print(sim, 400.0, seed=8)
        first_pos2 = next((e for e, x in sim.samples if x >= sim.p2_lo), None)
        self.assertIsNotNone(first_pos2)
        self.assertLess(first_pos2, 120.0, "took %.0f mm to reach pos2" % first_pos2)


if __name__ == "__main__":
    unittest.main()


class ReleaseSim(BufferSim):
    """Unloading: the extruder grips until it has retracted `grip` mm. After
    that the far end is free, so whatever slack the slider still holds is
    the spring pressing the soft tip back into the gears, and the buffer's
    pull moves the tip up the tube instead of the slider."""

    def __init__(self, ctrl, grip, **kw):
        super().__init__(ctrl, **kw)
        self.grip = grip
        self.tip_up = 0.0
        self.x_at_release = None

    def step(self, v):
        if self.E > -self.grip:
            return super().step(v)
        if self.x_at_release is None:
            self.x_at_release = self.x
        dE = v * self.dt
        self.E += dE
        self.t += self.dt
        self.hist.append((self.t, self.E))
        while self.pending and self.pending[0][0] <= self.t:
            self.applied = self.pending.pop(0)[1]
        pull = -self.applied * dE
        take = min(self.x, pull)
        self.x = max(0.0, self.x - take)
        self.tip_up += pull - take
        for name in ("pos1", "pos2", "pos3"):
            s = self.sensors[name]
            if s.update(self.x):
                self.ctrl.on_sensor(name, s.state, -1, self.E)
                self._note_rate()
        if self.t >= self.next_tick:
            self.next_tick += 0.1
            self.ctrl.on_progress(self.E)
            self._note_rate()


def unload_sim(x0, follow, grip=1e9, ratio_err=0.0):
    ctrl = new_ctrl()
    sim = ReleaseSim(ctrl, grip, x0=x0, ratio_err=ratio_err, faults=False)
    ctrl.set_follow(follow)
    # the plugin sets the rate before anything moves: no latency to model
    sim.applied = sim.target = ctrl.effective()
    return ctrl, sim


def synced_retract(x0, follow=None, ratio_err=0.0, mm=120.0, speed=20.0):
    ctrl, sim = unload_sim(x0, follow, ratio_err=ratio_err)
    xs = []
    for _ in range(int(mm / speed / 0.025)):
        sim.run(-speed, 0.025)
        xs.append(sim.x)
    return sim, min(xs), max(xs)


class TestUnload(unittest.TestCase):
    """Slack changes by (m - 1) * dE whichever way the extruder runs."""

    def test_forward_logic_on_a_retraction_pulls_against_the_extruder(self):
        # starting below pos2 the zone multipliers drive the slider the wrong way
        for x0, err in ((10.0, 0.0), (10.0, -0.03), (24.0, 0.03)):
            sim, lo, hi = synced_retract(x0, follow=None, ratio_err=err)
            self.assertGreater(sim.tug_total, 25.0, (x0, err))

    def test_holding_pos2_presses_the_tip_into_the_gears(self):
        # zone control from pos2: the spring is still compressed at the release
        ctrl, sim = unload_sim(29.5, None, grip=65.0)
        sim.run(-35.0, 25.0 / 35.0)
        sim.run(-20.0, 120.0 / 20.0)
        self.assertGreater(sim.x_at_release, 15.0)

    def test_a_relaxed_follow_never_pushes(self):
        for err in (-0.03, 0.0, 0.03):
            ctrl, sim = unload_sim(2.0, 1.02, grip=65.0, ratio_err=err)
            sim.run(-35.0, 25.0 / 35.0)
            sim.run(-20.0, 120.0 / 20.0)
            self.assertLess(sim.x_at_release, 3.0, err)
            self.assertEqual(sim.ground_mm, 0.0)
            # a slight pull while gripped, a few percent of the travel at most
            self.assertLess(sim.tug_total, 0.06 * 65.0 + 1.0, err)

    def test_after_the_release_the_buffer_carries_the_tip_away(self):
        ctrl, sim = unload_sim(2.0, 1.02, grip=65.0)
        sim.run(-35.0, 25.0 / 35.0)
        sim.run(-20.0, 120.0 / 20.0)
        self.assertAlmostEqual(sim.tip_up, 1.02 * (145.0 - 65.0), delta=3.0)
        # which is how BUFFER_UNLOAD gets the gears-to-nozzle length back
        self.assertAlmostEqual(145.0 - sim.tip_up / 1.02, 65.0, delta=3.0)

    def test_nothing_is_learned_at_a_fixed_multiplier(self):
        ctrl, sim = unload_sim(5.0, 1.02, ratio_err=0.04)
        trim = ctrl.trim
        for _ in range(200):
            sim.run(-20.0, 0.025)
        self.assertEqual((ctrl.trim, ctrl.trim_updates), (trim, 0))
        ctrl.set_follow(None)
        self.assertNotEqual(ctrl.multiplier(), 1.02)
