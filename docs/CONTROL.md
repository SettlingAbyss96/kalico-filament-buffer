# Control law and the math behind it

This is the model the controller is built on, the assumptions it makes, and where each default
number comes from. The code is in `filament_buffer.py` (`FeedController` for the control law,
`calibration_result` for calibration), and `tests/test_controller.py` checks all of it in a
simulation of the same model. [DESIGN.md](DESIGN.md) gives the plain-language version.

Measured values are from a Mellow LLL Buffer Plus on a Voron 2.4 (2026-10-04).

## 1. Symbols

| Symbol | Meaning |
|---|---|
| $E$ | extruder position, commanded filament mm (retractions make it go down) |
| $x$ | slider position: filament mm of slack stored in the buffer, $0$ at the rest end |
| $m$ | zone multiplier chosen by the controller (table in section 3) |
| $\tau$ | trim, learned, bounded to $1 \pm 0.05$ |
| $r$ | remaining feed error of the buffer: filament grip, wear, a slightly wrong `rotation_distance` |
| $g = (1+r)\\,\tau$ | effective gain: how much the buffer really feeds per mm the extruder takes, before $m$ |
| $e = g - 1$ | remaining ratio error after trimming |
| $\delta,\ \varepsilon$ | hover offsets: $m_{below} = 1+\delta$, $m_{target} = 1-\varepsilon$, with $\delta = 0.02$, $\varepsilon = 0.01$ |
| $x_1, x_2, x_3$ | sensor edges: top of the pos1 zone, lower edge of pos2, lower edge of pos3 |
| $w = x_3 - x_2$ | the pos2 band (`band_mm`) |

## 2. Plant model

The buffer motor is synced to the extruder, and the plugin sets its rotation distance to
$\mathrm{rd}_{base} / (\tau m)$, so the buffer commands $\tau m$ mm for each mm of extruder motion.
It really delivers $(1+r)$ times what it commands. Whatever the buffer delivers and the extruder
does not take ends up in the slider:

```math
\frac{dx}{dE} = (1+r)\,\tau\,m(z) - 1 = g\,m(z) - 1
```

This holds in both directions of $E$, so retractions are covered by the same equation. Time
doesn't appear: the slider only moves when the extruder does, so every rate and every fault
threshold below is a distance in extruded mm, not a time. Slow moves, travels and pauses can't
cause false faults.

Assumptions:

1. The filament path between the two gears is inextensible apart from the slider: the slider is
   the only place slack can go. Elastic give in the tube is small next to the sensor spacing and is
   handled in calibration (section 9).
2. The extruder takes exactly its commanded $E$. Its own `rotation_distance` error doesn't matter:
   calibration measures the buffer against commanded $E$, so only the ratio counts.
3. $r$ changes slowly (per filament or spool), so it can be learned.
4. A rate change reaches the motor after a latency $t_\ell \approx 0.1$ to $0.3$ s (the step
   generation window). Between sensor edges, $m$ is piecewise constant.

## 3. The zone multiplier

The sensors give a zone $z$, not a position. On the LLL Plus pos2 stays blocked from $x_2$ up
through pos3 (the "overlap" layout), so the sensor states map to zones with no ambiguity:

| pos1 | pos2 | pos3 | Slider | Zone |
|---|---|---|---|---|
| blocked | | | $x \le x_1$ | pos1 |
| | | | $x_1 < x < x_2$ | below |
| | blocked | | $x_2 \le x < x_3$ | pos2 |
| | blocked or not | blocked | $x \ge x_3$ | pos3 |

Measured: $x_1 \approx 19$ mm, $x_2 = 28.6$ mm, $x_3 = 33.0$ mm, so the gap is $9.6$ mm and
$w = 4.4$ mm.

```math
m(z) =
\begin{cases}
m_1 = 1.50 & \text{pos1} \\
m_{ab} = 1.15 & \text{below, approaching (came up from pos1)} \\
m_b = 1 + \delta = 1.02 & \text{below, hovering (slipped out of pos2)} \\
m_t = 1 - \varepsilon = 0.99 & \text{pos2} \\
m_{aa} = 0.85 & \text{pos2, coming down from pos3} \\
m_3 = 0.30 & \text{pos3}
\end{cases}
```

With three separate sensor windows (`sensor_layout: separate`) there is also a zone above pos2,
with $m_a = 0.98$ hovering and $m_{aa} = 0.85$ approaching. Which side the slider left pos2 on
is then inferred from the sign of $dx/dE$ inside pos2, which is negative while extruding when
$g\\,m_t < 1$.

## 4. Hovering at the lower edge of pos2

Near $x_2$ the controller is a relay (sliding-mode) control: above the edge $m = 1-\varepsilon$,
below it $m = 1+\delta$. The slider is pushed back to the edge from both sides when

```math
g\,(1-\varepsilon) < 1 < g\,(1+\delta)
\quad\Longleftrightarrow\quad
\frac{1}{1+\delta} < g < \frac{1}{1-\varepsilon}
```

which to first order is $-\delta < e < \varepsilon$, or $-2\\% < e < +1\\%$. Inside that band the slider
chatters around $x_2$ with an amplitude set by the debounce and the latency:

```math
|\Delta x| \lesssim \left|\frac{dx}{dE}\right| \left(d + v_E\, t_\ell\right)
```

where $d = 0.3$ mm is the debounce (section 7) and $v_E$ the extrusion speed. With
$|dx/dE| \le 0.02$ that is a few hundredths of a millimeter. Because the push of the spring
depends on $x$ only, holding $x$ at $x_2$ holds the assisting force constant.

Errors near $+\varepsilon$ are the best case: $g\\,m_t$ is close to 1, the slider barely moves and
almost no rate changes are sent.

## 5. Learning the trim

The trim only moves inside $[1 - 0.05,\ 1 + 0.05]$, and every update has the form
$\tau \leftarrow \tau\\,(1 + \Delta)$ with that clamp.

### Hover cycles

A clean cycle is pos2, out to below, back to pos2, all at the hover rates. Let $E_2$ and $E_b$ be
the extrusion spent in pos2 and below. The slider falls $b\\,E_2$ and rises $a\\,E_b$, and both are
the same excursion, so

```math
a = g(1+\delta) - 1, \qquad b = 1 - g(1-\varepsilon), \qquad a\,E_b = b\,E_2
```

The share of extrusion spent inside pos2 is

```math
f_2 = \frac{E_2}{E_2 + E_b} = \frac{a}{a+b} \approx \frac{e + \delta}{\delta + \varepsilon}
\quad\Longrightarrow\quad
\hat e = f_2\,(\delta + \varepsilon) - \delta
```

The update is $\Delta = -k\\,\hat e$ with gain $k = 0.5$ (`trim_gain`). A gain below 1 averages out
noise in $f_2$ from debounce, latency and retractions.

### Rising through the pos2 band

If the slider enters pos2 at its lower edge and still rises all the way to pos3 at $m_t$, the
error is above $+\varepsilon$ and the cycle method never gets a cycle. The transit itself measures
it. It took $\Delta E$ of extrusion to rise by $w$:

```math
\rho = \frac{w}{\Delta E} = g\,m_t - 1
\quad\Longrightarrow\quad
g = \frac{1+\rho}{m_t},
\qquad
\tau' = \tau\,\frac{m_t}{1+\rho}
\quad (\text{so that } g' = 1)
```

This is applied in one step. If $w$ is off by a factor $f$, the trim lands at
$g' \approx (1+\rho)/(1+f\rho)$. Overshooting low ($f > 1$) is safe because the slider then hovers
at the lower edge and the cycles refine $\tau$. Undershooting just leads to another, slower rise.
A rise with $\rho > 2 \cdot 0.05 + \varepsilon$ is faster than any ratio error the trim could
correct, so it is treated as a disturbance and gets the fixed nudge instead.

### Nudges

Reaching pos1 from "below, hovering", reaching pos3 from pos2, and hover legs longer than
`hover_stall_mm` (60 mm) each move the trim by $\pm 1\\%$ (`trim_nudge`) in the direction that
would have prevented it. With separate sensors, a nudge based on a wrong exit-side guess is
reverted when a hard sensor proves the guess wrong.

## 6. Retractions

The same plant equation gives the slider's motion during a retraction of length $\ell$:

```math
\Delta x = (g\,m - 1)\,(-\ell)
```

While hovering, $|g\\,m - 1| \le$ a few percent, so a 1 mm retraction moves the slider
less than 0.05 mm (the simulation checks this over thousands of retractions with errors of $\pm 4\\%$).
The buffer would pull against the extruder only if $x$ had to go below $0$, which needs the slider
at the rest end with the spring fully relaxed. A print that starts there with a retraction gives
one bounded tug:

```math
\text{tug} \le (m_1\,g - 1)\,\ell \approx 0.5\ \text{mm for } \ell = 1\ \text{mm}
```

That bound is fine for a 1 mm print retraction, and wrong for a 100 mm unload. Long retractions
get their own section: [11](#11-unloading-running-it-backwards).

## 7. Debounce and rate limiting

A sensor edge at extruder position $E_0$ counts only once the extruder has travelled

```math
\int_{E_0} |dE| \ \ge\ d = 0.3\ \text{mm}
```

with the new state still holding. A flicker shorter than that is dropped, and an accepted edge
keeps its original $E_0$, so zone distances are measured from where the edge really was.
Measuring travel with $|dE|$ means retractions count too, so an edge is not held back forever by
back-and-forth motion.

Rate changes are sent only when $m$ changes, at most once every 0.2 s. They use
`set_rotation_distance()` and never touch the motion queue.

## 8. Faults

Every fault is a distance of extrusion spent in a zone, $E - E_{entry}$.

**F2, tension** (tangle, slipping gear): still at pos1 after $D_2 = 60$ mm. From the rest end a
working buffer climbs out of the pos1 zone (depth $x_1 \approx 19$ mm) at $g\\,m_1 - 1 \approx 0.5$, so it
needs

```math
\Delta E = \frac{x_1}{g\,m_1 - 1} \approx \frac{19}{0.5} = 38\ \text{mm} < D_2
```

**F3, compression** (clog, extruder slipping): still at pos3 after $D_3 = 25$ mm. From the end
stop, about $s \approx 7$ mm above $x_3$, a working buffer falls at $1 - g\\,m_3 \approx 0.7$, so it
leaves pos3 within $s/0.7 \approx 10$ mm. In a real jam nothing is consumed, so
$dx/dE = g\\,m_3$, and by the time F3 pauses the buffer has pushed at most

```math
g\,m_3\,D_3 \approx 0.3 \times 25 = 7.5\ \text{mm}
```

into the jammed path. That is why $m_3$ is low.

**From rest to pos2**: $38$ mm in pos1 plus the gap at the approach rate,
$9.6 / (m_{ab} - 1) \approx 64$ mm, so about $100$ mm of extrusion, well inside the 200 mm the
extrusion test allows.

## 9. Calibration

`BUFFER_CALIBRATE` measures the buffer's true feed per commanded mm, $k$, against the extruder.

Model each sensor $i$ with an edge $u_i$ and a hysteresis $h_i$ on its clearing side: pos1 clears
at $u_1 + h_1$ when the slider rises and blocks at $u_1$ when it falls; pos3 blocks at $u_3$ when
rising and clears at $u_3 - h_3$ when falling.

- **Up:** the buffer feeds with the extruder holding. From pos1 clearing to pos3 blocking it
  commands $S_b$ and moves the slider by $u_3 - u_1 - h_1 = k\\,S_b$.
- **Down:** the extruder takes $S_e$ with the buffer holding. From pos3 clearing to pos1 blocking
  the slider moves by $u_3 - h_3 - u_1 = S_e$.

With $h_1 = h_3$ (the same sensor type; on the LLL Plus both measured below 0.1 mm) the two
spans are equal, and

```math
k = \frac{S_e}{S_b}, \qquad \mathrm{rd}_{new} = k \cdot \mathrm{rd}_{old}
```

The same reasoning removes other errors that are equal in both directions:

- A constant friction lag of the slider shifts both spans' endpoints by the same amount.
- Elastic give in the tube depends on the spring force, which is the same at the same sensor
  states, so it adds the same length to both spans.

What doesn't cancel is slack that was in the path before the first move, which is why calibration
runs one unmeasured warm-up cycle first.

Edge positions don't depend on where the moves stop. Each sensor report carries its receive time
$t$, which is converted to print time and then to positions from the step history:
$B(t)$ for the buffer motor and $E(t)$ for the extruder. Runs must agree within 4%, and the
median is used.

| Measurement | $S_b$ (commanded) | $S_e$ | $k$ |
|---|---|---|---|
| Before, with `rotation_distance` 13.974 | 30.0 and 31.0 mm | 13.5 and 14.0 mm | 0.450, 0.452 |
| After, with 6.300 | about 13.6 mm | about 13.8 mm | 1.009, 1.035, 1.004 |

So the true value is $13.974 \times 0.451 = 6.30$ mm per motor turn, and what is left is within
the trim's $\pm 5\\%$.

## 10. Stopping independent moves

Loading, the buttons and calibration move the buffer on its own (never while printing).

- **At a sensor:** pos2 and pos3 are also endstops on the buffer MCU. The move runs as a homing
  move and the MCU stops the motor when the sensor trips, so the stop doesn't wait for the host.
  The overshoot is the motor stopping from speed $v$, a fraction of a millimeter.
- **On a host condition** (a released button, filament leaving the inlet): steps are sent in
  segments of $T_{seg} = 50$ ms at most $T_{drip} = 100$ ms ahead, and the condition is checked
  between segments, so the motor runs on for at most

```math
\Delta s \le v\,(T_{drip} + T_{seg}) = 0.15\,v
```

  which is 3 mm at the 20 mm/s button speed. Before this change, button moves were queued up to
  2 s ahead and kept going long after the button was released.

## 11. Unloading: running it backwards

The plant equation from section 2 doesn't care which way the extruder runs:

```math
dx = (g\,m - 1)\,dE
```

The zone multipliers were picked for $`dE > 0`$. In pos1 the slider needs slack, so $`m_1 > 1`$
makes $`dx`$ positive. Flip the sign of $`dE`$ and the same $`m_1`$ makes $`dx`$ negative: the
correction drives the slider further the wrong way. A print retraction is too short for that to
matter (section 6). An unload is not. In the simulation, a 120 mm synced retraction that starts
below pos2 with the forward multipliers pulls 30 to 50 mm of filament against the extruder's grip,
which is grinding, plain and simple.

Mirroring the multipliers around 1 would fix the sign, and it's still the wrong idea, because it
holds the slider at pos2. The spring then presses the filament toward the extruder with all of
$`x_2`$ compressed, and at the moment the tip comes out of the gears that force lands on the soft
tip, against gears that are still turning. On the printer that left a mashed, swollen tip that
wouldn't go back up the tube, four times in a row.

So the unload doesn't control the slider at all. It takes the slack out first, leaving
$`x_0 = 2`$ mm, then runs the buffer at a fixed $`m = f = 1.02`$:

```math
dx = (g\,f - 1)\,dE < 0 \quad\text{for } dE < 0,\ g\,f > 1
```

so the slider can only lose slack. It sits at the rest end, and the extruder sets the pace while it
holds the filament (the buffer slips by $`(g f - 1)`$, a few percent, which the simulation keeps
under 5 mm over 65 mm with $`\pm 3\%`$ ratio errors). At the release there is nothing compressed to
push the tip back, and the buffer is already pulling.

**The gears-to-nozzle length.** With a total retraction $`E_{max}`$ and the release at $`L_n`$,
the buffer carries the tip $`f\,(E_{max} - L_n)`$ past the gears. After the free test feeds $`t`$,
feeding to contact takes $`F = f\,(E_{max} - L_n) - t + x_2`$, so

```math
L_n = E_{max} - \frac{F - x_2 + t}{f}
```

The simulation gets it back within 3 mm.

**The free test.** Feeding $`t`$ mm with the far end free just slides the filament, and the slider
stays relaxed. With the far end held, all of it lands in the slider. Choose

```math
x_1 < t < f\,(E_{max} - L_n)
```

so a held tip pushes the slider out of pos1 and a free one can't reach the gears. With $`t = x_1 + 3 = 22`$
and $`E_{max} = L_n + 45`$ that leaves a free tip about 24 mm above the gears after the test. If the
tip is stuck, the buffer slips for the overrun, about 45 mm, and the test stops it there.

**Where the tip ends up.** From contact the slider holds $`x_2`$, so with the inlet-to-gears path
$`L_p`$ measured by an autoload (the fed distance minus $`x_2`$), the final pull is

```math
\text{park: } x_2 + L_p - p, \qquad \text{eject: } x_2 + L_p + 60
```

with $`p = 50`$ mm. None of these lengths has to be exact. A 20 mm error in $`L_p`$ moves the
parked tip by 20 mm inside a 50 mm margin, an eject that pulls too far just spins the buffer gear,
and a load that feeds too far purges a little more.
