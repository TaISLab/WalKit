# Handle firmware (SC182029-FISTS-V1R0)

Arduino Nano + HX711 load cell amplifier on each handle. The sketch sends one line per measurement over USB serial,
`<preamble><number>\n` (`LL:12.34`, `RL:7.80`), which `walker_arduino/usb_conn` reads, finds the port by its preamble, and
publishes as `/left_handle` and `/right_handle` (`walker_msgs/ForceStamped`). `partial_loads` turns that reading into kg with
`walker_loads/config/handle_calib.yaml`.

**The changes below were not run on a board** (no Arduino toolchain here): the sketch passes a syntax check against stubs of the Arduino
and `HX711` (RobTillaart 0.3.x) APIs, nothing more. Compile it, flash one handle, and check the rate before putting it on the walker.

## Why the handles gave ~2.4 Hz

The old loop was `power_up(); get_units(); power_down();` for every reading. The HX711 needs about **400 ms** to settle after waking at
10 SPS, so each reading cost that: 1 / 0.42 s = 2.4 Hz, the rate measured on the handles of 37 of the 47 definitive bags. (`rate_hz` in
`walker_arduino` is only a ceiling for the ROS publisher; it cannot make the Arduino send more.) A gait cycle takes ~1.1 s: at 2.4 Hz
the load on the handles was below the Nyquist rate of the steps (1.8 Hz), and everything built from it (the centre of pressure, the
stability margin) used stale values.

## What changed in V1R1

- The HX711 stays on. One line per conversion, taken as soon as `is_ready()` says there is one, without blocking: **10 Hz** with the
  breakout board as it comes (RATE pin to ground = 10 SPS), **80 Hz** if the RATE pin is wired to VCC (on most boards that means cutting a
  trace: a hardware change, to be tried on one board first). `AVERAGE_SAMPLES` (default 1) averages several conversions into a line,
  e.g. 4 at 80 SPS gives 20 Hz with less noise. For the walker 25 Hz or more is the aim (the gait cycle is ~1.1 s and the stability work uses the loads on a 20 Hz grid); 10 Hz is already
  four times what there was.
- The zero: waits `SETTLE_MS` after start-up, drops the first conversions and averages `TARE_SAMPLES` = 20 (it was 10).
- Serial commands, only when somebody sends them (`ros2` never does): `T` repeats the tare (walker unloaded, hands off), `S` prints
  `#scale=… offset=… conversions_per_s=… average=…`. Replies start with `#` and are never sent unasked: `usb_conn` parses every line as
  `<preamble><number>` and would throw on text.
- Unchanged: the line format, `HX711_SCALE` (2280), the baud rate. The calibration table still applies.

## Flashing

Arduino IDE, board *Arduino Nano*, processor *ATmega328P* (old bootloader if the board needs it), library *HX711* by Rob Tillaart
**0.3.x**. Choose the side with `HANDLE_SIDE` (`"LL:"` left, `"RL:"` right) before compiling each.

- **Start with the hands off the handles.** The zero is taken when the board starts, and opening the port from the ROS node resets the
  Nano (DTR), so the zero is retaken every time the node starts.
- Check: `ros2 topic hz /left_handle` should read ~10 Hz (or the rate you chose), not 2.4. `screen /dev/ttyUSB0 115200` and `S` shows the
  conversion rate the chip really gives.

## Calibration: what to do beyond the firmware

1. **Extend the table past 20 kg.** `handle_calib.yaml` has 12 points up to 20 kg (33.4 readings/kg). `SplineFunction::interp` **clamps**
   the readings above the last point to its weight, so any load over 20 kg on a handle is reported as 20 kg. In the definitive bags the
   left handle sat above the table in 13 to 22 % of the samples of MF_test09 to MF_test14 and in 17 % of LA_test07. The HX711 itself is
   far from saturating (20 kg is 1.5 M of its 8.4 M counts). Calibrate to 50 kg.
2. **Load where the hands go.** Put the masses at the grip position, not over the sensor, and once pushing forward as well (cross
   sensitivity). A bending beam reads differently at different lever arms.
3. **Zero.** Retake it (`T`) at the start of a session with the walker unloaded, and look at the 0 kg reading at the end: a drift shows up
   there. Raw readings down to -132 were recorded in MT_test18 to 20.
4. **Ground truth.** The vertical load really transmitted is what a scale under the walker's wheels reads when the user leans on it
   (the user's feet on another scale). Compare it with the handles' sum.

`walker_arduino/scripts/calibrate_handle.py` measures the table (or reads it from a CSV) and prints the lines for `handle_calib.yaml`
and a linearity report; see its docstring.

After a recalibration, repeat the check of the centre-of-mass bias (`walker_stability/README.md`, "Bias of the centre of mass"): if the
`-8.75 kg·m / weight` signature goes away, the handles were the cause; if not, `com_moment_kgm` stays.
