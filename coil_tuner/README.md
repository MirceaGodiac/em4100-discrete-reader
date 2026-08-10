# Coil tuner

`coil_tuner.ino` sweeps the Arduino Timer1 carrier and measures the diode
envelope on A0. Use it to tune the LC tank without an LCR meter.

## Use

1. Build the circuit shown in the [project README](../README.md).
2. Remove all tags from the coil.
3. Open `coil_tuner.ino` in the Arduino IDE.
4. Select **Arduino Uno**, upload, and open Serial Monitor at **115200 baud**.

The sketch sweeps approximately 83–174 kHz and prints:

```text
OCR1A  freq_kHz  A0_avg
```

It finishes with the measured peak and the level at 125 kHz.

## Target

For a 16 MHz Arduino Uno:

```text
OCR1A=63 -> 125.0 kHz
```

Aim for a broad response centered close to that value. Absolute ADC level is
less important than a clear difference between tag-present and tag-absent
measurements.

## Adjusting the coil

- Peak below 125 kHz: reduce inductance or capacitance. With fixed C1, remove a
  turn or spread the winding.
- Peak above 125 kHz: increase inductance or capacitance. With fixed C1, add a
  turn or compress the winding.

The diode detector contains carrier ripple, so neighboring sweep values may not
form a perfectly smooth curve.
