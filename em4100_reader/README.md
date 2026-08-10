# EM4100 reader

`em4100_reader.ino` generates a 125 kHz field on D9, samples the passive
envelope detector on A0, and reports complete parity-valid 64-bit EM4100
frames. This is experimental short-range hardware.

Before connecting A0, verify its voltage with a meter and preferably an
oscilloscope. It must remain between 0 V and the Arduino supply voltage.
Direct D9 tank drive is experimental.

## Use

1. Tune the circuit with [`coil_tuner`](../coil_tuner/).
2. Open the sketch, select **Arduino Uno**, upload it, and open Serial Monitor
   at **115200 baud**.
3. Hold one tag centered and still over the coil for several seconds.

Output uses this form:

```text
EM4100 ID: <10_HEX_DIGITS>  (Manchester RF/64)
```

The same ID must pass complete row and column parity checks in three distinct
frames before it is reported.

## Decoder modes

The reader supports Manchester and biphase at RF/64 and RF/32. `START_MODE`
defaults to `MANCHESTER_RF64`, the common EM4100 format, to stay within the
Uno processing budget. Set it to `MODE_COUNT` when format auto-detection is
needed; the decoder locks to the first valid format.

## Timing and limitations

Timer1 generates the carrier. Timer2 starts an ADC conversion every 64 µs so
samples occur at a consistent carrier phase. Healthy diagnostics show
`missed=0 late=0`.

The sketch disables the Timer0 overflow interrupt while sampling; do not add
`delay()`, `millis()`, or `micros()` to the sampling loop. ASK Manchester and
biphase are supported; PSK is not. Range is short.
