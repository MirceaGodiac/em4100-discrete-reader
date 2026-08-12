# EM4100 128/64-bit reader

> This standalone reader is retained for decoder experiments and RF/32 or
> biphase testing. For normal Manchester RF/64 reading and guarded writing, use
> the combined [`em4100_cli`](../em4100_cli/) sketch.

`em4100_reader.ino` generates a 125 kHz field on D9, samples the passive
envelope detector on A0, and reports complete parity-valid EM4100 frames. In
Manchester RF/64 mode it also inspects the following 64 bits to distinguish a
repeating standard frame from a stable 128-bit candidate. This is experimental
short-range hardware.

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
Transmission length: 64 bits (validated frame repeats).
No distinct second 64-bit segment was observed.
```

The same ID must pass complete row and column parity checks in three distinct
frames before it is reported.

## 128-bit classification

After a valid Manchester RF/64 frame, the reader captures the next 64 decoded
bits three times:

- If those bits exactly reproduce the validated EM4100 frame, it reports a
  64-bit repeating transmission.
- If they are stable and different, it prints both halves as a
  `128-bit candidate`.
- If valid IDs repeat but no stable second half can be decoded, it falls back
  to reporting the validated 64-bit EM4100 frame and marks the total
  transmission length as undetermined.

The classification describes only the observed ASK data stream. It does not
prove how an access system authenticates the credential. A system may use a
different modulation, a challenge-response exchange, another frequency, or
additional data that this reader cannot decode.

## Decoder modes

The reader supports Manchester and biphase at RF/64 and RF/32. `START_MODE`
defaults to `MANCHESTER_RF64`, the common EM4100 format, to stay within the
Uno processing budget. Set it to `MODE_COUNT` when format auto-detection is
needed; the decoder locks after three matching valid frames. Extended
classification is available only in Manchester RF/64 mode.

## Timing and limitations

Timer1 generates the carrier. Timer2 starts an ADC conversion every 64 µs so
samples occur at a consistent carrier phase. Healthy diagnostics show
`missed=0 late=0`.

The sketch disables the Timer0 overflow interrupt while sampling; do not add
`delay()`, `millis()`, or `micros()` to the sampling loop. ASK Manchester and
biphase are supported; PSK is not. Range is short.
