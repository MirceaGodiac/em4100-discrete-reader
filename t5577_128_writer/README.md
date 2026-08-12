# Guarded ATA5577 128-bit writer

> This standalone writer is retained as a focused reference. For normal use,
> the primary [`em4100_cli`](../em4100_cli/) sketch combines source reading,
> exact 64/128-bit verification, writing, and standard recovery.

This standalone Arduino Uno sketch writes the Electra-style 128-bit layout to
an **unlocked, writable ATA5577/T5577 with real page-0 blocks B1 through B4**.
It uses the existing circuit: D9 supplies the 125 kHz carrier and downlink gaps,
and A0 reads the passive envelope detector.

It does not use a password, set password mode, or set any lock bit. It cannot
unlock, identify, or repair a password-protected tag. Read-only EM4100/EM4102
tags and limited-memory emulators are not supported.

## Safety and setup

Writing can leave a tag unreadable after weak coupling, movement, reset, power
loss, or use of an incompatible tag. Use only one tag that you own or are
authorized to program. Keep it centered and still until success or verified
recovery. Direct D9 tank drive is experimental and may not supply enough
programming energy.

The prepared data and recovery lock exist only in Arduino RAM. **Resetting or
powering off the Uno loses both, even when recovery is still required.** Do not
reset, upload, or disconnect power during a write/recovery incident.

1. Tune and validate the existing D9 carrier/A0 envelope circuit with the
   repository's reader tools.
2. Confirm that A0 remains between 0 V and the Uno supply voltage.
3. Compile/upload `t5577_128_writer.ino` for `arduino:avr:uno`.
4. Open Serial Monitor at 115200 baud with Newline enabled.
5. Before writing another tag, use the real source key with `ELECTRA` and
   `VERIFY`. Proceed only when its exact 128-bit data passes with healthy
   sampling.

## Commands

```text
ELECTRA <10_HEX_DIGITS> <16_HEX_DIGITS>
VERIFY
WRITE
RECOVER
CANCEL
HELP
```

- `ELECTRA` accepts exactly a 40-bit ID and an exact 64-bit suffix. It creates
  the complete parity-valid EM4100 frame in RAM. There are no built-in
  credential values or defaults.
- `VERIFY` is receive-only. It sends no downlink command and does not touch
  EEPROM. It looks for the prepared parity-valid base frame immediately
  followed by the exact prepared 64-bit suffix.
- `WRITE` writes and verifies the prepared 128-bit value.
- `RECOVER` warns, waits two seconds, and restores the prepared base ID as a
  standard 64-bit EM4100 transmission.
- `CANCEL` clears prepared RAM data. It is blocked while recovery is required.
- `HELP` prints the command summary.

If recovery fails, only `RECOVER` and `HELP` are accepted. Correct coupling
without resetting the Uno, then run `RECOVER` again.

## Exact block map

| Page-0 block | Contents |
| --- | --- |
| B0 | `0x00148080`: RF/64, Manchester, MAXBLK=4 |
| B1 | Upper 32 bits of the parity-valid 64-bit EM4100 frame |
| B2 | Lower 32 bits of the parity-valid 64-bit EM4100 frame |
| B3 | Upper 32 bits of the exact supplied suffix |
| B4 | Lower 32 bits of the exact supplied suffix |

The standard recovery configuration is B0 `0x00148040`: RF/64, Manchester,
MAXBLK=2. Both configuration words leave password mode and lock bits clear.

Programming order is B1, B2, B3, B4, then B0. B0 is deliberately last so
MAXBLK=4 cannot expose B3/B4 before those blocks are populated. EEPROM
programming order does not change the transmitted order: with MAXBLK=4 the tag
always transmits B1, B2, B3, B4.

The 64-bit base encoder emits all nine header bits, ten data nibbles and row
parities, four column parity bits, and the stop bit.

## Verification

The receiver uses the known parity decoder extended by a 64-bit suffix capture;
it does not use a raw rolling-pattern matcher. It samples A0 every 64 us, eight
samples per RF/64 bit, and runs eight possible Manchester bit phases in both
envelope polarities (16 decoders).

Timer1 and Timer2 are started on the same prescaler boundary with `GTCCR`, as in
the working reader. A result reports:

- ADC minimum and maximum
- missed ADC starts and late reads
- physically distinct parity-valid base frames
- physically distinct prepared-ID base frames
- exact prepared 128-bit cycles
- the last suffix observed after the prepared base when that suffix was wrong

Adjacent phase decoders are de-duplicated by sample distance. Success requires
at least three physically distinct exact 128-bit observations and
`missed=0 late=0`. `WRITE` performs this check after a final field power-cycle.
`VERIFY` performs the same receive-only check without programming.

## Guarded write and recovery

`WRITE` sets the RAM `recoveryRequired` flag before the first EEPROM command.
It writes B1 through B4, activates them by writing B0 last, power-cycles the
field, and verifies the exact base-plus-suffix cycle.

If exact verification fails, the sketch automatically makes up to three
standard recovery attempts. Each attempt writes B1, B2, then
`B0=0x00148040`, power-cycles the field, and requires at least three physically
distinct parity-valid frames for the prepared ID with `missed=0 late=0`.
Prepared data is retained whether recovery succeeds or fails.

`RECOVER` uses the same three-attempt flow after a destructive warning and a
two-second delay. Recovery is best-effort; software cannot compensate for a
protected/incompatible tag, absent B3/B4 storage, poor tuning, insufficient
programming field strength, or tag movement.

## Downlink timing and framing

The fixed, proven 125 kHz timing profile is:

- power-up: 400 field clocks
- start gap: 30 field clocks
- write gap: 19 field clocks
- zero: 24 field clocks
- one: 54 field clocks
- programming hold: 900 field clocks (7.2 ms)

Every page-0 write sends opcode `10`, clear lock bit `0`, 32 data bits most
significant bit first, then the three-bit block address most significant bit
first. No password bits or password value are sent.

The block layout was cross-checked against the official Flipper Electra format
as factual guidance. This sketch is an original implementation and does not
copy Flipper's GPL source.
