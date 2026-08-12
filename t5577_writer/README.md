# T5577/T5557 EM4100 writer

> This standalone writer is retained as a compact reference. The primary
> [`em4100_cli`](../em4100_cli/) sketch combines reading, stronger exact
> `frame || frame` verification, writing, and recovery in one interface.

`t5577_writer.ino` prepares, writes, and verifies a standard parity-valid
64-bit EM4100 transmission on compatible T5577/T5557 tags. It uses the D9 LC
tank and A0 passive envelope detector. This is experimental, tested
short-range hardware; direct D9 drive may not provide enough programming
energy for every tag.

Factory-programmed EM4100/EM4102 tags are read-only.

## Setup

1. Tune the coil with [`coil_tuner`](../coil_tuner/).
2. Confirm reliable reads with [`em4100_reader`](../em4100_reader/).
3. With A0 disconnected, verify the envelope voltage using a meter and
   preferably an oscilloscope. Connect A0 only when it remains between 0 V and
   the Arduino supply voltage.
4. Open the writer sketch, select **Arduino Uno**, upload it, and open Serial
   Monitor at **115200 baud** with **Newline** enabled.
5. Use only a tag and system you own or are authorized to test.

## Write

**WARNING: writing is destructive. Power loss, weak coupling, or moving the
tag can leave it unreadable. Keep exactly one writable tag centered and still
until verification or recovery finishes. Recovery is best-effort and cannot
guarantee repair. Do not reset or power off the Arduino while recovery is
required because its prepared recovery data is stored only in RAM.**

Prepare a 40-bit ID, then write it:

```text
ID <10_HEX_DIGITS>
WRITE
```

`ID` accepts exactly ten hexadecimal characters and only prepares data. The
writer encodes all EM4100 header, row parity, column parity, and stop bits,
then writes B1, B2, and B0. B0 uses the standard configuration
`0x00148040` for Manchester RF/64. Password and lock bits remain clear.

Success requires the requested ID to decode with valid parity in at least
three distinct repeated RF frames. Keep every other RFID tag out of range:
verification confirms the received transmission, not an addressed block
readback from one physically identified tag.

## Recovery

The sketch marks recovery as required before the first tag mutation. If final
verification fails, it keeps the prepared blocks and immediately attempts a
verified recovery. Use:

```text
RECOVER
```

until recovery succeeds or the hardware issue is corrected. While recovery is
required, `CANCEL` is blocked so the prepared repair data is not lost.

## Commands

- `ID <10_HEX_DIGITS>` — prepare a 40-bit ID.
- `WRITE` — write B1, B2, then B0 and verify.
- `RECOVER` — best-effort rewrite of the prepared standard data and verify.
- `CANCEL` — clear prepared data unless recovery is required.
- `HELP` — show commands.

The serial parser uses a fixed-size buffer and does not allocate dynamically.

## Tested timing

At 125 kHz the tested compatible clone-tag profile uses:

- Start gap: 30 field clocks
- Write gap: 19 field clocks
- Data zero: 24 field clocks
- Data one: 54 field clocks
- Programming hold: 900 field clocks (7.2 ms)

These values are fixed intentionally. Software cannot compensate for
insufficient field strength, poor tuning, or an incompatible/protected tag.
