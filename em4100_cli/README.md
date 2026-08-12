# Unified EM4100/T5577 CLI

`em4100_cli.ino` is the primary Arduino Uno sketch for this repository. It
combines Manchester RF/64 reading, 64/128-bit classification, standard EM4100
writing, extended 128-bit writing, exact receive-side verification, and
verified standard recovery in one serial command interface.

The sketch uses the existing hardware:

- D9 generates the 125 kHz carrier and ATA5577 downlink gaps.
- A0 samples the passive envelope detector.
- Serial Monitor runs at 115200 baud with Newline enabled.

Only use tags and systems you own or are authorized to test. Factory EM4100
and EM4102 tags are read-only. Writing requires an unlocked compatible
T5577/ATA5577; extended writing also requires real page-0 blocks B1 through B4.

## Setup

1. Tune the circuit with [`coil_tuner`](../coil_tuner/).
2. Verify that A0 always remains between 0 V and the Arduino supply voltage.
3. Upload `em4100_cli.ino` for `arduino:avr:uno`.
4. Open Serial Monitor at 115200 baud with Newline enabled.
5. Keep only one tag near the coil during reading, writing, and verification.

The sketch starts idle so no command can accidentally write a tag.

## Commands

```text
READ
STOP
PREP64 <10_HEX_DIGITS>
PREP128 <10_HEX_DIGITS> <16_HEX_DIGITS>
VERIFY
WRITE
RECOVER
CANCEL
STATUS
HELP
```

- `READ` starts continuous Manchester RF/64 decoding and reports once per
  second. While reading, enter `STOP` before any other command.
- `STOP` returns the CLI to idle.
- `PREP64` encodes a 40-bit ID into one complete parity-valid EM4100 frame.
- `PREP128` prepares the same encoded base followed by the exact supplied
  64-bit suffix. For reliable repeated header synchronization, the suffix must
  end in an even hexadecimal digit.
- `VERIFY` is receive-only. It sends no ATA5577 downlink and changes no tag
  memory.
- `WRITE` writes the prepared profile and verifies the exact emitted stream.
- `RECOVER` restores the prepared base as standard 64-bit EM4100 data.
- `CANCEL` clears prepared RAM data when recovery is not required.
- `STATUS` prints the current mode, prepared data, and recovery lock.
- `HELP` prints the command summary.

No credential values are built into the sketch.

## Reading

Run:

```text
READ
```

The reader validates the nine-bit header, all ten row parity bits, four column
parity bits, and stop bit. It then captures the next 64 bits. After three
de-duplicated stable cycles it reports one of:

- `Standard repeating 64-bit` when the following bits reproduce the encoded
  EM4100 frame.
- `128-bit candidate` with the decoded ID, encoded base, and exact suffix when
  the following 64 bits are stable and different.
- `Length unknown` when a valid base is present but no stable suffix is proved.

Healthy capture diagnostics show `missed=0 late=0`. Enter `STOP` on its own
line to leave reading mode.

## Standard 64-bit write

```text
PREP64 <10_HEX_DIGITS>
VERIFY
WRITE
```

`VERIFY` is useful before writing when the source tag is available. A standard
verification requires three healthy observations of:

```text
encoded EM4100 frame || identical encoded EM4100 frame
```

This is stronger than merely seeing the requested ID: one decoder phase must
prove three consecutive prepared frame repetitions. Passive observation proves
the emitted repeating bits, not the tag's internal MAXBLK value.

The writer programs:

- B1: upper 32 bits of the encoded frame
- B2: lower 32 bits of the encoded frame
- B0: `0x00148040` for Manchester RF/64 with MAXBLK=2

B0 is written last.

## Extended 128-bit write

First capture the source with `READ`, then use its decoded 10-hex-digit ID and
exact 16-hex-digit suffix:

```text
PREP128 <10_HEX_DIGITS> <16_HEX_DIGITS>
VERIFY
WRITE
```

Do not use the printed `base64` value as the ID argument. `PREP128` expects the
decoded 40-bit ID and computes the complete parity-valid base itself.

The writer programs:

- B1/B2: the encoded 64-bit EM4100 base
- B3/B4: the exact supplied 64-bit suffix
- B0: `0x00148080` for Manchester RF/64 with MAXBLK=4

Programming order is B1, B2, B3, B4, then B0. Writing B0 last prevents the tag
from transmitting uninitialized extended blocks. Verification requires three
consecutive healthy exact `base || suffix` cycles from one decoder phase.

If the suffix equals the base, the emitted bits can be verified but passive
observation cannot distinguish a genuine 128-bit period with duplicate halves
from an ordinary repeating 64-bit period.

## Recovery and safety

Writing is destructive. Weak coupling, tag movement, reset, power loss, or an
incompatible/protected tag can leave it unreadable. Direct D9 tank drive is
experimental and may not provide enough programming energy.

Before the first tag mutation, the sketch sets a recovery-required lock. If
final verification fails, it automatically makes up to three recovery
attempts. Each attempt writes B1, B2, then `B0=0x00148040` and requires three
consecutive healthy exact `frame || frame` observations.

If recovery still fails, only `RECOVER`, `STATUS`, and `HELP` are accepted.
Correct the tag position without resetting the Uno, then run:

```text
RECOVER
```

Prepared data and the recovery lock exist only in Arduino RAM. Resetting,
uploading another sketch, disconnecting USB, or losing power erases that state.
Recovery is best-effort and cannot unlock a protected tag or identify which
physical tag answered verification.

## Timing and implementation

Timer1 generates 125 kHz on D9. Timer2 starts an ADC conversion every 64 us,
exactly eight carrier periods apart. Both timers start from one synchronized
prescaler boundary using `GTCCR`, preventing carrier ripple from aliasing into
false data.

The tested ATA5577 downlink profile is:

- power-up: 400 field clocks
- start gap: 30 field clocks
- write gap: 19 field clocks
- zero: 24 field clocks
- one: 54 field clocks
- programming hold: 900 field clocks

Every page-0 write sends opcode `10`, a clear lock bit, 32 data bits most
significant bit first, and the three-bit block address most significant bit
first. Password mode and lock bits remain clear.

The sketch uses fixed-size buffers and no dynamic allocation. Continuous
reading and verification share one decoder bank to fit comfortably within the
Arduino Uno's SRAM.
