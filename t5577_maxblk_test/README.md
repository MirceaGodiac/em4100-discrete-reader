# T5577 MAXBLK=1 controlled capability test

> This is an advanced destructive diagnostic, not the normal reader/writer.
> Use [`em4100_cli`](../em4100_cli/) for routine reading and guarded 64/128-bit
> programming.

This isolated Arduino Uno lab sketch answers one question: can a writable tag
that currently emits the prepared standard 64-bit EM4100 cycle with
`MAXBLK=2` accept `B0=0x00148020` and then emit B1 as a 32-bit cycle?

The sketch always attempts to restore `B0=0x00148040` (`MAXBLK=2`) after the
test. It uses the existing circuit: D9 supplies the 125 kHz carrier/downlink
gaps and A0 reads the passive envelope detector.

## Safety

**Use only one sacrificial writable tag that you own or are explicitly
authorized to test. Keep it centered and still. A weak field, movement, reset,
or power loss during a write can leave the tag unreadable. Direct Arduino pin
drive is experimental and may not supply enough programming energy. Verify
that A0 always remains between 0 V and the Arduino supply voltage.**

The recovery-required flag and prepared B1/B2 values exist only in Arduino
RAM. **A reset or power loss erases that recovery state and its prepared
data.** Do not reset or disconnect the Uno while recovery is required.

Every destructive command prints a warning and waits two seconds. Password
and lock bits remain clear.

## Setup and commands

Upload `t5577_maxblk_test.ino` for `arduino:avr:uno`, then open Serial Monitor
at 115200 baud with Newline enabled.

```text
ID <10_HEX_DIGITS>
ID2 <10_HEX_DIGITS>
OBSERVE
READ <page 0|1> <block 0-7>
DUMP
TESTMAX1
TESTRF32
TEST128
RECOVER
HELP
```

- `ID` accepts exactly ten hexadecimal digits. It computes the complete
  parity-valid 64-bit EM4100 frame and derives B1 and B2; no credential is
  hardcoded.
- `OBSERVE` listens without transmitting anything and reports which bit rate and
  period the tag actually uses. It is the only command that assumes nothing.
- `READ` issues one T5577 direct access command and reports the block the tag
  answers with. It writes nothing.
- `DUMP` reads page 0 blocks 0-7 and page 1 blocks 0-3. Page 1 blocks 1 and 2
  hold the manufacturer traceability data, which a genuine ATA5577C exposes and
  many clones leave empty.
- `TESTMAX1` runs the controlled test and unconditional restore workflow.
- `ID2` prepares a second EM4100 frame and derives B3 and B4 from it. It must
  differ from `ID`.
- `TESTRF32` writes only `B0=0x00088040`, which changes the data bit rate field
  from RF/64 to RF/32 and leaves modulation and MAXBLK alone, then looks for the
  same 64-bit frame arriving at the faster rate.
- `TEST128` writes the second frame into B3 and B4, then `B0=0x00148080`
  (MAXBLK=4), and looks for both frames back to back as one 128-bit cycle.
- `RECOVER` rewrites prepared B1, B2, and `B0=0x00148040`, verifies the
  64-bit cycle, and tries up to three times.
- `HELP` prints the command list.

If restoration fails, the RAM recovery lock retains the prepared data and
allows only `RECOVER`, `OBSERVE`, `READ`, `DUMP`, and `HELP`. The read commands
stay available because they write nothing and help diagnose a failed restore.

## Identifying an unknown tag with OBSERVE

Every other command in this sketch assumes the tag streams a 64-bit EM4100 frame
at RF/64. That assumption is built into the sample rate, the capture width, and
the restore path, and it is exactly the assumption to drop when an unfamiliar tag
behaves oddly. `OBSERVE` drops it.

`OBSERVE` never gaps the field and never writes, so it is safe on a tag that
must not be modified. It runs six three-second captures, scoring period lengths
of 32, 64, and 128 bits against both RF/64 and RF/32, and prints the cycle count
each one reached next to the ceiling that a perfect capture at that combination
would give:

```text
  RF/64 width=128 cycles=45/45 value=0xFFF803F91391082D155C0F0FFFFFFFFF strong=99% maxDiff=80
    valid EM4100 frame ID=03E8B77AB5 starts at bit 13, envelope inverted
    the other 64 bits are 0x7E1E000000000000
```

The tag's real format is the row whose count is closest to its ceiling. A tag
with a genuine 128-bit period reaches its ceiling at width 128 while 64 and 32
fall well short, because no shorter window repeats. The reverse does not hold:
a 64-bit tag also scores at width 128, since two copies of a 64-bit frame fill a
128-bit window exactly. That case is called out explicitly with `both halves are
equal, so the true period is 64 bits`.

Reading at the wrong rate does not produce a clean shorter or longer period, it
produces a decimated stream with no stable period at all. Scanning both rates is
therefore what separates "this tag is unusual" from "this tag is being sampled
wrong". Cycle counts saturate at 255, so a ceiling above that is shown as 255.

### Locating the frame inside the window

A captured window starts wherever the capture happened to start, and the
envelope detector can resolve the two Manchester levels either way round, so a
frame inside a window sits at an unknown rotation in an unknown polarity. Every
64- and 128-bit observation is therefore searched at all rotations in both
polarities for a parity-valid EM4100 frame.

Validation re-encodes the ten digits read out of each candidate and requires all
64 bits to match, which checks the nine-bit header, the ten row parities, the
four column parities and the stop bit in a single comparison. A false positive
would need a full 64-bit coincidence, so a reported ID can be trusted even
though the alignment was guessed.

Finding a valid frame in one half of a 128-bit period, with no 64-bit period,
means the tag sends one EM4100 frame per 128 bit times rather than per 64. The
other half is reported verbatim. If it demodulates to a constant while `strong`
stays near 100 percent, those bit times carry an all-zero or all-one payload
rather than silence, because an unmodulated field produces windows too weak to
sustain a run and the cycle count would have collapsed instead.

## Reading blocks back

`READ` and `DUMP` send `opcode`, a clear lock bit, and the three address bits,
with no data field and **no password**. Omitting the password matters: sending a
password to a tag that has none enabled is the documented way to lock a T5577,
and this sketch never does it.

A direct access answer is a bare repeating 32-bit block with no preamble, so its
bit alignment is unknown. Each answer is therefore also reported as its
numerically smallest rotation, which makes comparison alignment independent.
Two different configuration values cannot collide this way unless they are
rotations of each other, and `0x00148040` and `0x00148020` are not. `READ` also
prints all 32 rotations so the true value can be read off directly; for a basic
mode configuration block it is the rotation with eleven leading zero bits.

`no stable 32-bit answer` means no 32-bit period repeated at least three times
in a healthy capture. Either the tag ignored the direct access command and stayed
in regular read mode, or it does not implement direct access at all.

To tell those apart, check the reported value against the tag's own 64-bit frame.
A tag that ignored the command keeps streaming that frame, so any 32-bit value
reported is an arbitrary window of it and lines up with the frame at some bit
offset. Every read is therefore also reported as the first 32 bits received after
the command, per phase and polarity. That value keeps its alignment to the end of
the downlink, so an answer that is sent once and not repeated shows up there even
though it cannot satisfy the three-cycle periodicity requirement.

The read capture is deliberately short, 200 ms, for the same reason. A one second
capture buries a brief answer under the regular read stream that follows it.

## Why two configuration tests

`TESTMAX1` and `TESTRF32` write different fields of block 0 to answer different
questions.

MAXBLK controls how many blocks the tag streams, so `TESTMAX1` asks whether that
specific field is implemented. An `UNCHANGED` result there is ambiguous: the write
may have been rejected, or it may have been stored and ignored.

The data bit rate field is active in basic mode on every documented T5577, so
`TESTRF32` asks the broader question of whether a block 0 write changes the tag's
behaviour at all. `ACCEPTED` proves block 0 is writable and narrows the earlier
result to MAXBLK alone not being implemented. `UNCHANGED` means two independent
fields of block 0 both had no effect, which is what a fixed-function EM4100
emulator that only accepts writes to blocks 1 and 2 would do.

Both tests are recoverable. The downlink timing is measured in carrier field
clocks and does not depend on the rate the tag answers at, so a tag that switches
to RF/32 still accepts the write that puts it back.

## Going for 128 bits

MAXBLK=4 streams blocks 1 to 4, which is 128 bits. Raising MAXBLK alone is not a
valid test of this, because if blocks 3 and 4 happen to hold copies of blocks 1
and 2, a 128-bit stream is bit for bit identical to a 64-bit one. A tag whose
address decoding wraps onto its real storage would produce exactly that.

`TEST128` therefore writes a second distinct EM4100 frame into B3 and B4 before
raising MAXBLK, and re-checks the 64-bit output in between:

- `ALIAS64` proving the first frame means addresses 3 and 4 are separate from
  blocks 1 and 2.
- `ALIASED` means the second frame now streams as 64 bits on its own, so writing
  address 3 and 4 landed on blocks 1 and 2. The tag has fewer real blocks than
  its address field can express.
- `ACCEPTED` means the full 128-bit cycle of both frames was proven, so MAXBLK=4
  is implemented and blocks 3 and 4 are real storage.
- `UNCHANGED` means the first frame still streams as 64 bits with distinct data
  now sitting in blocks 3 and 4, which rules out the aliasing explanation and
  leaves MAXBLK simply not being applied.

The 128-bit periodicity check uses the same rolling comparison as the 64-bit one,
held as two 64-bit halves where the high half carries the older bits. Matches are
exact up to rotation of the whole 128-bit word.

Two things keep that check inside the 64 us sample interval, and both matter
because exceeding it corrupts a capture rather than slowing it down. The top bit
of each half is read as a single byte instead of shifting a 64-bit value by 63.
And only the normal polarity is tracked at 128 bits: the inverse candidate is fed
the complemented bit, so its register is always the exact complement of the normal
one and its periodicity is identical, making it pure redundancy. The matcher
compares the complement of the target as well, so an inverted signal is still
found.

Any capture that reports a non-zero `late` count dropped samples and its cycle
counts mean nothing. `TEST128` says so explicitly rather than letting the 64-bit
fallback stand in for a wide search that never really ran. Note that a healthy
64-bit result is still conclusive on its own: an unbroken 64-bit period of the
first frame is incompatible with a 128-bit stream containing the second one,
whatever blocks 3 and 4 hold.

`TEST128` restores B1, B2 and B0 rather than B0 alone, because unlike the other
tests it writes blocks other than 0. An `ACCEPTED` result can be confirmed
independently by the `em4100_reader` sketch, which classifies a stable distinct
second 64-bit segment as a 128-bit candidate.

## Test workflow

Before any write, the sketch must observe the prepared 64-bit frame as a
continuous repeated cycle at least three times with `missed=0 late=0`.
Otherwise it aborts without writing.

After setting the RAM recovery lock, it writes only
`B0=0x00148020`, power-cycles the field, and looks for B1 as a continuously
repeated 32-bit cycle. It then always writes `B0=0x00148040`, power-cycles
again, and requires three healthy repeats of the prepared full frame before
clearing the lock.

Each capture reports ADC minimum/maximum, missed conversions, late reads, and
the number of consecutive cycle periods matched. Sampling is synchronous at
64 us (eight 125 kHz carrier periods). Eight bit phases and both polarities are
tested. Matches are exact up to cycle rotation; adjacent phase candidates are
not added together.

## Result meanings

- `ACCEPTED` means the exact prepared B1 was observed as a continuous
  rotation-equivalent 32-bit cycle at least three times with healthy
  sampling. It proves acceptance for this tag and coupling setup only.
- `UNCHANGED` means the 32-bit test was not proven, but a separate healthy
  capture observed the original prepared 64-bit cycle at least three times.
  The attempted MAXBLK=1 configuration did not produce the expected change.
- `INCONCLUSIVE` means neither the expected repeated 32-bit B1 nor the
  original repeated 64-bit frame was proven in healthy captures. It does not
  establish whether the configuration write was accepted.
- `RECOVERY FAILED` means three B1/B2/B0 rewrite-and-verify attempts did not
  prove the prepared 64-bit cycle. The tag may still be programmed correctly,
  partially programmed, out of coupling range, protected, incompatible, or
  unreadable. Keep power on so the RAM recovery data remains available,
  correct the physical setup, and run `RECOVER` again.

The capability result and restoration result are separate. An `ACCEPTED`,
`UNCHANGED`, or `INCONCLUSIVE` line is not a safe completion unless it is
followed by `RESTORED` or a successful `RECOVER`.

## Fixed write timing

The 125 kHz profile tested on the development hardware is intentionally fixed:

- Start gap: 30 field clocks
- Write gap: 19 field clocks
- Zero: 24 field clocks
- One: 54 field clocks
- Programming hold: 900 field clocks (7.2 ms)

Software cannot compensate for poor tuning, insufficient programming field,
a read-only EM4100/EM4102 tag, or a protected/incompatible T5577-family tag.

## Downlink framing cross-check

The framing and timing above were checked against three independent
implementations. All of them send the same sequence: start gap, two opcode bits,
a lock bit, 32 data bits, three address bits, then a programming hold. Each data
bit is a field-on interval whose length carries the value, followed by a write
gap.

| Source | Start gap | Write gap | Zero | One | Program hold |
| --- | --- | --- | --- | --- | --- |
| This sketch | 30 | 19 | 24 | 54 | 900 |
| Flipper `lib/lfrfid/tools/t5577.c` | 30 | 18 | 24 | 56 | 700 |
| Proxmark3 `armsrc/lfops.c` generic | 31 | 20 | 18 | 50 | n/a |
| MultiKey `sendOpT5557` | 30 | 19 | 24 | 54 | 15 ms |

The ATA5577C limits quoted in the Proxmark3 source are a start gap of 8 to 50
and a write gap of 8 to 20 field clocks, so every value here is in range. The
zero and one lengths differ between projects but always keep roughly a 32 field
clock separation, which is what the chip discriminates on.

Two differences from the references are deliberate and known to be harmless:
this sketch does not send Flipper's trailing reset sequence after a write, and
it holds the programming field for 7.2 ms rather than Flipper's 5.6 ms or
MultiKey's 15 ms. The tag is power-cycled after every write instead, which both
ends the command and forces the configuration register to reload.
