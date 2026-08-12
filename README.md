# Discrete 125 kHz EM4100/T5577 CLI

An experimental Arduino Uno project for classifying 64-bit and candidate
128-bit EM4100-style transmissions, and writing standard 64-bit or extended
128-bit data to compatible T5577 tags. It uses a hand-wound coil and passive
envelope detector with no dedicated RFID reader IC, comparator, or external
coil driver.

The primary [`em4100_cli`](em4100_cli/) sketch combines reading, preparation,
writing, exact verification, and recovery in one fixed-buffer serial CLI.

## Project structure

- [`coil_tuner/`](coil_tuner/) — sweep the LC tank and locate resonance.
- [`em4100_cli/`](em4100_cli/) — primary combined reader and guarded
  64/128-bit ATA5577 writer.
- [`em4100_reader/`](em4100_reader/) — decode EM4100 frames and classify a
  repeated 64-bit frame versus a stable distinct second half; retained as a
  standalone reference reader.
- [`t5577_writer/`](t5577_writer/) — prepare, write, and verify standard
  EM4100 data; retained as a standalone reference writer.
- [`t5577_128_writer/`](t5577_128_writer/) — guarded ATA5577 B1–B4 writer
  retained as a standalone reference writer.
- [`t5577_maxblk_test/`](t5577_maxblk_test/) — advanced destructive
  capability, direct-read, RF/32, and MAXBLK diagnostics for sacrificial tags.

Each sketch folder has its own setup and usage guide. All sketches use
115200 baud.

## Parts

- Arduino Uno or compatible 16 MHz ATmega328P board
- Hand-wound air-core coil, approximately 4 cm diameter
- 2 × 100 nF ceramic capacitors (`104`)
- 1 × 1N4148 diode
- 1 × 220 Ω resistor
- 1 × 330 Ω resistor
- Breadboard, jumper wires, USB cable, and Arduino IDE

## Circuit

![Discrete 125 kHz RFID front end](circuit_diagram.svg)

1. Connect D9 through 220 Ω to the tank node.
2. Connect the coil and C1 in parallel from the tank node to GND.
3. Connect the 1N4148 anode to the tank; its cathode is the envelope node.
4. Connect A0, C2, and 330 Ω to the envelope node.
5. Connect the other ends of C2 and 330 Ω to common GND.

Before connecting A0, use a meter and preferably an oscilloscope to verify that
the envelope remains between 0 V and the Arduino supply voltage. Directly
driving the tank from D9 is experimental and limits range and programming
energy.

Regenerate the diagram with:

```powershell
python generate_circuit_diagram.py
```

## Quick start

1. Upload [`coil_tuner.ino`](coil_tuner/coil_tuner.ino) and tune near 125 kHz.
2. Upload [`em4100_cli.ino`](em4100_cli/em4100_cli.ino), open Serial Monitor at
   115200 baud with Newline enabled, and run `READ`.
3. Enter `STOP`, then use `PREP64` or `PREP128` followed by receive-only
   `VERIFY`.
4. Run `WRITE` only with one known compatible rewritable tag centered over the
   coil. Keep power connected until exact verification or recovery finishes.

## Safety and limitations

- Use only tags and systems you own or are authorized to test.
- Factory-programmed EM4100/EM4102 tags are read-only.
- Writing is destructive; power loss or tag movement can leave a tag
  unreadable. Keep one tag centered until verification or recovery finishes.
- Keep every other tag away from the coil during writing and verification.
- Recovery is best-effort and cannot guarantee repair after an interrupted or
  weak-field write. Resetting or powering off the Arduino loses recovery data.

## License

[MIT](LICENSE)
