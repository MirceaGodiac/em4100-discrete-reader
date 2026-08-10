"""Generate a publication-ready SVG of the discrete 125 kHz RFID circuit."""

from pathlib import Path


OUTPUT = Path(__file__).with_name("circuit_diagram.svg")


def line(x1, y1, x2, y2, css="wire"):
    return f'<line class="{css}" x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}"/>'


def text(x, y, value, css="label", anchor="middle"):
    return (
        f'<text class="{css}" x="{x}" y="{y}" '
        f'text-anchor="{anchor}">{value}</text>'
    )


parts = [
    """<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="600"
viewBox="0 0 1200 600" role="img" aria-labelledby="title description">
<title id="title">125 kHz RFID front end</title>
<desc id="description">Arduino D9 drives a parallel LC tank through 220 ohms.
A 1N4148 diode, 100 nanofarad capacitor and 330 ohm resistor form the envelope
detector connected to A0.</desc>
<style>
  .background { fill: white; }
  .wire, .component, .block {
    stroke: black; stroke-width: 3; fill: none;
    stroke-linecap: square; stroke-linejoin: miter;
  }
  .node { fill: black; }
  .title { font: 22px Arial, sans-serif; }
  .label { font: 17px Arial, sans-serif; }
  .value { font: 15px Arial, sans-serif; }
  .pin { font: 15px Arial, sans-serif; }
</style>
<rect class="background" width="1200" height="600"/>
""",
    text(50, 45, "125 kHz RFID front end", "title", "start"),
    # Controller.
    '<rect class="block" x="60" y="200" width="180" height="250"/>',
    text(150, 245, "U1", "label"),
    text(150, 275, "Arduino Uno", "value"),
    text(215, 305, "D9", "pin", "end"),
    text(150, 220, "A0", "pin"),
    text(215, 425, "GND", "pin", "end"),
    # A0 wire routed above the signal path.
    line(150, 200, 150, 120),
    line(150, 120, 830, 120),
    line(830, 120, 830, 300),
    # Carrier output and R1.
    line(240, 300, 330, 300),
    text(280, 280, "125 kHz", "value"),
    '<path class="component" d="M330 300 L345 285 L370 315 L395 285 L420 315 L445 285 L460 300"/>',
    text(395, 255, "R1", "label"),
    text(395, 345, "220 Ω", "value"),
    line(460, 300, 520, 300),
    '<circle class="node" cx="520" cy="300" r="5"/>',
    # Parallel LC tank.
    line(520, 300, 520, 360),
    '<path class="component" d="M520 360 C495 372 495 388 520 400 C545 412 545 428 520 440"/>',
    line(520, 440, 520, 500),
    text(480, 390, "L1", "label"),
    text(475, 415, "coil", "value"),
    line(520, 300, 630, 300),
    line(630, 300, 630, 375),
    line(605, 375, 655, 375),
    line(605, 395, 655, 395),
    line(630, 395, 630, 500),
    text(675, 380, "C1", "label", "start"),
    text(675, 405, "100 nF", "value", "start"),
    # Envelope detector.
    line(630, 300, 700, 300),
    '<path class="component" d="M700 270 L700 330 L750 300 Z"/>',
    line(750, 270, 750, 330),
    line(750, 300, 830, 300),
    text(725, 250, "D1", "label"),
    text(725, 355, "1N4148", "value"),
    '<circle class="node" cx="830" cy="300" r="5"/>',
    line(830, 300, 910, 300),
    line(910, 300, 910, 375),
    line(885, 375, 935, 375),
    line(885, 395, 935, 395),
    line(910, 395, 910, 500),
    text(870, 380, "C2", "label", "end"),
    text(870, 405, "100 nF", "value", "end"),
    line(830, 300, 1030, 300),
    line(1030, 300, 1030, 350),
    '<path class="component" d="M1030 350 L1015 365 L1045 390 L1015 415 L1045 440 L1015 465 L1030 480"/>',
    line(1030, 480, 1030, 500),
    text(1070, 400, "R2", "label", "start"),
    text(1070, 425, "330 Ω", "value", "start"),
    # Common ground.
    line(240, 420, 280, 420),
    line(280, 420, 280, 500),
    line(280, 500, 1030, 500),
    line(655, 500, 655, 525),
    line(625, 525, 685, 525),
    line(637, 538, 673, 538),
    line(647, 551, 663, 551),
    "</svg>",
]

OUTPUT.write_text("\n".join(parts), encoding="utf-8")
print(OUTPUT)
