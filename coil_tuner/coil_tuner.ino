/*
 * LC tank resonance finder for Arduino Uno / ATmega328P.
 *
 * Sweeps the carrier frequency and measures the diode-envelope DC voltage on
 * A0. Run without a tag. The largest A0_avg value approximates the resonance
 * of the hand-wound coil in parallel with the tank capacitor.
 *
 * Existing circuit:
 *   D9 -- 220R -- tank node
 *   tank node -- coil -- GND
 *   tank node -- 100nF -- GND
 *   tank node -- 1N4148 anode; cathode -- envelope node
 *   envelope node -- A0
 *   envelope node -- 100nF -- GND
 *   envelope node -- 330R -- GND
 */

#include <Arduino.h>

void setCarrier(uint8_t ocr) {
  TCCR1A = _BV(COM1A0);
  TCCR1B = _BV(WGM12) | _BV(CS10); // CTC, prescaler 1
  TCNT1 = 0;
  OCR1A = ocr;
}

long averageEnvelope() {
  long sum = 0;
  for (uint16_t i = 0; i < 200; i++) {
    sum += analogRead(A0);
  }
  return sum / 200;
}

void setup() {
  Serial.begin(115200);
  pinMode(9, OUTPUT);
  Serial.println(F("OCR1A\tfreq_kHz\tA0_avg"));

  long peak = -1;
  long at125k = -1;
  uint8_t peakOcr = 0;

  for (int16_t ocr = 95; ocr >= 45; ocr--) {
    setCarrier((uint8_t)ocr);
    delay(30);

    const long envelope = averageEnvelope();
    const float frequencyKHz = 8000.0 / (ocr + 1);

    Serial.print(ocr);
    Serial.print('\t');
    Serial.print(frequencyKHz, 1);
    Serial.print('\t');
    Serial.println(envelope);

    if (envelope > peak) {
      peak = envelope;
      peakOcr = (uint8_t)ocr;
    }
    if (ocr == 63) {
      at125k = envelope;
    }
  }

  Serial.println(F("--- SUMMARY ---"));
  Serial.print(F("Peak A0_avg = "));
  Serial.print(peak);
  Serial.print(F(" at OCR1A="));
  Serial.print(peakOcr);
  Serial.print(F(" -> "));
  Serial.print(8000.0 / (peakOcr + 1), 1);
  Serial.println(F(" kHz"));

  Serial.print(F("A0_avg at 125.0 kHz (OCR1A=63) = "));
  Serial.println(at125k);

  if (peak > 0) {
    Serial.print(F("125 kHz envelope is approximately "));
    Serial.print((uint8_t)(100.0 * at125k / peak));
    Serial.println(F("% of the measured peak."));
  }

  Serial.println(F("Aim for a broad peak centered near OCR1A=63."));
  Serial.println(F("Judge signal quality by tag/no-tag separation, not absolute ADC level."));
}

void loop() {}
