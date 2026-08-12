/*
 * Controlled T5577 MAXBLK capability test for Arduino Uno / ATmega328P.
 * D9: 125 kHz carrier/downlink gaps. A0: passive envelope detector.
 * Use only one sacrificial writable tag that you are authorized to test.
 */

#include <Arduino.h>
#include <string.h>

const uint8_t PIN_CARRIER = 9;
const uint32_t CONFIG_MAXBLK1 = 0x00148020UL;
const uint32_t CONFIG_MAXBLK2 = 0x00148040UL;
// Same modulation and MAXBLK as CONFIG_MAXBLK2, only the data bit rate field
// changes from RF/64 to RF/32. Unlike MAXBLK this field is active in basic mode
// on every documented T5577, and a tag that honours it halves its bit period,
// which is a positive observation rather than an absence of one.
const uint32_t CONFIG_RF32 = 0x00088040UL;
// MAXBLK occupies bits 7 to 5, so 4 streams blocks 1 to 4 as 128 bits.
const uint32_t CONFIG_MAXBLK4 = 0x00148080UL;

const uint16_t POWER_UP_CLOCKS = 400;
const uint8_t START_GAP_CLOCKS = 30;
const uint8_t WRITE_GAP_CLOCKS = 19;
const uint8_t ZERO_CLOCKS = 24;
const uint8_t ONE_CLOCKS = 54;
const uint16_t PROGRAM_HOLD_CLOCKS = 900;
// Proxmark3 waits this long after a direct access command before sampling.
const uint16_t READ_SETTLE_CLOCKS = 137;

const uint8_t MIN_LEVEL_DIFF = 2;
const uint8_t REQUIRED_CYCLES = 3;
const uint32_t CAPTURE_SAMPLES = 46875UL;  // Three seconds at 64 us/sample.
// A 32-bit block at RF/64 repeats every 16.4 ms, so 200 ms holds twelve cycles.
// Staying short matters: if a tag answers briefly and then returns to regular
// read mode, a long capture buries the answer under the regular read stream.
const uint32_t READ_CAPTURE_SAMPLES = 3125UL;

struct RawCandidate {
  // A 128-bit shift register held as two halves. The stream arrives most
  // significant bit first, so high holds the older 64 bits of the window.
  union CycleValue {
    uint32_t bits32;
    uint64_t bits64;
    struct {
      uint64_t low;
      uint64_t high;
    } bits128;
    // AVR is little endian, so byte 7 is the top of low and byte 15 the top of
    // high. Reading those bytes directly keeps the 128-bit path inside the
    // 64 us sample interval; two 64-bit shifts per bit did not fit.
    uint8_t bytes[16];
  } rolling, proven;
  // The first 32 bits of the first continuous strong run in a capture. Unlike
  // the periodic value this keeps its alignment to the end of the downlink
  // command, so it also shows an answer that never repeats.
  uint32_t firstBits;
  bool haveFirstBits;
  uint8_t fill;
  uint8_t cycleProgress;
  uint8_t cycles;
  uint8_t provenCycles;
};

struct CaptureResult {
  uint16_t adcMin;
  uint16_t adcMax;
  uint16_t missed;
  uint16_t late;
  uint16_t maxDifference;
  uint8_t strongPercent;
  uint8_t matches;
};

struct SavedCaptureHardware {
  uint8_t timsk0;
  uint8_t tccr2a;
  uint8_t tccr2b;
  uint8_t timsk2;
  uint8_t ocr2a;
  uint8_t tcnt2;
  uint8_t admux;
  uint8_t adcsra;
  uint8_t adcsrb;
  uint8_t didr0;
};

RawCandidate candidates[16];  // Eight sample phases, both polarities.
SavedCaptureHardware savedHardware;

uint16_t sampleRing[8];
uint16_t firstHalfSum;
uint16_t secondHalfSum;
uint8_t ringPos;
uint8_t ringFill;
// Eight samples per bit decodes RF/64, four decodes RF/32. The half window sums
// fewer samples at RF/32, so the strong window threshold scales with it.
uint8_t bitSamples;
uint8_t halfSamples;
uint8_t ringMask;
uint8_t activeCandidates;
uint8_t matchThreshold;
uint32_t sampleCount;
uint16_t captureMin;
uint16_t captureMax;
volatile uint16_t missedSamples;
volatile uint16_t lateSamples;
uint16_t maxWindowDifference;
uint32_t strongWindows;
uint32_t totalWindows;

uint64_t captureTarget;
uint64_t captureTargetHigh;
uint8_t captureWidth;

uint8_t preparedId[5];
uint64_t preparedFrame;
uint32_t preparedBlock1;
uint32_t preparedBlock2;
bool prepared;

// A second EM4100 frame occupying blocks 3 and 4. Distinct content there is what
// makes a 128-bit stream distinguishable from a 64-bit one.
uint8_t preparedId2[5];
uint64_t preparedFrame2;
uint32_t preparedBlock3;
uint32_t preparedBlock4;
bool prepared2;

bool recoveryRequired;

char serialBuffer[24];
uint8_t serialLength;
bool discardSerialLine;

ISR(TIMER2_COMPA_vect) {
  if (ADCSRA & _BV(ADSC)) {
    missedSamples++;
  } else {
    if (ADCSRA & _BV(ADIF)) lateSamples++;
    ADCSRA |= _BV(ADSC);
  }
}

void carrierOn() {
  TIFR1 = _BV(OCF1A);
  TCCR1A |= _BV(COM1A0);
}

void carrierOff() {
  TCCR1A &= ~_BV(COM1A0);
  PORTB &= ~_BV(PORTB1);
}

void waitFieldClocks(uint16_t clocks) {
  uint32_t us = (uint32_t)clocks * 8UL;
  while (us > 16000UL) {
    delayMicroseconds(16000);
    us -= 16000UL;
  }
  delayMicroseconds((uint16_t)us);
}

void fieldGap(uint8_t clocks) {
  carrierOff();
  waitFieldClocks(clocks);
  carrierOn();
}

void sendDownlinkBit(uint8_t bit) {
  waitFieldClocks(bit ? ONE_CLOCKS : ZERO_CLOCKS);
  fieldGap(WRITE_GAP_CLOCKS);
}

void writePage0Block(uint8_t block, uint32_t data) {
  waitFieldClocks(POWER_UP_CLOCKS);
  fieldGap(START_GAP_CLOCKS);
  sendDownlinkBit(1);
  sendDownlinkBit(0);  // Page 0 write; no password mode.
  sendDownlinkBit(0);  // Lock bit remains clear.
  for (int8_t bit = 31; bit >= 0; bit--) {
    sendDownlinkBit((data >> bit) & 1U);
  }
  sendDownlinkBit((block >> 2) & 1U);
  sendDownlinkBit((block >> 1) & 1U);
  sendDownlinkBit(block & 1U);
  waitFieldClocks(PROGRAM_HOLD_CLOCKS);
}

// Direct access read: opcode, lock bit, then only the three address bits.
// No password is ever sent, so this cannot trigger the password-read failure
// mode that locks a tag which has no password enabled. The absent 32-bit data
// field is what distinguishes a read from a write of the same block.
void sendDirectAccessCommand(uint8_t page, uint8_t block) {
  waitFieldClocks(POWER_UP_CLOCKS);
  fieldGap(START_GAP_CLOCKS);
  sendDownlinkBit(1);
  sendDownlinkBit(page & 1U);
  sendDownlinkBit(0);  // Lock bit remains clear.
  sendDownlinkBit((block >> 2) & 1U);
  sendDownlinkBit((block >> 1) & 1U);
  sendDownlinkBit(block & 1U);
  waitFieldClocks(READ_SETTLE_CLOCKS);
}

void powerCycleField() {
  carrierOff();
  delay(100);
  carrierOn();
  delay(20);
}

void writeFreshBlock(uint8_t block, uint32_t data) {
  powerCycleField();
  noInterrupts();
  writePage0Block(block, data);
  interrupts();
}

void appendFrameBit(uint64_t &frame, uint8_t bit) {
  frame = (frame << 1) | (bit & 1U);
}

uint64_t encodeEm4100(const uint8_t id[5]) {
  uint64_t frame = 0;
  uint8_t columnParity[4] = {0, 0, 0, 0};
  for (uint8_t i = 0; i < 9; i++) appendFrameBit(frame, 1);
  for (uint8_t row = 0; row < 10; row++) {
    const uint8_t value = id[row >> 1];
    const uint8_t nibble = (row & 1U) ? (value & 0x0F) : (value >> 4);
    uint8_t rowParity = 0;
    for (uint8_t col = 0; col < 4; col++) {
      const uint8_t bit = (nibble >> (3 - col)) & 1U;
      appendFrameBit(frame, bit);
      rowParity ^= bit;
      columnParity[col] ^= bit;
    }
    appendFrameBit(frame, rowParity);
  }
  for (uint8_t col = 0; col < 4; col++) {
    appendFrameBit(frame, columnParity[col]);
  }
  appendFrameBit(frame, 0);
  return frame;
}

void resetCandidateRun(RawCandidate &candidate) {
  candidate.rolling.bits128.low = 0;
  candidate.rolling.bits128.high = 0;
  candidate.fill = 0;
  candidate.cycleProgress = 0;
  candidate.cycles = 0;
}

void clearCandidate(RawCandidate &candidate) {
  resetCandidateRun(candidate);
  candidate.proven.bits128.low = 0;
  candidate.proven.bits128.high = 0;
  candidate.provenCycles = 0;
  candidate.firstBits = 0;
  candidate.haveFirstBits = false;
}

void preserveCandidate32(RawCandidate &candidate) {
  if (candidate.cycles > candidate.provenCycles) {
    candidate.proven.bits32 = candidate.rolling.bits32;
    candidate.provenCycles = candidate.cycles;
  }
}

void preserveCandidate64(RawCandidate &candidate) {
  if (candidate.cycles > candidate.provenCycles) {
    candidate.proven.bits64 = candidate.rolling.bits64;
    candidate.provenCycles = candidate.cycles;
  }
}

inline __attribute__((always_inline))
void feedRawBit32(RawCandidate &candidate, uint8_t bit) {
  if (candidate.fill < 32) {
    candidate.rolling.bits32 =
        (candidate.rolling.bits32 << 1) | (bit & 1U);
    if (++candidate.fill == 32) {
      candidate.cycleProgress = 0;
      candidate.cycles = 1;
      if (!candidate.haveFirstBits) {
        candidate.firstBits = candidate.rolling.bits32;
        candidate.haveFirstBits = true;
      }
      preserveCandidate32(candidate);
    }
    return;
  }

  const uint8_t expected = (candidate.rolling.bits32 >> 31) & 1U;
  candidate.rolling.bits32 =
      (candidate.rolling.bits32 << 1) | (bit & 1U);
  if (bit != expected) {
    // The mismatching bit completes a new rolling 32-bit candidate period.
    candidate.cycleProgress = 0;
    candidate.cycles = 1;
    return;
  }

  if (++candidate.cycleProgress == 32) {
    candidate.cycleProgress = 0;
    if (candidate.cycles != 255) candidate.cycles++;
    preserveCandidate32(candidate);
  }
}

inline __attribute__((always_inline))
void feedRawBit64(RawCandidate &candidate, uint8_t bit) {
  if (candidate.fill < 64) {
    candidate.rolling.bits64 =
        (candidate.rolling.bits64 << 1) | (bit & 1U);
    if (++candidate.fill == 64) {
      candidate.cycleProgress = 0;
      candidate.cycles = 1;
      preserveCandidate64(candidate);
    }
    return;
  }

  const uint8_t expected = (candidate.rolling.bits64 >> 63) & 1U;
  candidate.rolling.bits64 =
      (candidate.rolling.bits64 << 1) | (bit & 1U);
  if (bit != expected) {
    // The mismatching bit completes a new rolling 64-bit candidate period.
    candidate.cycleProgress = 0;
    candidate.cycles = 1;
    return;
  }

  if (++candidate.cycleProgress == 64) {
    candidate.cycleProgress = 0;
    if (candidate.cycles != 255) candidate.cycles++;
    preserveCandidate64(candidate);
  }
}

void preserveCandidate128(RawCandidate &candidate) {
  if (candidate.cycles > candidate.provenCycles) {
    candidate.proven.bits128.low = candidate.rolling.bits128.low;
    candidate.proven.bits128.high = candidate.rolling.bits128.high;
    candidate.provenCycles = candidate.cycles;
  }
}

inline __attribute__((always_inline))
void feedRawBit128(RawCandidate &candidate, uint8_t bit) {
  const uint8_t promoted = candidate.rolling.bytes[7] >> 7;
  const uint8_t expected = candidate.rolling.bytes[15] >> 7;
  candidate.rolling.bits128.high =
      (candidate.rolling.bits128.high << 1) | promoted;
  candidate.rolling.bits128.low =
      (candidate.rolling.bits128.low << 1) | (bit & 1U);

  if (candidate.fill < 128) {
    if (++candidate.fill == 128) {
      candidate.cycleProgress = 0;
      candidate.cycles = 1;
      preserveCandidate128(candidate);
    }
    return;
  }

  if (bit != expected) {
    // The mismatching bit completes a new rolling 128-bit candidate period.
    candidate.cycleProgress = 0;
    candidate.cycles = 1;
    return;
  }

  if (++candidate.cycleProgress == 128) {
    candidate.cycleProgress = 0;
    if (candidate.cycles != 255) candidate.cycles++;
    preserveCandidate128(candidate);
  }
}

inline __attribute__((always_inline))
void feedRawBit(RawCandidate &candidate, uint8_t bit) {
  if (captureWidth == 32) {
    feedRawBit32(candidate, bit);
  } else if (captureWidth == 64) {
    feedRawBit64(candidate, bit);
  } else {
    feedRawBit128(candidate, bit);
  }
}

void processManchesterWindow(int16_t difference, uint8_t phase) {
  const uint16_t magnitude =
      difference < 0 ? (uint16_t)-difference : (uint16_t)difference;
  totalWindows++;
  if (magnitude > maxWindowDifference) maxWindowDifference = magnitude;
  RawCandidate &normal = candidates[phase << 1];
  RawCandidate &inverse = candidates[(phase << 1) | 1];
  if (magnitude < matchThreshold) {
    resetCandidateRun(normal);
    resetCandidateRun(inverse);
    return;
  }
  strongWindows++;
  const uint8_t bit = difference > 0 ? 1 : 0;
  feedRawBit(normal, bit);
  // The inverse candidate is fed the complemented bit, so its register is always
  // the exact complement of the normal one and its periodicity is identical.
  // At 128 bits that redundancy costs more time than the sample interval allows,
  // so it is dropped and the matcher compares the complement of the target too.
  if (captureWidth != 128) feedRawBit(inverse, bit ^ 1U);
}

void processSample(uint16_t sample) {
  sampleCount++;
  if (sample < captureMin) captureMin = sample;
  if (sample > captureMax) captureMax = sample;

  if (ringFill < bitSamples) {
    sampleRing[ringPos] = sample;
    ringPos = (ringPos + 1) & ringMask;
    if (++ringFill != bitSamples) return;
    firstHalfSum = 0;
    secondHalfSum = 0;
    for (uint8_t i = 0; i < halfSamples; i++) firstHalfSum += sampleRing[i];
    for (uint8_t i = halfSamples; i < bitSamples; i++) {
      secondHalfSum += sampleRing[i];
    }
  } else {
    const uint8_t middle = (ringPos + halfSamples) & ringMask;
    const uint16_t oldFirst = sampleRing[ringPos];
    const uint16_t oldSecond = sampleRing[middle];
    firstHalfSum = firstHalfSum - oldFirst + oldSecond;
    secondHalfSum = secondHalfSum - oldSecond + sample;
    sampleRing[ringPos] = sample;
    ringPos = (ringPos + 1) & ringMask;
  }

  processManchesterWindow(
      (int16_t)secondHalfSum - (int16_t)firstHalfSum,
      sampleCount & ringMask);
}

void beginCapture(uint64_t targetHigh, uint64_t target, uint8_t width,
                  bool restartCarrier, uint8_t samplesPerBit) {
  captureTarget = target;
  captureTargetHigh = targetHigh;
  captureWidth = width;
  bitSamples = samplesPerBit;
  halfSamples = samplesPerBit >> 1;
  ringMask = samplesPerBit - 1;
  activeCandidates = samplesPerBit << 1;
  matchThreshold = halfSamples * MIN_LEVEL_DIFF;
  ringPos = 0;
  ringFill = 0;
  firstHalfSum = 0;
  secondHalfSum = 0;
  sampleCount = 0;
  captureMin = 1023;
  captureMax = 0;
  missedSamples = 0;
  lateSamples = 0;
  maxWindowDifference = 0;
  strongWindows = 0;
  totalWindows = 0;
  for (uint8_t i = 0; i < 16; i++) clearCandidate(candidates[i]);

  savedHardware.timsk0 = TIMSK0;
  savedHardware.tccr2a = TCCR2A;
  savedHardware.tccr2b = TCCR2B;
  savedHardware.timsk2 = TIMSK2;
  savedHardware.ocr2a = OCR2A;
  savedHardware.tcnt2 = TCNT2;
  savedHardware.admux = ADMUX;
  savedHardware.adcsra = ADCSRA;
  savedHardware.adcsrb = ADCSRB;
  savedHardware.didr0 = DIDR0;

  uint8_t savedSreg = SREG;
  cli();
  TIMSK0 &= ~_BV(TOIE0);
  TIMSK2 = 0;
  ADMUX = _BV(REFS0);  // AVcc reference, ADC0/A0.
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);  // 500 kHz ADC.
  ADCSRB = 0;
  DIDR0 |= _BV(ADC0D);
  ADCSRA |= _BV(ADSC);
  SREG = savedSreg;
  while (ADCSRA & _BV(ADSC)) {}
  ADCSRA |= _BV(ADIF);

  savedSreg = SREG;
  cli();
  if (restartCarrier) {
    // Start the carrier and sample timer from the same prescaler boundary.
    // Matching the working reader keeps every ADC conversion at a known carrier
    // phase instead of locking onto a stable alias of residual carrier ripple.
    GTCCR = _BV(TSM) | _BV(PSRSYNC) | _BV(PSRASY);
    TCCR1A = _BV(COM1A0);
    TCCR1B = _BV(WGM12) | _BV(CS10);
    TCNT1 = 0;
    OCR1A = 63;
  } else {
    // A direct access answer is already in flight, so the carrier must keep
    // running. Waiting for the start of a carrier period still pins the sample
    // phase without touching Timer1.
    while (TCNT1L > 8) {}
  }
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS21);
  TCNT2 = 0;
  OCR2A = 127;  // 16 MHz / 8 / 128 = one trigger every 64 us.
  TIFR2 = _BV(OCF2A);
  TIMSK2 = _BV(OCIE2A);
  if (restartCarrier) GTCCR = 0;
  SREG = savedSreg;
}

uint16_t readSynchronousSample() {
  while (!(ADCSRA & _BV(ADIF))) {}
  const uint16_t sample = ADC;
  ADCSRA |= _BV(ADIF);
  return sample;
}

CaptureResult endCapture() {
  const uint8_t savedSreg = SREG;
  cli();
  TIMSK2 = 0;
  TCCR2B = 0;
  ADCSRA &= ~_BV(ADEN);
  SREG = savedSreg;

  CaptureResult result;
  result.adcMin = captureMin;
  result.adcMax = captureMax;
  result.missed = missedSamples;
  result.late = lateSamples;
  result.maxDifference = maxWindowDifference;
  result.strongPercent =
      totalWindows ? (uint8_t)((strongWindows * 100UL) / totalWindows) : 0;
  // Adjacent phase/polarity candidates describe the same RF stream. Taking
  // the strongest continuous run, rather than adding votes, de-duplicates it.
  result.matches = 0;
  if (captureWidth == 32) {
    uint32_t rotatedTarget = (uint32_t)captureTarget;
    for (uint8_t rotation = 0; rotation < 32; rotation++) {
      for (uint8_t i = 0; i < 16; i++) {
        if (candidates[i].provenCycles > result.matches &&
            candidates[i].proven.bits32 == rotatedTarget) {
          result.matches = candidates[i].provenCycles;
        }
      }
      rotatedTarget = (rotatedTarget << 1) | (rotatedTarget >> 31);
    }
  } else if (captureWidth == 64) {
    uint64_t rotatedTarget = captureTarget;
    for (uint8_t rotation = 0; rotation < 64; rotation++) {
      for (uint8_t i = 0; i < 16; i++) {
        if (candidates[i].provenCycles > result.matches &&
            candidates[i].proven.bits64 == rotatedTarget) {
          result.matches = candidates[i].provenCycles;
        }
      }
      rotatedTarget = (rotatedTarget << 1) | (rotatedTarget >> 63);
    }
  } else {
    uint64_t rotatedHigh = captureTargetHigh;
    uint64_t rotatedLow = captureTarget;
    for (uint8_t rotation = 0; rotation < 128; rotation++) {
      for (uint8_t i = 0; i < 16; i++) {
        if (candidates[i].provenCycles <= result.matches) continue;
        const bool direct = candidates[i].proven.bits128.high == rotatedHigh &&
                            candidates[i].proven.bits128.low == rotatedLow;
        const bool inverted = candidates[i].proven.bits128.high == ~rotatedHigh &&
                              candidates[i].proven.bits128.low == ~rotatedLow;
        if (direct || inverted) result.matches = candidates[i].provenCycles;
      }
      const uint64_t carry = (rotatedLow >> 63) & 1U;
      const uint64_t top = (rotatedHigh >> 63) & 1U;
      rotatedHigh = (rotatedHigh << 1) | carry;
      rotatedLow = (rotatedLow << 1) | top;
    }
  }

  cli();
  TCCR2A = savedHardware.tccr2a;
  TCCR2B = savedHardware.tccr2b;
  OCR2A = savedHardware.ocr2a;
  TCNT2 = savedHardware.tcnt2;
  TIFR2 = _BV(OCF2A);
  TIMSK2 = savedHardware.timsk2;
  ADMUX = savedHardware.admux;
  ADCSRB = savedHardware.adcsrb;
  DIDR0 = savedHardware.didr0;
  ADCSRA = savedHardware.adcsra;
  // Re-enable the previous Timer0 mask. Time elapsed during capture is not
  // reconstructed, so millis()/micros() should not be used for measurements.
  TIMSK0 = savedHardware.timsk0;
  SREG = savedSreg;
  return result;
}

void printCaptureCandidates(uint8_t width) {
  if (width == 128) {
    Serial.println(F("  phase candidates (normal polarity only at 128 bits):"));
  } else {
    Serial.println(F("  phase/polarity periodic candidates:"));
  }
  for (uint8_t i = 0; i < activeCandidates; i++) {
    Serial.print(F("    phase="));
    Serial.print(i >> 1);
    Serial.print(F(" polarity="));
    Serial.print((i & 1U) ? F("inverse") : F("normal"));
    Serial.print(F(" bestCycles="));
    Serial.print(candidates[i].provenCycles);
    Serial.print(F(" value="));
    if (width == 128) {
      for (int8_t nibble = 15; nibble >= 0; nibble--) {
        Serial.print(
            (uint8_t)((candidates[i].proven.bits128.high >> (nibble * 4)) & 0x0F),
            HEX);
      }
    }
    const int8_t firstNibble = width == 32 ? 7 : 15;
    const uint64_t value =
        width == 32 ? candidates[i].proven.bits32 : candidates[i].proven.bits128.low;
    for (int8_t nibble = firstNibble; nibble >= 0; nibble--) {
      Serial.print((uint8_t)((value >> (nibble * 4)) & 0x0F), HEX);
    }
    Serial.println();
  }
}

CaptureResult runCapture(uint64_t targetHigh, uint64_t target, uint8_t width,
                         uint32_t samples, bool restartCarrier,
                         uint8_t samplesPerBit) {
  beginCapture(targetHigh, target, width, restartCarrier, samplesPerBit);
  while (sampleCount < samples) {
    processSample(readSynchronousSample());
  }
  return endCapture();
}

void printCaptureHealth(const __FlashStringHelper *label,
                        const CaptureResult &result, bool showMatches) {
  Serial.print(label);
  Serial.print(F(": ADC="));
  Serial.print(result.adcMin);
  Serial.print('-');
  Serial.print(result.adcMax);
  Serial.print(F(" missed="));
  Serial.print(result.missed);
  Serial.print(F(" late="));
  Serial.print(result.late);
  Serial.print(F(" strong="));
  Serial.print(result.strongPercent);
  Serial.print(F("% maxDiff="));
  Serial.print(result.maxDifference);
  if (showMatches) {
    Serial.print(F(" matches="));
    Serial.print(result.matches);
  }
  Serial.println();
}

CaptureResult captureCyclesWide(const __FlashStringHelper *label,
                                uint64_t targetHigh, uint64_t target,
                                uint8_t width, uint8_t samplesPerBit) {
  const CaptureResult result =
      runCapture(targetHigh, target, width, CAPTURE_SAMPLES, true,
                 samplesPerBit);
  printCaptureHealth(label, result, true);
  printCaptureCandidates(width);
  return result;
}

CaptureResult captureCyclesAt(const __FlashStringHelper *label, uint64_t target,
                              uint8_t width, uint8_t samplesPerBit) {
  return captureCyclesWide(label, 0, target, width, samplesPerBit);
}

CaptureResult captureCycles(const __FlashStringHelper *label,
                            uint64_t target, uint8_t width) {
  return captureCyclesAt(label, target, width, 8);
}

bool captureHealthy(const CaptureResult &result) {
  return result.missed == 0 && result.late == 0;
}

bool capturePassed(const CaptureResult &result) {
  return captureHealthy(result) && result.matches >= REQUIRED_CYCLES;
}

void printHex32(uint32_t value) {
  for (int8_t nibble = 7; nibble >= 0; nibble--) {
    Serial.print((uint8_t)((value >> (nibble * 4)) & 0x0F), HEX);
  }
}

void printHex64(uint64_t value) {
  for (int8_t nibble = 15; nibble >= 0; nibble--) {
    Serial.print((uint8_t)((value >> (nibble * 4)) & 0x0F), HEX);
  }
}

int8_t hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// A direct access answer is a bare repeating 32-bit block with no preamble, so
// its bit alignment is unknown. Reducing every answer to its numerically
// smallest rotation makes comparisons alignment independent, and a real
// configuration block is close to that rotation anyway because its eleven
// leading bits are zero in basic mode.
uint32_t minRotation32(uint32_t value) {
  uint32_t best = value;
  uint32_t rotated = value;
  for (uint8_t i = 0; i < 31; i++) {
    rotated = (rotated << 1) | (rotated >> 31);
    if (rotated < best) best = rotated;
  }
  return best;
}

bool sameRotation32(uint32_t left, uint32_t right) {
  return minRotation32(left) == minRotation32(right);
}

void printRotations32(uint32_t value) {
  uint32_t rotated = value;
  for (uint8_t i = 0; i < 32; i++) {
    if ((i & 3U) == 0) Serial.print(F("\n    "));
    printHex32(rotated);
    Serial.print(' ');
    rotated = (rotated << 1) | (rotated >> 31);
  }
  Serial.println();
}

uint8_t strongestCandidate() {
  uint8_t best = 0;
  for (uint8_t i = 1; i < 16; i++) {
    if (candidates[i].provenCycles > candidates[best].provenCycles) best = i;
  }
  return best;
}

void describeBlockValue(uint32_t value) {
  if (prepared && sameRotation32(value, preparedBlock1)) {
    Serial.println(F("    identifies as prepared B1"));
  } else if (prepared && sameRotation32(value, preparedBlock2)) {
    Serial.println(F("    identifies as prepared B2"));
  } else if (sameRotation32(value, CONFIG_MAXBLK2)) {
    Serial.println(F("    identifies as 0x00148040 EM4100 Manchester RF/64 MAXBLK=2"));
  } else if (sameRotation32(value, CONFIG_MAXBLK1)) {
    Serial.println(F("    identifies as 0x00148020 EM4100 Manchester RF/64 MAXBLK=1"));
  } else if (sameRotation32(value, 0x00148080UL)) {
    Serial.println(F("    identifies as 0x00148080 EM4100 Manchester RF/64 MAXBLK=4"));
  } else if (sameRotation32(value, 0x000880E0UL)) {
    Serial.println(F("    identifies as 0x000880E0 T5577 factory default"));
  } else if (value == 0) {
    Serial.println(F("    all zero, or no answer was demodulated"));
  }
}

void readBlock(uint8_t page, uint8_t block, bool verbose) {
  powerCycleField();
  noInterrupts();
  sendDirectAccessCommand(page, block);
  interrupts();
  const CaptureResult result =
      runCapture(0, 0, 32, READ_CAPTURE_SAMPLES, false, 8);

  const uint8_t best = strongestCandidate();
  const uint32_t value = candidates[best].proven.bits32;
  const uint8_t cycles = candidates[best].provenCycles;

  Serial.print(F("READ p"));
  Serial.print(page);
  Serial.print(F(" b"));
  Serial.print(block);
  Serial.print(F(": cycles="));
  Serial.print(cycles);
  Serial.print(F(" minRotation=0x"));
  printHex32(minRotation32(value));
  Serial.print(F(" raw=0x"));
  printHex32(value);
  Serial.println();
  printCaptureHealth(F("  capture"), result, false);
  if (cycles >= REQUIRED_CYCLES && captureHealthy(result)) {
    describeBlockValue(value);
    if (verbose) {
      Serial.println(F("  all 32 rotations of the answer:"));
      printRotations32(value);
    }
  } else {
    Serial.println(F("    no stable 32-bit answer; see notes in README"));
  }

  // Printed even without periodicity, because a one-shot answer would appear
  // here and nowhere else.
  Serial.println(F("  first 32 bits after the command, per phase/polarity:"));
  for (uint8_t i = 0; i < activeCandidates; i++) {
    if ((i & 3U) == 0) Serial.print(F("    "));
    printHex32(candidates[i].firstBits);
    Serial.print((i & 3U) == 3U ? '\n' : ' ');
  }
  if (verbose) printCaptureCandidates(32);
}

void readCommand(const char *arguments) {
  int8_t page = hexValue(arguments[0]);
  if (page < 0 || page > 1 || arguments[1] != ' ') {
    Serial.println(F("Usage: READ <page 0|1> <block 0-7>"));
    return;
  }
  int8_t block = hexValue(arguments[2]);
  if (block < 0 || block > 7 || arguments[3] != '\0') {
    Serial.println(F("Usage: READ <page 0|1> <block 0-7>"));
    return;
  }
  readBlock((uint8_t)page, (uint8_t)block, true);
}

void dumpCommand() {
  Serial.println(F("Direct access dump. No password is sent; nothing is written."));
  for (uint8_t block = 0; block < 8; block++) readBlock(0, block, false);
  for (uint8_t block = 0; block < 4; block++) readBlock(1, block, false);
  Serial.println(F("Dump complete. Page 1 blocks 1 and 2 hold traceability data."));
}

// A captured window has no preamble to align to and the envelope detector may
// resolve the two Manchester levels either way round, so a frame inside it can
// sit at any rotation in either polarity. Re-encoding the ten digits read out of
// a candidate and demanding that all 64 bits match verifies the header, both
// parity axes and the stop bit in one comparison, which makes a false positive a
// 64-bit coincidence rather than a plausible accident.
bool findEm4100(uint64_t high, uint64_t low, uint8_t width, uint8_t id[5],
                uint8_t *rotationOut, bool *invertedOut, uint64_t *restOut) {
  for (uint8_t polarity = 0; polarity < 2; polarity++) {
    uint64_t h = polarity ? ~high : high;
    uint64_t l = polarity ? ~low : low;
    for (uint8_t rotation = 0; rotation < width; rotation++) {
      const uint64_t frame = (width == 128) ? h : l;
      uint8_t candidate[5];
      for (uint8_t row = 0; row < 10; row++) {
        uint8_t nibble = 0;
        for (uint8_t col = 0; col < 4; col++) {
          nibble = (uint8_t)((nibble << 1) |
                             (uint8_t)((frame >> (54 - 5 * row - col)) & 1U));
        }
        if (row & 1U) {
          candidate[row >> 1] |= nibble;
        } else {
          candidate[row >> 1] = (uint8_t)(nibble << 4);
        }
      }
      if (encodeEm4100(candidate) == frame) {
        for (uint8_t i = 0; i < 5; i++) id[i] = candidate[i];
        *rotationOut = rotation;
        *invertedOut = polarity != 0;
        *restOut = (width == 128) ? l : 0;
        return true;
      }
      if (width == 128) {
        const uint64_t carry = l >> 63;
        const uint64_t top = h >> 63;
        h = (h << 1) | carry;
        l = (l << 1) | top;
      } else {
        l = (l << 1) | (l >> 63);
      }
    }
  }
  return false;
}

// Every other command here assumes the tag streams a 64-bit EM4100 frame at
// RF/64, which is exactly the assumption to drop when a tag behaves oddly. This
// one assumes nothing: it never gaps the field and never writes, and it scores
// three period lengths against both plausible bit rates. Whichever combination
// reaches a cycle count near its ceiling is the tag's real format.
void observeAt(uint8_t width, uint8_t samplesPerBit) {
  uint64_t targetHigh = 0;
  uint64_t target = 0;
  if (width == 32) {
    target = preparedBlock1;
  } else if (width == 64) {
    target = preparedFrame;
  } else {
    targetHigh = preparedFrame;
    target = preparedFrame2;
  }

  const CaptureResult result =
      runCapture(targetHigh, target, width, CAPTURE_SAMPLES, true, samplesPerBit);
  const RawCandidate &best = candidates[strongestCandidate()];
  // The cycle counter saturates at 255, so the ceiling has to as well or a
  // perfect short-period capture would look like a partial one.
  uint16_t ceiling = (uint16_t)((CAPTURE_SAMPLES / samplesPerBit) / width);
  if (ceiling > 255) ceiling = 255;

  Serial.print(F("  RF/"));
  Serial.print(samplesPerBit == 8 ? 64 : 32);
  Serial.print(F(" width="));
  if (width < 100) Serial.print(' ');
  Serial.print(width);
  Serial.print(F(" cycles="));
  Serial.print(best.provenCycles);
  Serial.print('/');
  Serial.print(ceiling);
  Serial.print(F(" value=0x"));
  if (width == 32) {
    printHex32(best.proven.bits32);
  } else {
    if (width == 128) printHex64(best.proven.bits128.high);
    printHex64(best.proven.bits128.low);
  }
  Serial.print(F(" strong="));
  Serial.print(result.strongPercent);
  Serial.print(F("% maxDiff="));
  Serial.print(result.maxDifference);
  if (!captureHealthy(result)) Serial.print(F(" UNHEALTHY"));
  Serial.println();

  if (width == 128 && best.provenCycles >= REQUIRED_CYCLES &&
      best.proven.bits128.high == best.proven.bits128.low) {
    Serial.println(F("    both halves are equal, so the true period is 64 bits"));
  }

  if (width == 32 || best.provenCycles < REQUIRED_CYCLES) return;
  uint8_t id[5];
  uint8_t rotation;
  bool inverted;
  uint64_t rest;
  if (!findEm4100(best.proven.bits128.high, best.proven.bits128.low, width, id,
                  &rotation, &inverted, &rest)) {
    Serial.println(F("    no parity-valid EM4100 frame at any rotation"));
    return;
  }
  Serial.print(F("    valid EM4100 frame ID="));
  for (uint8_t i = 0; i < 5; i++) {
    if (id[i] < 16) Serial.print('0');
    Serial.print(id[i], HEX);
  }
  Serial.print(F(" starts at bit "));
  Serial.print(rotation);
  if (inverted) Serial.print(F(", envelope inverted"));
  Serial.println();
  if (width == 128) {
    Serial.print(F("    the other 64 bits are 0x"));
    printHex64(rest);
    Serial.println();
  }
}

void observeCommand() {
  Serial.println(F("Passive observation. No downlink command, no writes."));
  Serial.println(F("The tag's real rate and period are the row whose cycle"));
  Serial.println(F("count is closest to its ceiling. Takes about twenty seconds."));
  powerCycleField();
  observeAt(32, 8);
  observeAt(64, 8);
  observeAt(128, 8);
  observeAt(32, 4);
  observeAt(64, 4);
  observeAt(128, 4);
  Serial.println(F("A period of 128 at one rate, with 64 and 32 well short of"));
  Serial.println(F("their ceilings, is a genuine 128-bit tag."));
}

void printPrepared() {
  Serial.print(F("Prepared ID="));
  for (uint8_t i = 0; i < 5; i++) {
    if (preparedId[i] < 16) Serial.print('0');
    Serial.print(preparedId[i], HEX);
  }
  Serial.print(F(" B1=0x"));
  printHex32(preparedBlock1);
  Serial.print(F(" B2=0x"));
  printHex32(preparedBlock2);
  Serial.println();
}

bool parseId(const char *text, uint8_t id[5]) {
  if (strlen(text) != 10) return false;
  for (uint8_t i = 0; i < 5; i++) {
    const int8_t high = hexValue(text[i * 2]);
    const int8_t low = hexValue(text[i * 2 + 1]);
    if (high < 0 || low < 0) return false;
    id[i] = (high << 4) | low;
  }
  return true;
}

void prepareId(const char *text) {
  uint8_t id[5];
  if (!parseId(text, id)) {
    Serial.println(F("Invalid ID: use exactly 10 hexadecimal digits."));
    return;
  }
  memcpy(preparedId, id, sizeof(preparedId));
  preparedFrame = encodeEm4100(preparedId);
  preparedBlock1 = (uint32_t)(preparedFrame >> 32);
  preparedBlock2 = (uint32_t)preparedFrame;
  prepared = true;
  printPrepared();
}

void prepareId2(const char *text) {
  uint8_t id[5];
  if (!parseId(text, id)) {
    Serial.println(F("Invalid ID: use exactly 10 hexadecimal digits."));
    return;
  }
  if (prepared && memcmp(id, preparedId, sizeof(id)) == 0) {
    Serial.println(F("ID2 must differ from ID, or 128 bits cannot be told from 64."));
    return;
  }
  memcpy(preparedId2, id, sizeof(preparedId2));
  preparedFrame2 = encodeEm4100(preparedId2);
  preparedBlock3 = (uint32_t)(preparedFrame2 >> 32);
  preparedBlock4 = (uint32_t)preparedFrame2;
  prepared2 = true;
  Serial.print(F("Prepared ID2="));
  for (uint8_t i = 0; i < 5; i++) {
    if (preparedId2[i] < 16) Serial.print('0');
    Serial.print(preparedId2[i], HEX);
  }
  Serial.print(F(" B3=0x"));
  printHex32(preparedBlock3);
  Serial.print(F(" B4=0x"));
  printHex32(preparedBlock4);
  Serial.println();
}

void destructiveWarning(const __FlashStringHelper *action) {
  Serial.println(F("WARNING: DESTRUCTIVE TAG WRITE."));
  Serial.println(F("Use one centered sacrificial tag you are authorized to test."));
  Serial.println(F("Do not move the tag, reset, or remove power."));
  Serial.print(action);
  Serial.println(F(" begins in 2 seconds."));
  Serial.flush();
  delay(2000);
}

bool restoreConfigOnly() {
  Serial.println(F("Restoring B0=0x00148040 (MAXBLK=2)..."));
  writeFreshBlock(0, CONFIG_MAXBLK2);
  powerCycleField();
  const CaptureResult restored =
      captureCycles(F("RESTORE64"), preparedFrame, 64);
  if (capturePassed(restored)) {
    recoveryRequired = false;
    Serial.println(F("RESTORED: prepared 64-bit cycle verified 3 times."));
    return true;
  }
  Serial.println(F("RESTORE FAILED: recovery lock retained; use RECOVER."));
  return false;
}

void testMax1() {
  if (!prepared) {
    Serial.println(F("Prepare the current tag ID first: ID <10_HEX_DIGITS>."));
    return;
  }

  destructiveWarning(F("TESTMAX1"));
  const CaptureResult baseline =
      captureCycles(F("BASELINE64"), preparedFrame, 64);
  if (!capturePassed(baseline)) {
    Serial.println(F("ABORTED WITHOUT WRITING: baseline or sampling unhealthy."));
    return;
  }

  recoveryRequired = true;  // Set before the first mutating downlink.
  Serial.println(F("Writing only B0=0x00148020 (MAXBLK=1)..."));
  writeFreshBlock(0, CONFIG_MAXBLK1);
  powerCycleField();

  const CaptureResult max1 =
      captureCycles(F("TEST32"), (uint64_t)preparedBlock1, 32);
  enum Classification : uint8_t { ACCEPTED, UNCHANGED, INCONCLUSIVE };
  Classification classification = INCONCLUSIVE;
  if (capturePassed(max1)) {
    classification = ACCEPTED;
  } else {
    const CaptureResult original =
        captureCycles(F("CHECK64"), preparedFrame, 64);
    if (capturePassed(original)) classification = UNCHANGED;
  }

  if (classification == ACCEPTED) {
    Serial.println(F("RESULT: ACCEPTED - B1 repeated as a 32-bit cycle."));
  } else if (classification == UNCHANGED) {
    Serial.println(F("RESULT: UNCHANGED - original 64-bit cycle remained."));
  } else {
    Serial.println(F("RESULT: INCONCLUSIVE - neither healthy cycle proved."));
  }

  // Restoration is unconditional after any mutation, regardless of result.
  restoreConfigOnly();
}

// Asks a different question from TESTMAX1: not whether MAXBLK is implemented,
// but whether a block 0 write changes the tag's behaviour at all.
void testRf32() {
  if (!prepared) {
    Serial.println(F("Prepare the current tag ID first: ID <10_HEX_DIGITS>."));
    return;
  }

  destructiveWarning(F("TESTRF32"));
  const CaptureResult baseline =
      captureCycles(F("BASELINE64"), preparedFrame, 64);
  if (!capturePassed(baseline)) {
    Serial.println(F("ABORTED WITHOUT WRITING: baseline or sampling unhealthy."));
    return;
  }

  recoveryRequired = true;  // Set before the first mutating downlink.
  Serial.println(F("Writing only B0=0x00088040 (RF/32, Manchester, MAXBLK=2)..."));
  writeFreshBlock(0, CONFIG_RF32);
  powerCycleField();

  const CaptureResult fast =
      captureCyclesAt(F("TESTRF32"), preparedFrame, 64, 4);
  if (capturePassed(fast)) {
    Serial.println(F("RESULT: ACCEPTED - same frame arrived at RF/32."));
    Serial.println(F("Block 0 writes take effect on this tag."));
  } else {
    const CaptureResult slow =
        captureCycles(F("CHECK64"), preparedFrame, 64);
    if (capturePassed(slow)) {
      Serial.println(F("RESULT: UNCHANGED - frame still arrives at RF/64."));
      Serial.println(F("The data bit rate field was not applied."));
    } else {
      Serial.println(F("RESULT: INCONCLUSIVE - neither rate proved."));
    }
  }

  // Restoration is unconditional after any mutation, regardless of result.
  restoreConfigOnly();
}

void restoreFull();

// Goes after 128-bit operation: blocks 3 and 4 get a second distinct EM4100
// frame before MAXBLK is raised, so a 128-bit stream cannot be mistaken for the
// 64-bit one. The intermediate check also reveals whether writing addresses 3
// and 4 disturbs blocks 1 and 2, which is what an aliased address map would do.
void test128() {
  if (!prepared || !prepared2) {
    Serial.println(F("Prepare both frames first: ID <10_HEX> and ID2 <10_HEX>."));
    return;
  }

  destructiveWarning(F("TEST128"));
  const CaptureResult baseline =
      captureCycles(F("BASELINE64"), preparedFrame, 64);
  if (!capturePassed(baseline)) {
    Serial.println(F("ABORTED WITHOUT WRITING: baseline or sampling unhealthy."));
    return;
  }

  recoveryRequired = true;  // Set before the first mutating downlink.
  Serial.println(F("Writing B3 and B4 with the second frame..."));
  writeFreshBlock(3, preparedBlock3);
  writeFreshBlock(4, preparedBlock4);
  powerCycleField();

  const CaptureResult intact =
      captureCycles(F("ALIAS64"), preparedFrame, 64);
  if (capturePassed(intact)) {
    Serial.println(F("Addresses 3 and 4 did not disturb blocks 1 and 2."));
  } else {
    const CaptureResult moved =
        captureCycles(F("ALIASCHK"), preparedFrame2, 64);
    if (capturePassed(moved)) {
      Serial.println(F("ALIASED: the second frame now streams as 64 bits."));
      Serial.println(F("Addresses 3 and 4 map onto blocks 1 and 2 on this tag."));
    } else {
      Serial.println(F("First frame no longer proved after writing B3 and B4."));
    }
  }

  Serial.println(F("Writing only B0=0x00148080 (MAXBLK=4)..."));
  writeFreshBlock(0, CONFIG_MAXBLK4);
  powerCycleField();

  const CaptureResult wide = captureCyclesWide(
      F("TEST128"), preparedFrame, preparedFrame2, 128, 8);
  if (!captureHealthy(wide)) {
    // Without this the 128-bit search could be silently skipped and the 64-bit
    // fallback reported as though the wide search had actually run.
    Serial.println(F("WARNING: the 128-bit capture dropped samples."));
    Serial.println(F("Its zero cycle counts prove nothing about the tag."));
  }
  // A 128-bit window whose halves are equal means the transmission period is
  // really 64 bits, so the second frame never reached the air. Saying so turns
  // the candidate dump into a conclusion instead of something to decode by hand.
  const RawCandidate &widest = candidates[strongestCandidate()];
  if (widest.provenCycles >= REQUIRED_CYCLES &&
      widest.proven.bits128.high == widest.proven.bits128.low) {
    Serial.println(F("  the 128-bit window is one 64-bit pattern repeated twice"));
  }

  if (capturePassed(wide)) {
    Serial.println(F("RESULT: ACCEPTED - both frames streamed as a 128-bit cycle."));
  } else {
    const CaptureResult narrow =
        captureCycles(F("CHECK64"), preparedFrame, 64);
    if (capturePassed(narrow)) {
      Serial.println(F("RESULT: UNCHANGED - only the first frame streams as 64 bits."));
      Serial.println(F("MAXBLK=4 was not applied even with distinct B3 and B4."));
      // This holds regardless of the wide capture: an unbroken 64-bit period of
      // the first frame is incompatible with a 128-bit stream containing the
      // second one, whatever blocks 3 and 4 actually hold.
      Serial.println(F("The unbroken 64-bit period rules out a 128-bit stream."));
    } else {
      Serial.println(F("RESULT: INCONCLUSIVE - neither 128 nor 64 bits proved."));
    }
  }

  restoreFull();
}

bool recoverOnce() {
  writeFreshBlock(1, preparedBlock1);
  writeFreshBlock(2, preparedBlock2);
  writeFreshBlock(0, CONFIG_MAXBLK2);
  powerCycleField();
  const CaptureResult result =
      captureCycles(F("RECOVER64"), preparedFrame, 64);
  return capturePassed(result);
}

// Used after a test that may have written blocks other than 0, so restoring the
// configuration alone would not be enough.
void restoreFull() {
  Serial.println(F("Restoring B1, B2, then B0=0x00148040 (MAXBLK=2)..."));
  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    if (recoverOnce()) {
      recoveryRequired = false;
      Serial.println(F("RESTORED: prepared 64-bit cycle verified 3 times."));
      return;
    }
    Serial.print(F("Restore attempt "));
    Serial.print(attempt);
    Serial.println(F("/3 did not verify."));
  }
  Serial.println(F("RESTORE FAILED: recovery lock retained; use RECOVER."));
}

void recoverCommand() {
  if (!prepared) {
    Serial.println(F("No prepared ID is available for recovery."));
    return;
  }
  destructiveWarning(F("RECOVER"));
  recoveryRequired = true;
  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    Serial.print(F("Recovery attempt "));
    Serial.print(attempt);
    Serial.println(F("/3: rewriting B1, B2, then B0 MAXBLK=2."));
    if (recoverOnce()) {
      recoveryRequired = false;
      Serial.println(F("RECOVERED: prepared 64-bit cycle verified 3 times."));
      return;
    }
  }
  Serial.println(F("RECOVERY FAILED: prepared data and RAM lock retained."));
  Serial.println(F("Keep power on and the tag still; correct coupling, then RECOVER."));
}

void showHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  ID <10_HEX_DIGITS>  prepare expected frame/B1/B2"));
  Serial.println(F("  ID2 <10_HEX_DIGITS> prepare second frame/B3/B4 for 128 bits"));
  Serial.println(F("  OBSERVE             passive: find the tag's real rate and period"));
  Serial.println(F("  READ <page> <block> direct access read, writes nothing"));
  Serial.println(F("  DUMP                read page 0 blocks 0-7 and page 1 blocks 0-3"));
  Serial.println(F("  TESTMAX1            test B0 MAXBLK=1, always restore"));
  Serial.println(F("  TESTRF32            test B0 bit rate RF/32, always restore"));
  Serial.println(F("  TEST128             write B3/B4 and MAXBLK=4, always restore"));
  Serial.println(F("  RECOVER             rewrite B1/B2/B0, up to 3 tries"));
  Serial.println(F("  HELP                show this help"));
  Serial.println(F("Writes keep password and lock bits clear. Authorized sacrificial tag only."));
}

bool equalsIgnoreCase(const char *left, const char *right) {
  while (*left && *right) {
    char a = *left++;
    char b = *right++;
    if (a >= 'a' && a <= 'z') a -= 'a' - 'A';
    if (b >= 'a' && b <= 'z') b -= 'a' - 'A';
    if (a != b) return false;
  }
  return *left == '\0' && *right == '\0';
}

void handleCommand(char *command) {
  while (*command == ' ') command++;
  char *end = command + strlen(command);
  while (end > command && end[-1] == ' ') *--end = '\0';
  if (*command == '\0') return;

  const bool readOnlyCommand =
      equalsIgnoreCase(command, "RECOVER") ||
      equalsIgnoreCase(command, "HELP") ||
      equalsIgnoreCase(command, "DUMP") ||
      equalsIgnoreCase(command, "OBSERVE") ||
      strncasecmp(command, "READ ", 5) == 0;
  if (recoveryRequired && !readOnlyCommand) {
    Serial.println(
        F("Blocked by recovery lock: only RECOVER, OBSERVE, READ, DUMP or HELP."));
    return;
  }

  if (strncasecmp(command, "ID2 ", 4) == 0) {
    prepareId2(command + 4);
  } else if (strncasecmp(command, "ID ", 3) == 0) {
    prepareId(command + 3);
  } else if (strncasecmp(command, "READ ", 5) == 0) {
    readCommand(command + 5);
  } else if (equalsIgnoreCase(command, "READ")) {
    Serial.println(F("Usage: READ <page 0|1> <block 0-7>"));
  } else if (equalsIgnoreCase(command, "DUMP")) {
    dumpCommand();
  } else if (equalsIgnoreCase(command, "OBSERVE")) {
    observeCommand();
  } else if (equalsIgnoreCase(command, "TESTMAX1")) {
    testMax1();
  } else if (equalsIgnoreCase(command, "TESTRF32")) {
    testRf32();
  } else if (equalsIgnoreCase(command, "TEST128")) {
    test128();
  } else if (equalsIgnoreCase(command, "RECOVER")) {
    recoverCommand();
  } else if (equalsIgnoreCase(command, "HELP")) {
    showHelp();
  } else {
    Serial.println(F("Unknown command. Type HELP."));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CARRIER, OUTPUT);
  digitalWrite(PIN_CARRIER, LOW);
  TCCR1A = _BV(COM1A0);
  TCCR1B = _BV(WGM12) | _BV(CS10);
  TCNT1 = 0;
  OCR1A = 63;

  Serial.println(F("T5577 block probe and MAXBLK=1 capability test ready."));
  Serial.println(F("D9 carrier, A0 envelope, Serial 115200/Newline."));
  Serial.println(F("RAM recovery state is LOST on reset or power loss."));
  showHelp();
}

void loop() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (discardSerialLine) {
        discardSerialLine = false;
        serialLength = 0;
        continue;
      }
      serialBuffer[serialLength] = '\0';
      handleCommand(serialBuffer);
      serialLength = 0;
    } else if (discardSerialLine) {
      continue;
    } else if (serialLength < sizeof(serialBuffer) - 1) {
      serialBuffer[serialLength++] = c;
    } else {
      serialLength = 0;
      discardSerialLine = true;
      Serial.println(F("Command too long."));
    }
  }
}
