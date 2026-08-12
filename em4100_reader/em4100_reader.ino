/*
 * Software-only EM4100 reader for Arduino Uno / ATmega328P.
 *
 * Timer1 generates the 125 kHz carrier on D9. Timer2 starts ADC conversions
 * every 64 us, exactly eight carrier periods apart, so residual carrier ripple
 * is sampled at a fixed phase instead of aliasing into false data edges.
 *
 * The decoder tries Manchester and biphase at RF/64 and RF/32, validates the
 * complete EM4100 parity structure, then locks onto the detected format.
 * In Manchester RF/64 mode it also captures the next 64 bits. An exact repeat
 * of the validated EM4100 frame is reported as 64-bit operation; a stable,
 * different suffix is reported as a 128-bit candidate.
 *
 * Circuit:
 *   D9 -- 220R -- tank node
 *   tank node -- coil || 100nF -- GND
 *   tank node -- 1N4148 anode; cathode -- envelope node
 *   envelope node -- A0
 *   envelope node -- 100nF || 330R -- GND
 */

#include <Arduino.h>
#include <string.h>

const uint8_t PIN_CARRIER = 9;
const uint8_t SEARCHING = 0xFF;
const uint8_t MIN_AVERAGE_LEVEL_DIFF = 2;
const uint32_t REPORT_SAMPLES = 15625UL;

enum DecodeMode : uint8_t {
  MANCHESTER_RF64,
  MANCHESTER_RF32,
  BIPHASE_RF64,
  BIPHASE_RF32,
  MODE_COUNT
};

// Manchester RF/64 is the common EM4100 format. Starting locked avoids
// overrunning the Uno while testing every format in parallel. Change this to
// MODE_COUNT only when format auto-detection is specifically needed.
const DecodeMode START_MODE = MANCHESTER_RF64;

struct Decoder {
  uint16_t header;
  uint8_t pos;
  uint8_t rowXor;
  uint8_t colXor;
  uint8_t nibble;
  uint8_t id[5];
  uint64_t trailingData;
  uint8_t trailingBits;
};

struct BiphaseDecoder {
  Decoder frame;
  uint16_t previousSecondHalf;
  bool havePrevious;
};

Decoder manchester64[16];
Decoder manchester32[8];
BiphaseDecoder biphase64[8];
BiphaseDecoder biphase32[4];

uint16_t sampleRing[8];
uint16_t firstHalfSum = 0;
uint16_t secondHalfSum = 0;
uint8_t ringPos = 0;
uint8_t ringFill = 0;

uint16_t sampleRing32[4];
uint16_t firstHalfSum32 = 0;
uint16_t secondHalfSum32 = 0;
uint8_t ringPos32 = 0;
uint8_t ringFill32 = 0;

uint32_t totalSamples = 0;
uint32_t blockSamples = 0;
uint32_t blockSum = 0;
uint32_t strongWindows64 = 0;
uint32_t strongWindows32 = 0;
uint16_t blockMin = 1023;
uint16_t blockMax = 0;
uint16_t blockMaxDiff64 = 0;
uint16_t blockMaxDiff32 = 0;
uint16_t framesThisBlock[MODE_COUNT] = {0, 0, 0, 0};

uint8_t voteId[5];
uint8_t voteCount = 0;
uint8_t lastAcceptedId[5];
uint32_t lastAcceptedSample = 0;
bool haveLastAccepted = false;

uint8_t confirmedId[5];
bool confirmationPending = false;
DecodeMode confirmedMode = MANCHESTER_RF64;
DecodeMode lockedMode = START_MODE;
uint8_t printedId[5];
bool havePrintedId = false;

uint64_t cycleTailCandidate = 0;
uint8_t cycleIdCandidate[5];
uint8_t cycleVotes = 0;
uint32_t lastCycleSample = 0;
bool haveLastCycleSample = false;

uint64_t confirmedCycleTail = 0;
uint8_t confirmedCycleId[5];
bool cycleClassificationPending = false;

uint64_t printedCycleTail = 0;
uint8_t printedCycleId[5];
bool havePrintedCycleClassification = false;
uint8_t fallbackId[5];
bool haveReportedFallback = false;

volatile uint16_t missedAdcStarts = 0;
volatile uint16_t lateAdcReads = 0;

ISR(TIMER2_COMPA_vect) {
  if (ADCSRA & _BV(ADSC)) {
    missedAdcStarts++;
  } else {
    if (ADCSRA & _BV(ADIF)) lateAdcReads++;
    ADCSRA |= _BV(ADSC);
  }
}

void initDecoder(Decoder &decoder) {
  decoder.header = 0;
  decoder.pos = SEARCHING;
  decoder.rowXor = 0;
  decoder.colXor = 0;
  decoder.nibble = 0;
  memset(decoder.id, 0, sizeof(decoder.id));
  decoder.trailingData = 0;
  decoder.trailingBits = 0;
}

void beginPayload(Decoder &decoder) {
  decoder.pos = 0;
  decoder.rowXor = 0;
  decoder.colXor = 0;
  decoder.nibble = 0;
  memset(decoder.id, 0, sizeof(decoder.id));
}

void rejectPayload(Decoder &decoder) {
  decoder.pos = SEARCHING;
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
    const uint8_t nibble =
        (row & 1U) ? (value & 0x0F) : (value >> 4);
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

void noteCompleteCycle(const uint8_t id[5], uint64_t trailing) {
  // Adjacent phase candidates can decode the same physical transmission.
  if (haveLastCycleSample &&
      trailing == cycleTailCandidate &&
      memcmp(id, cycleIdCandidate, 5) == 0 &&
      (uint32_t)(totalSamples - lastCycleSample) < 512UL) {
    return;
  }
  lastCycleSample = totalSamples;
  haveLastCycleSample = true;

  if (cycleVotes != 0 &&
      trailing == cycleTailCandidate &&
      memcmp(id, cycleIdCandidate, 5) == 0) {
    if (cycleVotes < 255) cycleVotes++;
  } else {
    cycleTailCandidate = trailing;
    memcpy(cycleIdCandidate, id, 5);
    cycleVotes = 1;
  }

  if (cycleVotes >= 3 &&
      (!havePrintedCycleClassification ||
       trailing != printedCycleTail ||
       memcmp(id, printedCycleId, 5) != 0)) {
    confirmedCycleTail = trailing;
    memcpy(confirmedCycleId, id, 5);
    cycleClassificationPending = true;
  }
}

void noteValidFrame(const uint8_t id[5], DecodeMode mode) {
  framesThisBlock[mode]++;

  // Adjacent sample phases may decode the same physical frame. Count those as
  // one observation; a repeated RF/64 frame is hundreds of samples later.
  if (haveLastAccepted &&
      memcmp(id, lastAcceptedId, 5) == 0 &&
      (uint32_t)(totalSamples - lastAcceptedSample) < 256UL) {
    return;
  }

  memcpy(lastAcceptedId, id, 5);
  lastAcceptedSample = totalSamples;
  haveLastAccepted = true;

  if (voteCount != 0 && memcmp(id, voteId, 5) == 0) {
    if (voteCount < 255) voteCount++;
  } else {
    memcpy(voteId, id, 5);
    voteCount = 1;
  }

  if (voteCount >= 3 &&
      (!havePrintedId || memcmp(voteId, printedId, 5) != 0)) {
    if (lockedMode == MODE_COUNT) lockedMode = mode;
    memcpy(confirmedId, voteId, 5);
    confirmedMode = mode;
    confirmationPending = true;
  }
}

void feedBit(Decoder &decoder, uint8_t bit, DecodeMode mode) {
  if (decoder.trailingBits != 0) {
    decoder.trailingData = (decoder.trailingData << 1) | (bit & 1U);
    decoder.trailingBits--;
    if (decoder.trailingBits == 0) {
      noteCompleteCycle(decoder.id, decoder.trailingData);
      decoder.header = 0;
    }
    return;
  }

  decoder.header = ((decoder.header << 1) | bit) & 0x03FF;

  if (decoder.pos == SEARCHING) {
    // Stop bit zero followed by the nine header ones.
    if (decoder.header == 0x01FF) beginPayload(decoder);
    return;
  }

  const uint8_t pos = decoder.pos;

  if (pos < 50) {
    const uint8_t row = pos / 5;
    const uint8_t col = pos % 5;

    if (col < 4) {
      decoder.rowXor ^= bit;
      if (bit) decoder.colXor ^= _BV(col);
      decoder.nibble = (decoder.nibble << 1) | bit;
    } else {
      if ((decoder.rowXor ^ bit) != 0) {
        rejectPayload(decoder);
        return;
      }

      if ((row & 1) == 0) {
        decoder.id[row >> 1] = decoder.nibble << 4;
      } else {
        decoder.id[row >> 1] |= decoder.nibble;
      }
      decoder.rowXor = 0;
      decoder.nibble = 0;
    }
  } else if (pos < 54) {
    const uint8_t col = pos - 50;
    if (bit != ((decoder.colXor >> col) & 1)) {
      rejectPayload(decoder);
      return;
    }
  } else {
    if (bit == 0) {
      noteValidFrame(decoder.id, mode);
      if (mode == MANCHESTER_RF64) {
        decoder.trailingData = 0;
        decoder.trailingBits = 64;
      }
    }
    rejectPayload(decoder);
    return;
  }

  decoder.pos++;
}

void feedInvalid(Decoder &decoder) {
  decoder.header = 0;
  decoder.trailingBits = 0;
  rejectPayload(decoder);
}

void processManchesterWindow(int16_t difference, Decoder *decoders,
                             uint8_t phase, uint8_t threshold,
                             DecodeMode mode) {
  const uint16_t magnitude =
      difference < 0 ? (uint16_t)-difference : (uint16_t)difference;
  Decoder &normal = decoders[phase << 1];
  Decoder &inverse = decoders[(phase << 1) | 1];

  if (magnitude < threshold) {
    feedInvalid(normal);
    feedInvalid(inverse);
    return;
  }

  if (mode == MANCHESTER_RF64) {
    strongWindows64++;
    if (magnitude > blockMaxDiff64) blockMaxDiff64 = magnitude;
  } else {
    strongWindows32++;
    if (magnitude > blockMaxDiff32) blockMaxDiff32 = magnitude;
  }

  const uint8_t bit = difference > 0 ? 1 : 0;
  feedBit(normal, bit, mode);
  feedBit(inverse, bit ^ 1, mode);
}

void processBiphaseWindow(uint16_t first, uint16_t second,
                          BiphaseDecoder &candidate, uint8_t threshold,
                          DecodeMode mode) {
  if (!candidate.havePrevious) {
    candidate.previousSecondHalf = second;
    candidate.havePrevious = true;
    return;
  }

  const int16_t boundaryDifference =
      (int16_t)first - (int16_t)candidate.previousSecondHalf;
  const int16_t middleDifference = (int16_t)second - (int16_t)first;
  candidate.previousSecondHalf = second;

  const uint16_t boundaryMagnitude =
      boundaryDifference < 0
          ? (uint16_t)-boundaryDifference
          : (uint16_t)boundaryDifference;
  const uint16_t middleMagnitude =
      middleDifference < 0
          ? (uint16_t)-middleDifference
          : (uint16_t)middleDifference;

  // Biphase always transitions at the bit boundary. A zero has an additional
  // middle transition; a one keeps the same level through the whole bit.
  if (boundaryMagnitude < threshold) {
    feedInvalid(candidate.frame);
    return;
  }

  const uint8_t bit = middleMagnitude >= threshold ? 0 : 1;
  feedBit(candidate.frame, bit, mode);
}

void processSample(uint16_t sample) {
  totalSamples++;
  blockSamples++;
  blockSum += sample;
  if (sample < blockMin) blockMin = sample;
  if (sample > blockMax) blockMax = sample;

  if (ringFill < 8) {
    sampleRing[ringPos] = sample;
    ringPos = (ringPos + 1) & 0x07;
    ringFill++;

    if (ringFill == 8) {
      firstHalfSum = 0;
      secondHalfSum = 0;
      for (uint8_t i = 0; i < 4; i++) firstHalfSum += sampleRing[i];
      for (uint8_t i = 4; i < 8; i++) secondHalfSum += sampleRing[i];

      const int16_t difference =
          (int16_t)secondHalfSum - (int16_t)firstHalfSum;
      const uint8_t threshold = 4 * MIN_AVERAGE_LEVEL_DIFF;
      const uint8_t phase = totalSamples & 0x07;

      if (lockedMode == MODE_COUNT || lockedMode == MANCHESTER_RF64) {
        processManchesterWindow(difference, manchester64, phase, threshold,
                                MANCHESTER_RF64);
      }
      if (lockedMode == MODE_COUNT || lockedMode == BIPHASE_RF64) {
        processBiphaseWindow(firstHalfSum, secondHalfSum, biphase64[phase],
                             threshold, BIPHASE_RF64);
      }
    }
  } else {
    const uint8_t middlePos = (ringPos + 4) & 0x07;
    const uint16_t oldFirst = sampleRing[ringPos];
    const uint16_t oldSecond = sampleRing[middlePos];

    firstHalfSum = firstHalfSum - oldFirst + oldSecond;
    secondHalfSum = secondHalfSum - oldSecond + sample;
    sampleRing[ringPos] = sample;
    ringPos = (ringPos + 1) & 0x07;

    const int16_t difference =
        (int16_t)secondHalfSum - (int16_t)firstHalfSum;
    const uint8_t threshold = 4 * MIN_AVERAGE_LEVEL_DIFF;
    const uint8_t phase = totalSamples & 0x07;

    if (lockedMode == MODE_COUNT || lockedMode == MANCHESTER_RF64) {
      processManchesterWindow(difference, manchester64, phase, threshold,
                              MANCHESTER_RF64);
    }
    if (lockedMode == MODE_COUNT || lockedMode == BIPHASE_RF64) {
      processBiphaseWindow(firstHalfSum, secondHalfSum, biphase64[phase],
                           threshold, BIPHASE_RF64);
    }
  }

  if (lockedMode == MANCHESTER_RF64 || lockedMode == BIPHASE_RF64) return;

  if (ringFill32 < 4) {
    sampleRing32[ringPos32] = sample;
    ringPos32 = (ringPos32 + 1) & 0x03;
    ringFill32++;

    if (ringFill32 == 4) {
      firstHalfSum32 = 0;
      secondHalfSum32 = 0;
      for (uint8_t i = 0; i < 2; i++) firstHalfSum32 += sampleRing32[i];
      for (uint8_t i = 2; i < 4; i++) secondHalfSum32 += sampleRing32[i];
    } else {
      return;
    }
  } else {
    const uint8_t middlePos = (ringPos32 + 2) & 0x03;
    const uint16_t oldFirst = sampleRing32[ringPos32];
    const uint16_t oldSecond = sampleRing32[middlePos];

    firstHalfSum32 = firstHalfSum32 - oldFirst + oldSecond;
    secondHalfSum32 = secondHalfSum32 - oldSecond + sample;
    sampleRing32[ringPos32] = sample;
    ringPos32 = (ringPos32 + 1) & 0x03;
  }

  const int16_t difference32 =
      (int16_t)secondHalfSum32 - (int16_t)firstHalfSum32;
  const uint8_t threshold32 = 2 * MIN_AVERAGE_LEVEL_DIFF;
  const uint8_t phase32 = totalSamples & 0x03;

  if (lockedMode == MODE_COUNT || lockedMode == MANCHESTER_RF32) {
    processManchesterWindow(difference32, manchester32, phase32, threshold32,
                            MANCHESTER_RF32);
  }
  if (lockedMode == MODE_COUNT || lockedMode == BIPHASE_RF32) {
    processBiphaseWindow(firstHalfSum32, secondHalfSum32, biphase32[phase32],
                         threshold32, BIPHASE_RF32);
  }
}

void initHardware() {
  pinMode(PIN_CARRIER, OUTPUT);
  digitalWrite(PIN_CARRIER, LOW);

  // ADC0 (A0), AVcc reference, 500 kHz ADC clock: about 26 us/conversion.
  ADMUX = _BV(REFS0);
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);
  ADCSRB = 0;
  DIDR0 |= _BV(ADC0D);

  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {}
  ADCSRA |= _BV(ADIF);

  const uint8_t savedSreg = SREG;
  cli();

  // Hold/reset both prescalers so the carrier and sample timer start together.
  GTCCR = _BV(TSM) | _BV(PSRSYNC) | _BV(PSRASY);

  TCCR1A = _BV(COM1A0);
  TCCR1B = _BV(WGM12) | _BV(CS10);
  TCNT1 = 0;
  OCR1A = 63;

  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS21);
  TCNT2 = 0;
  OCR2A = 127;
  TIMSK2 &= ~_BV(OCIE2A);
  TIFR2 = _BV(OCF2A);

  GTCCR = 0;
  SREG = savedSreg;
}

void resumeSampling() {
  ADCSRA |= _BV(ADIF);
  TIFR2 = _BV(OCF2A);
  TIMSK2 |= _BV(OCIE2A);
}

void pauseSampling() {
  TIMSK2 &= ~_BV(OCIE2A);
  while (ADCSRA & _BV(ADSC)) {}
  ADCSRA |= _BV(ADIF);
}

uint16_t readSynchronousSample() {
  while (!(ADCSRA & _BV(ADIF))) {}
  const uint16_t sample = ADC;
  ADCSRA |= _BV(ADIF);
  return sample;
}

void printHexId(const uint8_t id[5]) {
  for (uint8_t i = 0; i < 5; i++) {
    if (id[i] < 16) Serial.print('0');
    Serial.print(id[i], HEX);
  }
}

void printHex64(uint64_t value) {
  for (int8_t nibble = 15; nibble >= 0; nibble--) {
    Serial.print((uint8_t)((value >> (nibble * 4)) & 0x0F), HEX);
  }
}

const __FlashStringHelper *modeName(DecodeMode mode) {
  switch (mode) {
    case MANCHESTER_RF64: return F("Manchester RF/64");
    case MANCHESTER_RF32: return F("Manchester RF/32");
    case BIPHASE_RF64: return F("Biphase RF/64");
    case BIPHASE_RF32: return F("Biphase RF/32");
    default: return F("searching");
  }
}

void reportAndResetBlock() {
  pauseSampling();

  Serial.print(F("ADC mean="));
  Serial.print(blockSamples ? blockSum / blockSamples : 0);
  Serial.print(F(" min="));
  Serial.print(blockMin);
  Serial.print(F(" max="));
  Serial.print(blockMax);
  Serial.print(F(" M64="));
  Serial.print(blockSamples ? (strongWindows64 * 100UL) / blockSamples : 0);
  Serial.print('%');
  Serial.print('/');
  Serial.print(blockMaxDiff64);
  Serial.print(F(" M32="));
  Serial.print(blockSamples ? (strongWindows32 * 100UL) / blockSamples : 0);
  Serial.print('%');
  Serial.print('/');
  Serial.print(blockMaxDiff32);
  Serial.print(F(" frames[M64,M32,B64,B32]="));

  for (uint8_t i = 0; i < MODE_COUNT; i++) {
    if (i) Serial.print(',');
    Serial.print(framesThisBlock[i]);
  }

  Serial.print(F(" vote="));
  Serial.print(voteCount);
  Serial.print(F(" lock="));
  Serial.print(modeName(lockedMode));
  Serial.print(F(" missed="));
  Serial.print(missedAdcStarts);
  Serial.print(F(" late="));
  Serial.println(lateAdcReads);

  if (confirmationPending) {
    Serial.print(F("EM4100 ID: "));
    printHexId(confirmedId);
    Serial.print(F("  ("));
    Serial.print(modeName(confirmedMode));
    Serial.println(')');

    memcpy(printedId, confirmedId, 5);
    havePrintedId = true;
    confirmationPending = false;
  }

  if (cycleClassificationPending) {
    const uint64_t repeatedFrame = encodeEm4100(confirmedCycleId);
    Serial.print(F("Classification for ID "));
    printHexId(confirmedCycleId);
    Serial.println(':');
    if (confirmedCycleTail == repeatedFrame) {
      Serial.println(F("Transmission length: 64 bits (validated frame repeats)."));
      Serial.println(F("No distinct second 64-bit segment was observed."));
    } else {
      Serial.println(F("Transmission length: 128-bit candidate."));
      Serial.print(F("First 64 bits:  "));
      printHex64(repeatedFrame);
      Serial.println();
      Serial.print(F("Second 64 bits: "));
      printHex64(confirmedCycleTail);
      Serial.println();
    }

    printedCycleTail = confirmedCycleTail;
    memcpy(printedCycleId, confirmedCycleId, 5);
    havePrintedCycleClassification = true;
    cycleClassificationPending = false;
  }

  const bool classifiedCurrentId =
      havePrintedCycleClassification &&
      memcmp(voteId, printedCycleId, 5) == 0;
  if (!cycleClassificationPending &&
      voteCount >= 6 &&
      !classifiedCurrentId &&
      (!haveReportedFallback || memcmp(voteId, fallbackId, 5) != 0)) {
    Serial.print(F("Fallback for ID "));
    printHexId(voteId);
    Serial.println(':');
    Serial.println(F("Decode mode: validated 64-bit EM4100 frame only."));
    Serial.println(F("Total transmission length is undetermined."));
    Serial.println(F("A valid repeating ID was found, but no stable distinct"));
    Serial.println(F("second 64-bit segment could be decoded."));
    memcpy(fallbackId, voteId, 5);
    haveReportedFallback = true;
  }

  Serial.flush();

  blockSamples = 0;
  blockSum = 0;
  strongWindows64 = 0;
  strongWindows32 = 0;
  blockMin = 1023;
  blockMax = 0;
  blockMaxDiff64 = 0;
  blockMaxDiff32 = 0;
  for (uint8_t i = 0; i < MODE_COUNT; i++) framesThisBlock[i] = 0;
  missedAdcStarts = 0;
  lateAdcReads = 0;

  resumeSampling();
}

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 16; i++) initDecoder(manchester64[i]);
  for (uint8_t i = 0; i < 8; i++) initDecoder(manchester32[i]);

  for (uint8_t i = 0; i < 8; i++) {
    initDecoder(biphase64[i].frame);
    biphase64[i].previousSecondHalf = 0;
    biphase64[i].havePrevious = false;
  }

  for (uint8_t i = 0; i < 4; i++) {
    initDecoder(biphase32[i].frame);
    biphase32[i].previousSecondHalf = 0;
    biphase32[i].havePrevious = false;
  }

  initHardware();

  Serial.println(F("EM4100 128/64-bit classifier ready."));
  Serial.println(F("A distinct stable suffix is required for a 128-bit result."));
  Serial.println(F("Keep the tag centered and still for several seconds."));
  Serial.flush();

  // Timer0 ISR jitter would occasionally move the phase-sensitive ADC trigger.
  TIMSK0 &= ~_BV(TOIE0);
  resumeSampling();
}

void loop() {
  processSample(readSynchronousSample());
  if (blockSamples >= REPORT_SAMPLES) reportAndResetBlock();
}
