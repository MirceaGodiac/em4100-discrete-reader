/*
 * Unified EM4100 reader and T5577 writer for Arduino Uno / ATmega328P.
 * Authorized tags only. D9 direct tank drive is experimental; keep the A0
 * envelope voltage within the Arduino input limits before connecting it.
 */

#include <Arduino.h>
#include <string.h>

const uint8_t PIN_CARRIER = 9;
const uint8_t SEARCHING = 0xFF;
const uint8_t CAPTURE_THRESHOLD = 8;
const uint8_t REQUIRED_CYCLES = 3;
const uint16_t DEDUP_SAMPLES = 128;
const uint32_t READ_REPORT_SAMPLES = 15625UL;
const uint32_t VERIFY_SAMPLES = 46875UL;

const uint32_t CONFIG_STANDARD = 0x00148040UL;
const uint32_t CONFIG_EXTENDED = 0x00148080UL;
const uint16_t POWER_UP_CLOCKS = 400;
const uint8_t START_GAP_CLOCKS = 30;
const uint8_t WRITE_GAP_CLOCKS = 19;
const uint8_t ZERO_CLOCKS = 24;
const uint8_t ONE_CLOCKS = 54;
const uint16_t PROGRAM_HOLD_CLOCKS = 900;

enum PreparedKind : uint8_t {
  PREPARED_NONE,
  PREPARED_64,
  PREPARED_128
};

struct Decoder {
  uint16_t header;
  uint8_t pos;
  uint8_t rowXor;
  uint8_t colXor;
  uint8_t nibble;
  uint8_t id[5];
  uint64_t suffix;
  uint8_t suffixBits;
  uint8_t cycleId[5];
  uint64_t cycleSuffix;
  uint32_t lastCycleSample;
  uint8_t stableStreak;
  uint8_t exactStreak;
  bool haveCycleCandidate;
};

struct CaptureResult {
  uint16_t adcMin;
  uint16_t adcMax;
  uint16_t missed;
  uint16_t late;
  uint16_t validBases;
  uint16_t preparedBases;
  uint16_t exactCycles;
  uint16_t stableCycles;
  uint64_t stableSuffix;
  uint64_t lastWrongSuffix;
  uint8_t stableId[5];
  uint8_t lastValidId[5];
  bool haveStable;
  bool haveValidId;
  bool haveWrongSuffix;
};

Decoder decoders[16];  // Eight sample phases, both envelope polarities.
uint16_t sampleRing[8];
uint16_t firstHalfSum;
uint16_t secondHalfSum;
uint8_t ringPos;
uint8_t ringFill;
uint32_t sampleCount;
uint16_t captureMin;
uint16_t captureMax;
volatile uint16_t missedSamples;
volatile uint16_t lateSamples;
uint8_t savedTimer0Mask;

uint16_t validBaseCount;
uint16_t preparedBaseCount;
uint16_t exactCycleCount;
uint32_t lastBaseSample;
uint32_t lastPreparedBaseSample;
bool haveBaseSample;
bool havePreparedBaseSample;
uint8_t lastValidId[5];
bool haveLastValidId;

uint8_t expectedId[5];
uint64_t expectedSuffix;
bool trackExpected;
uint64_t lastWrongSuffix;
bool haveWrongSuffix;

uint8_t stableId[5];
uint64_t stableSuffix;
uint16_t stableCycles;
bool haveStableCandidate;

uint8_t preparedId[5];
uint64_t preparedFrame;
uint64_t preparedSuffix;
uint32_t preparedBlocks[4];
PreparedKind preparedKind = PREPARED_NONE;
bool recoveryRequired;
bool readMode;

char serialBuffer[48];
uint8_t serialLength;
bool discardSerialLine;
bool readLineReady;
bool readLineOverflow;

ISR(TIMER2_COMPA_vect) {
  if (ADCSRA & _BV(ADSC)) {
    if (missedSamples != 0xFFFF) missedSamples++;
  } else {
    if ((ADCSRA & _BV(ADIF)) && lateSamples != 0xFFFF) lateSamples++;
    ADCSRA |= _BV(ADSC);
  }
}

void carrierOn() {
  TIFR1 = _BV(OCF1A);
  TCCR1A |= _BV(COM1A0);
}

void carrierOff() {
  TCCR1A &= (uint8_t)~_BV(COM1A0);
  PORTB &= (uint8_t)~_BV(PORTB1);
}

void configureCarrier() {
  TCCR1A = _BV(COM1A0);
  TCCR1B = _BV(WGM12) | _BV(CS10);
  TCNT1 = 0;
  OCR1A = 63;  // Toggle every 64 CPU clocks: 125 kHz on OC1A/D9.
}

void waitFieldClocks(uint16_t clocks) {
  uint32_t microseconds = (uint32_t)clocks * 8UL;
  while (microseconds > 16000UL) {
    delayMicroseconds(16000);
    microseconds -= 16000UL;
  }
  delayMicroseconds((uint16_t)microseconds);
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
  sendDownlinkBit(0);  // Page-0 write opcode 10.
  sendDownlinkBit(0);  // Lock bit clear; password mode is not used.
  for (int8_t bit = 31; bit >= 0; bit--) {
    sendDownlinkBit((data >> bit) & 1U);
  }
  sendDownlinkBit((block >> 2) & 1U);
  sendDownlinkBit((block >> 1) & 1U);
  sendDownlinkBit(block & 1U);
  waitFieldClocks(PROGRAM_HOLD_CLOCKS);
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

void resetDecoder(Decoder &decoder) {
  decoder.header = 0;
  decoder.pos = SEARCHING;
  decoder.rowXor = 0;
  decoder.colXor = 0;
  decoder.nibble = 0;
  memset(decoder.id, 0, sizeof(decoder.id));
  decoder.suffix = 0;
  decoder.suffixBits = 0;
  memset(decoder.cycleId, 0, sizeof(decoder.cycleId));
  decoder.cycleSuffix = 0;
  decoder.lastCycleSample = 0;
  decoder.stableStreak = 0;
  decoder.exactStreak = 0;
  decoder.haveCycleCandidate = false;
}

void beginPayload(Decoder &decoder) {
  decoder.pos = 0;
  decoder.rowXor = 0;
  decoder.colXor = 0;
  decoder.nibble = 0;
  memset(decoder.id, 0, sizeof(decoder.id));
}

void rejectDecoder(Decoder &decoder) {
  decoder.header = 0;
  decoder.pos = SEARCHING;
  decoder.suffixBits = 0;
}

bool sufficientlySeparated(uint32_t previous, bool havePrevious) {
  return !havePrevious ||
         (uint32_t)(sampleCount - previous) >= DEDUP_SAMPLES;
}

__attribute__((noinline))
void noteValidBase(const uint8_t id[5]) {
  if (sufficientlySeparated(lastBaseSample, haveBaseSample)) {
    if (validBaseCount != 0xFFFF) validBaseCount++;
    lastBaseSample = sampleCount;
    haveBaseSample = true;
    memcpy(lastValidId, id, sizeof(lastValidId));
    haveLastValidId = true;
  }
  if (trackExpected &&
      memcmp(id, expectedId, sizeof(expectedId)) == 0 &&
      sufficientlySeparated(lastPreparedBaseSample, havePreparedBaseSample)) {
    if (preparedBaseCount != 0xFFFF) preparedBaseCount++;
    lastPreparedBaseSample = sampleCount;
    havePreparedBaseSample = true;
  }
}

__attribute__((noinline))
void noteCompleteCycle(Decoder &decoder) {
  // One base-plus-suffix period is exactly 128 bits * 8 samples. Requiring
  // exact spacing rejects streams with an additional active data block.
  const bool consecutive =
      decoder.haveCycleCandidate &&
      (uint32_t)(sampleCount - decoder.lastCycleSample) == 1024UL;
  const bool sameCycle =
      decoder.haveCycleCandidate &&
      decoder.suffix == decoder.cycleSuffix &&
      memcmp(decoder.id, decoder.cycleId, sizeof(decoder.id)) == 0;

  if (consecutive && sameCycle) {
    if (decoder.stableStreak != 255) decoder.stableStreak++;
  } else {
    decoder.stableStreak = 1;
    memcpy(decoder.cycleId, decoder.id, sizeof(decoder.cycleId));
    decoder.cycleSuffix = decoder.suffix;
  }

  if (decoder.stableStreak > stableCycles) {
    stableCycles = decoder.stableStreak;
    memcpy(stableId, decoder.id, sizeof(stableId));
    stableSuffix = decoder.suffix;
    haveStableCandidate = true;
  }

  const bool exact =
      trackExpected &&
      decoder.suffix == expectedSuffix &&
      memcmp(decoder.id, expectedId, sizeof(expectedId)) == 0;
  if (exact) {
    if (consecutive && decoder.exactStreak != 0) {
      if (decoder.exactStreak != 255) decoder.exactStreak++;
    } else {
      decoder.exactStreak = 1;
    }
    if (decoder.exactStreak > exactCycleCount) {
      exactCycleCount = decoder.exactStreak;
    }
  } else {
    decoder.exactStreak = 0;
    if (trackExpected &&
        memcmp(decoder.id, expectedId, sizeof(expectedId)) == 0) {
      lastWrongSuffix = decoder.suffix;
      haveWrongSuffix = true;
    }
  }

  decoder.lastCycleSample = sampleCount;
  decoder.haveCycleCandidate = true;
}

void feedBit(Decoder &decoder, uint8_t bit) {
  if (decoder.suffixBits != 0) {
    decoder.suffix = (decoder.suffix << 1) | (bit & 1U);
    if (--decoder.suffixBits == 0) {
      noteCompleteCycle(decoder);
      decoder.header = 0;
      decoder.pos = SEARCHING;
    }
    return;
  }

  decoder.header = ((decoder.header << 1) | (bit & 1U)) & 0x03FF;
  if (decoder.pos == SEARCHING) {
    // A stop zero followed by all nine EM4100 header ones.
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
        rejectDecoder(decoder);
        return;
      }
      if ((row & 1U) == 0) {
        decoder.id[row >> 1] = decoder.nibble << 4;
      } else {
        decoder.id[row >> 1] |= decoder.nibble;
      }
      decoder.rowXor = 0;
      decoder.nibble = 0;
    }
  } else if (pos < 54) {
    const uint8_t col = pos - 50;
    if (bit != ((decoder.colXor >> col) & 1U)) {
      rejectDecoder(decoder);
      return;
    }
  } else {
    if (bit != 0) {
      rejectDecoder(decoder);
      return;
    }
    noteValidBase(decoder.id);
    decoder.header = 0;
    decoder.pos = SEARCHING;
    decoder.suffix = 0;
    decoder.suffixBits = 64;
    return;
  }
  decoder.pos++;
}

inline __attribute__((always_inline))
void processManchesterWindow(int16_t difference, uint8_t phase) {
  const uint16_t magnitude =
      difference < 0 ? (uint16_t)-difference : (uint16_t)difference;
  Decoder &normal = decoders[phase << 1];
  Decoder &inverse = decoders[(phase << 1) | 1];
  if (magnitude < CAPTURE_THRESHOLD) {
    rejectDecoder(normal);
    rejectDecoder(inverse);
    return;
  }
  const uint8_t bit = difference > 0 ? 1 : 0;
  feedBit(normal, bit);
  feedBit(inverse, bit ^ 1U);
}

void processSample(uint16_t sample) {
  sampleCount++;
  if (sample < captureMin) captureMin = sample;
  if (sample > captureMax) captureMax = sample;

  if (ringFill < 8) {
    sampleRing[ringPos] = sample;
    ringPos = (ringPos + 1) & 0x07;
    if (++ringFill != 8) return;
    firstHalfSum = 0;
    secondHalfSum = 0;
    for (uint8_t i = 0; i < 4; i++) firstHalfSum += sampleRing[i];
    for (uint8_t i = 4; i < 8; i++) secondHalfSum += sampleRing[i];
  } else {
    const uint8_t middle = (ringPos + 4) & 0x07;
    const uint16_t oldFirst = sampleRing[ringPos];
    const uint16_t oldSecond = sampleRing[middle];
    firstHalfSum = firstHalfSum - oldFirst + oldSecond;
    secondHalfSum = secondHalfSum - oldSecond + sample;
    sampleRing[ringPos] = sample;
    ringPos = (ringPos + 1) & 0x07;
  }

  processManchesterWindow(
      (int16_t)secondHalfSum - (int16_t)firstHalfSum,
      sampleCount & 0x07);
}

void startCapture(bool useExpected, const uint8_t id[5], uint64_t suffix) {
  Serial.flush();
  trackExpected = useExpected;
  if (useExpected) {
    memcpy(expectedId, id, sizeof(expectedId));
    expectedSuffix = suffix;
  }

  for (uint8_t i = 0; i < 16; i++) resetDecoder(decoders[i]);
  ringPos = 0;
  ringFill = 0;
  firstHalfSum = 0;
  secondHalfSum = 0;
  sampleCount = 0;
  captureMin = 1023;
  captureMax = 0;
  missedSamples = 0;
  lateSamples = 0;
  validBaseCount = 0;
  preparedBaseCount = 0;
  exactCycleCount = 0;
  lastBaseSample = 0;
  lastPreparedBaseSample = 0;
  haveBaseSample = false;
  havePreparedBaseSample = false;
  haveLastValidId = false;
  lastWrongSuffix = 0;
  haveWrongSuffix = false;
  stableSuffix = 0;
  stableCycles = 0;
  haveStableCandidate = false;

  ADMUX = _BV(REFS0);  // AVcc reference and ADC0/A0.
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);  // 500 kHz ADC clock.
  ADCSRB = 0;
  DIDR0 |= _BV(ADC0D);
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {}
  ADCSRA |= _BV(ADIF);

  const uint8_t savedSreg = SREG;
  cli();
  savedTimer0Mask = TIMSK0;
  TIMSK0 &= (uint8_t)~_BV(TOIE0);
  TIMSK2 = 0;

  // Hold and reset both prescalers so D9 and the 64 us ADC trigger start from
  // one known boundary and preserve a fixed carrier sampling phase.
  GTCCR = _BV(TSM) | _BV(PSRSYNC) | _BV(PSRASY);
  configureCarrier();
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS21);
  TCNT2 = 0;
  OCR2A = 127;
  TIFR2 = _BV(OCF2A);
  TIMSK2 = _BV(OCIE2A);
  GTCCR = 0;
  SREG = savedSreg;
}

uint16_t readSynchronousSample() {
  while (!(ADCSRA & _BV(ADIF))) {}
  const uint16_t sample = ADC;
  ADCSRA |= _BV(ADIF);
  return sample;
}

CaptureResult finishCapture() {
  const uint8_t savedSreg = SREG;
  cli();
  TIMSK2 = 0;
  TCCR2B = 0;
  ADCSRA &= (uint8_t)~_BV(ADEN);
  TIFR0 = _BV(TOV0);  // Do not deliver one stale Timer0 overflow on restore.
  TIMSK0 = savedTimer0Mask;
  SREG = savedSreg;

  CaptureResult result;
  result.adcMin = captureMin;
  result.adcMax = captureMax;
  result.missed = missedSamples;
  result.late = lateSamples;
  result.validBases = validBaseCount;
  result.preparedBases = preparedBaseCount;
  result.exactCycles = exactCycleCount;
  result.stableCycles = stableCycles;
  result.stableSuffix = stableSuffix;
  result.lastWrongSuffix = lastWrongSuffix;
  memcpy(result.stableId, stableId, sizeof(result.stableId));
  memcpy(result.lastValidId, lastValidId, sizeof(result.lastValidId));
  result.haveStable = haveStableCandidate && stableCycles >= REQUIRED_CYCLES;
  result.haveValidId = haveLastValidId;
  result.haveWrongSuffix = haveWrongSuffix;
  return result;
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

void printId(const uint8_t id[5]) {
  for (uint8_t i = 0; i < 5; i++) {
    if (id[i] < 16) Serial.print('0');
    Serial.print(id[i], HEX);
  }
}

void printCaptureResult(const __FlashStringHelper *label,
                        const CaptureResult &result) {
  Serial.print(label);
  Serial.print(F(": ADC="));
  Serial.print(result.adcMin);
  Serial.print('-');
  Serial.print(result.adcMax);
  Serial.print(F(" missed="));
  Serial.print(result.missed);
  Serial.print(F(" late="));
  Serial.print(result.late);
  Serial.print(F(" validBase="));
  Serial.print(result.validBases);
  Serial.print(F(" preparedBase="));
  Serial.print(result.preparedBases);
  Serial.print(F(" exactCycle="));
  Serial.print(result.exactCycles);
  if (result.haveWrongSuffix) {
    Serial.print(F(" lastWrongSuffix=0x"));
    printHex64(result.lastWrongSuffix);
  } else {
    Serial.print(F(" lastWrongSuffix=none"));
  }
  Serial.println();
}

bool captureHealthy(const CaptureResult &result) {
  return result.missed == 0 && result.late == 0;
}

CaptureResult runBlockingCapture(const uint8_t id[5], uint64_t suffix) {
  startCapture(true, id, suffix);
  while (sampleCount < VERIFY_SAMPLES &&
         exactCycleCount < REQUIRED_CYCLES) {
    processSample(readSynchronousSample());
  }
  return finishCapture();
}

bool verifyExact(const uint8_t id[5], uint64_t suffix,
                 const __FlashStringHelper *label) {
  const CaptureResult result = runBlockingCapture(id, suffix);
  printCaptureResult(label, result);
  return captureHealthy(result) &&
         result.exactCycles >= REQUIRED_CYCLES;
}

void printReadReport(const CaptureResult &result) {
  printCaptureResult(F("READ"), result);
  if (result.haveStable) {
    const uint64_t base = encodeEm4100(result.stableId);
    Serial.print(F("EM4100 ID="));
    printId(result.stableId);
    Serial.print(F(" stableCycles="));
    Serial.println(result.stableCycles);
    if (result.stableSuffix == base) {
      Serial.print(F("Standard repeating 64-bit: 0x"));
      printHex64(base);
      Serial.println();
    } else {
      Serial.print(F("128-bit candidate: ID="));
      printId(result.stableId);
      Serial.print(F(" base64=0x"));
      printHex64(base);
      Serial.print(F(" suffix64=0x"));
      printHex64(result.stableSuffix);
      Serial.println();
    }
  } else if (result.validBases != 0) {
    Serial.print(F("Length unknown: parity-valid base"));
    if (result.haveValidId) {
      Serial.print(F(", last ID="));
      printId(result.lastValidId);
    }
    Serial.println(F(", but no stable following 64 bits."));
  } else {
    Serial.println(F("No parity-valid EM4100 base frame."));
  }
}

int8_t hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool parseIdExact(const char *text, uint8_t id[5]) {
  for (uint8_t i = 0; i < 5; i++) {
    const int8_t high = hexValue(text[i * 2]);
    const int8_t low = hexValue(text[i * 2 + 1]);
    if (high < 0 || low < 0) return false;
    id[i] = (uint8_t)((high << 4) | low);
  }
  return true;
}

bool parseHex64Exact(const char *text, uint64_t *valueOut) {
  uint64_t value = 0;
  for (uint8_t i = 0; i < 16; i++) {
    const int8_t digit = hexValue(text[i]);
    if (digit < 0) return false;
    value = (value << 4) | (uint8_t)digit;
  }
  *valueOut = value;
  return true;
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

bool prefixIgnoreCase(const char *text, const char *prefix) {
  while (*prefix) {
    char a = *text++;
    char b = *prefix++;
    if (a >= 'a' && a <= 'z') a -= 'a' - 'A';
    if (b >= 'a' && b <= 'z') b -= 'a' - 'A';
    if (a != b) return false;
  }
  return true;
}

void printPrepared() {
  if (preparedKind == PREPARED_NONE) {
    Serial.println(F("Prepared: none"));
    return;
  }
  Serial.print(F("Prepared "));
  Serial.print(preparedKind == PREPARED_64 ? F("64") : F("128"));
  Serial.print(F(": ID="));
  printId(preparedId);
  Serial.print(F(" base64=0x"));
  printHex64(preparedFrame);
  Serial.print(F(" suffix64=0x"));
  printHex64(preparedSuffix);
  for (uint8_t i = 0; i < (preparedKind == PREPARED_64 ? 2 : 4); i++) {
    Serial.print(F(" B"));
    Serial.print(i + 1);
    Serial.print(F("=0x"));
    printHex32(preparedBlocks[i]);
  }
  Serial.print(F(" B0=0x"));
  printHex32(preparedKind == PREPARED_64 ? CONFIG_STANDARD : CONFIG_EXTENDED);
  Serial.println();
  if (preparedKind == PREPARED_128 && preparedSuffix == preparedFrame) {
    Serial.println(F("Note: equal 128-bit halves are physically indistinguishable"));
    Serial.println(F("from a repeating 64-bit period by passive verification."));
  }
}

void prepare64(const char *command) {
  if (strlen(command) != 17 || command[6] != ' ') {
    Serial.println(F("Usage: PREP64 <10_HEX_DIGITS>"));
    return;
  }
  uint8_t id[5];
  if (!parseIdExact(command + 7, id)) {
    Serial.println(F("PREP64 requires exactly 10 hexadecimal digits."));
    return;
  }
  memcpy(preparedId, id, sizeof(preparedId));
  preparedFrame = encodeEm4100(preparedId);
  preparedSuffix = preparedFrame;
  preparedBlocks[0] = (uint32_t)(preparedFrame >> 32);
  preparedBlocks[1] = (uint32_t)preparedFrame;
  preparedBlocks[2] = 0;
  preparedBlocks[3] = 0;
  preparedKind = PREPARED_64;
  printPrepared();
}

void prepare128(const char *command) {
  if (strlen(command) != 35 || command[7] != ' ' ||
      command[18] != ' ') {
    Serial.println(F("Usage: PREP128 <10_HEX_DIGITS> <16_HEX_DIGITS>"));
    return;
  }
  uint8_t id[5];
  uint64_t suffix;
  if (!parseIdExact(command + 8, id) ||
      !parseHex64Exact(command + 19, &suffix)) {
    Serial.println(F("PREP128 requires exact hexadecimal ID and suffix."));
    return;
  }
  if (suffix & 1U) {
    Serial.println(F("PREP128 suffix must end in 0, 2, 4, 6, 8, A, C, or E."));
    Serial.println(F("A final 1 bit removes reliable EM4100 header re-sync."));
    return;
  }
  memcpy(preparedId, id, sizeof(preparedId));
  preparedFrame = encodeEm4100(preparedId);
  preparedSuffix = suffix;
  preparedBlocks[0] = (uint32_t)(preparedFrame >> 32);
  preparedBlocks[1] = (uint32_t)preparedFrame;
  preparedBlocks[2] = (uint32_t)(preparedSuffix >> 32);
  preparedBlocks[3] = (uint32_t)preparedSuffix;
  preparedKind = PREPARED_128;
  printPrepared();
}

void destructiveWarning(const __FlashStringHelper *action) {
  Serial.println(F("WARNING: destructive write to one unlocked authorized tag."));
  Serial.println(F("Keep the target centered; do not reset or remove power."));
  Serial.print(action);
  Serial.println(F(" begins in 2 seconds."));
  Serial.flush();
  delay(2000);
}

bool recoverStandardAttempts() {
  recoveryRequired = true;
  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    Serial.print(F("Recovery attempt "));
    Serial.print(attempt);
    Serial.println(F("/3: B1, B2, B0=00148040."));
    writeFreshBlock(1, preparedBlocks[0]);
    writeFreshBlock(2, preparedBlocks[1]);
    writeFreshBlock(0, CONFIG_STANDARD);
    powerCycleField();
    if (verifyExact(preparedId, preparedFrame, F("RECOVER VERIFY"))) {
      recoveryRequired = false;
      Serial.println(F("RECOVERED: 3 consecutive frame||frame cycles verified."));
      Serial.println(F("Prepared request retained; VERIFY still checks that request."));
      return true;
    }
  }
  Serial.println(F("RECOVERY FAILED: RAM lock and prepared base retained."));
  Serial.println(F("Keep power on and use RECOVER after correcting coupling."));
  return false;
}

void verifyCommand() {
  if (preparedKind == PREPARED_NONE) {
    Serial.println(F("Nothing prepared. Use PREP64 or PREP128."));
    return;
  }
  Serial.println(F("VERIFY is receive-only; no downlink or EEPROM mutation."));
  if (preparedKind == PREPARED_128 && preparedSuffix == preparedFrame) {
    Serial.println(F("Equal halves verify the bits, but physical 64/128 period"));
    Serial.println(F("cannot be distinguished passively."));
  }
  const bool passed =
      verifyExact(preparedId, preparedSuffix, F("VERIFY"));
  if (passed) {
    if (preparedKind == PREPARED_64) {
      Serial.println(F("VERIFY PASS: 3 consecutive exact frame||frame cycles."));
    } else {
      Serial.println(F("VERIFY PASS: 3 consecutive exact base+suffix cycles."));
    }
  } else {
    Serial.println(F("VERIFY FAIL: three exact healthy cycles were not proved."));
  }
}

void writeCommand() {
  if (preparedKind == PREPARED_NONE) {
    Serial.println(F("Nothing prepared. Use PREP64 or PREP128."));
    return;
  }
  destructiveWarning(F("WRITE"));
  recoveryRequired = true;  // Set before the first mutating downlink.

  if (preparedKind == PREPARED_64) {
    Serial.println(F("Writing B1, B2, then B0 last."));
    writeFreshBlock(1, preparedBlocks[0]);
    writeFreshBlock(2, preparedBlocks[1]);
    writeFreshBlock(0, CONFIG_STANDARD);
  } else {
    Serial.println(F("Writing B1, B2, B3, B4, then B0 last."));
    writeFreshBlock(1, preparedBlocks[0]);
    writeFreshBlock(2, preparedBlocks[1]);
    writeFreshBlock(3, preparedBlocks[2]);
    writeFreshBlock(4, preparedBlocks[3]);
    writeFreshBlock(0, CONFIG_EXTENDED);
  }
  powerCycleField();

  if (verifyExact(preparedId, preparedSuffix, F("WRITE VERIFY"))) {
    recoveryRequired = false;
    Serial.println(F("WRITE SUCCESS: 3 consecutive exact cycles verified healthy."));
    Serial.println(F("Prepared data retained so VERIFY can be repeated."));
    return;
  }

  Serial.println(F("WRITE VERIFY FAILED: automatic standard recovery starts."));
  recoverStandardAttempts();
}

void recoverCommand() {
  if (preparedKind == PREPARED_NONE) {
    Serial.println(F("No prepared recovery base is available."));
    return;
  }
  destructiveWarning(F("RECOVER"));
  recoverStandardAttempts();
}

void cancelCommand() {
  memset(preparedId, 0, sizeof(preparedId));
  preparedFrame = 0;
  preparedSuffix = 0;
  memset(preparedBlocks, 0, sizeof(preparedBlocks));
  preparedKind = PREPARED_NONE;
  Serial.println(F("Prepared data cleared."));
}

void statusCommand() {
  Serial.print(F("Mode: "));
  Serial.println(readMode ? F("READ") : F("idle"));
  printPrepared();
  Serial.print(F("Recovery lock: "));
  Serial.println(recoveryRequired ? F("REQUIRED") : F("clear"));
  if (recoveryRequired) {
    Serial.println(F("Strict lock: only RECOVER, STATUS, and HELP are accepted."));
  }
  Serial.println(F("WARNING: prepared data and recovery lock are RAM-only."));
  Serial.println(F("Reset or power loss erases this recovery state."));
}

void showHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  READ"));
  Serial.println(F("  STOP"));
  Serial.println(F("  PREP64 <10_HEX_DIGITS>"));
  Serial.println(F("  PREP128 <10_HEX_DIGITS> <16_HEX_DIGITS>"));
  Serial.println(F("  VERIFY"));
  Serial.println(F("  WRITE"));
  Serial.println(F("  RECOVER"));
  Serial.println(F("  CANCEL"));
  Serial.println(F("  STATUS"));
  Serial.println(F("  HELP"));
  Serial.println(F("Manchester RF/64 only; no password or lock bits are set."));
}

void startReadMode() {
  readMode = true;
  serialLength = 0;
  discardSerialLine = false;
  readLineReady = false;
  readLineOverflow = false;
  Serial.println(F("READ started. Type STOP on its own line to return to idle."));
  startCapture(preparedKind != PREPARED_NONE, preparedId, preparedSuffix);
}

void handleIdleCommand(char *command) {
  if (command[0] == '\0') return;

  if (recoveryRequired &&
      !equalsIgnoreCase(command, "RECOVER") &&
      !equalsIgnoreCase(command, "STATUS") &&
      !equalsIgnoreCase(command, "HELP")) {
    Serial.println(F("Blocked by recovery lock: RECOVER, STATUS, or HELP only."));
    return;
  }

  if (equalsIgnoreCase(command, "READ")) {
    startReadMode();
  } else if (equalsIgnoreCase(command, "STOP")) {
    Serial.println(F("Already idle."));
  } else if (prefixIgnoreCase(command, "PREP64")) {
    prepare64(command);
  } else if (prefixIgnoreCase(command, "PREP128")) {
    prepare128(command);
  } else if (equalsIgnoreCase(command, "VERIFY")) {
    verifyCommand();
  } else if (equalsIgnoreCase(command, "WRITE")) {
    writeCommand();
  } else if (equalsIgnoreCase(command, "RECOVER")) {
    recoverCommand();
  } else if (equalsIgnoreCase(command, "CANCEL")) {
    cancelCommand();
  } else if (equalsIgnoreCase(command, "STATUS")) {
    statusCommand();
  } else if (equalsIgnoreCase(command, "HELP")) {
    showHelp();
  } else {
    Serial.println(F("Unknown or malformed command. Type HELP."));
  }
}

void ingestSerialByte(char c, bool duringRead) {
  if (c == '\r') return;
  if (c == '\n') {
    if (duringRead) {
      readLineOverflow = discardSerialLine;
      if (!discardSerialLine) serialBuffer[serialLength] = '\0';
      readLineReady = true;
    } else if (discardSerialLine) {
      Serial.println(F("Command too long; line discarded."));
    } else {
      serialBuffer[serialLength] = '\0';
      handleIdleCommand(serialBuffer);
    }
    serialLength = 0;
    discardSerialLine = false;
    return;
  }
  if (discardSerialLine) return;
  if (serialLength < sizeof(serialBuffer) - 1) {
    serialBuffer[serialLength++] = c;
  } else {
    serialLength = 0;
    discardSerialLine = true;
  }
}

void serviceReadMode() {
  processSample(readSynchronousSample());

  // Limit serial work to one byte after each completed sample-processing pass.
  if (Serial.available() > 0) {
    ingestSerialByte((char)Serial.read(), true);
  }

  if (readLineReady) {
    const CaptureResult result = finishCapture();
    printCaptureResult(F("READ paused"), result);
    if (!readLineOverflow && equalsIgnoreCase(serialBuffer, "STOP")) {
      readMode = false;
      Serial.println(F("READ stopped; idle."));
    } else {
      Serial.println(F("Command ignored during READ. Type STOP first."));
      startCapture(preparedKind != PREPARED_NONE, preparedId, preparedSuffix);
    }
    readLineReady = false;
    readLineOverflow = false;
    return;
  }

  if (sampleCount >= READ_REPORT_SAMPLES) {
    const CaptureResult result = finishCapture();
    printReadReport(result);
    startCapture(preparedKind != PREPARED_NONE, preparedId, preparedSuffix);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CARRIER, OUTPUT);
  digitalWrite(PIN_CARRIER, LOW);
  configureCarrier();

  Serial.println(F("Unified EM4100/T5577 CLI ready; idle at boot."));
  Serial.println(F("D9 carrier, A0 envelope, 115200 baud, Newline commands."));
  Serial.println(F("RAM recovery state is LOST on reset or power loss."));
  showHelp();
}

void loop() {
  if (readMode) {
    serviceReadMode();
    return;
  }

  while (Serial.available() > 0) {
    ingestSerialByte((char)Serial.read(), false);
    if (readMode) return;
  }
}
