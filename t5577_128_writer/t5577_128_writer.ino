/*
 * Guarded 128-bit ATA5577 writer for Arduino Uno / ATmega328P.
 *
 * D9 supplies the 125 kHz carrier and downlink gaps. A0 reads the passive
 * envelope detector. This is only for unlocked, writable ATA5577/T5577 tags
 * with real page-0 blocks B1 through B4. Password and lock bits stay clear.
 */

#include <Arduino.h>
#include <string.h>

const uint8_t PIN_CARRIER = 9;
const uint32_t CONFIG_128 = 0x00148080UL;
const uint32_t CONFIG_64 = 0x00148040UL;

const uint16_t POWER_UP_CLOCKS = 400;
const uint8_t START_GAP_CLOCKS = 30;
const uint8_t WRITE_GAP_CLOCKS = 19;
const uint8_t ZERO_CLOCKS = 24;
const uint8_t ONE_CLOCKS = 54;
const uint16_t PROGRAM_HOLD_CLOCKS = 900;

const uint8_t SEARCHING = 0xFF;
const uint8_t VERIFY_THRESHOLD = 8;
const uint8_t REQUIRED_OBSERVATIONS = 3;
const uint32_t VERIFY_SAMPLES = 46875UL;  // Three seconds at 64 us/sample.
const uint16_t DEDUP_SAMPLES = 128;

struct VerifyDecoder {
  uint16_t header;
  uint8_t pos;
  uint8_t rowXor;
  uint8_t colXor;
  uint8_t nibble;
  uint8_t id[5];
  uint64_t suffix;
  uint8_t suffixBits;
};

struct VerifyResult {
  uint16_t adcMin;
  uint16_t adcMax;
  uint16_t missed;
  uint16_t late;
  uint8_t validBaseFrames;
  uint8_t preparedBaseFrames;
  uint8_t exactCycles;
  uint64_t lastWrongSuffix;
  bool haveWrongSuffix;
};

uint8_t preparedId[5];
uint64_t preparedFrame;
uint64_t preparedSuffix;
uint32_t preparedBlocks[4];
bool prepared;
bool recoveryRequired;

VerifyDecoder decoders[16];  // Eight sample phases, both polarities.
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
uint8_t validBaseFrames;
uint8_t preparedBaseFrames;
uint8_t exactCycles;
uint32_t lastBaseSample;
uint32_t lastPreparedBaseSample;
uint32_t lastExactSample;
bool haveBaseSample;
bool havePreparedBaseSample;
bool haveExactSample;
uint64_t lastWrongSuffix;
bool haveWrongSuffix;
bool captureSuffix;
uint8_t savedTimer0Mask;

char serialBuffer[40];
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
  sendDownlinkBit(0);  // Lock bit remains clear.
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

void resetDecoder(VerifyDecoder &decoder) {
  decoder.header = 0;
  decoder.pos = SEARCHING;
  decoder.rowXor = 0;
  decoder.colXor = 0;
  decoder.nibble = 0;
  memset(decoder.id, 0, sizeof(decoder.id));
  decoder.suffix = 0;
  decoder.suffixBits = 0;
}

void beginPayload(VerifyDecoder &decoder) {
  decoder.pos = 0;
  decoder.rowXor = 0;
  decoder.colXor = 0;
  decoder.nibble = 0;
  memset(decoder.id, 0, sizeof(decoder.id));
}

void rejectDecoder(VerifyDecoder &decoder) {
  decoder.header = 0;
  decoder.pos = SEARCHING;
  decoder.suffixBits = 0;
}

bool distinctFrom(uint32_t previous, bool havePrevious) {
  return !havePrevious || (uint32_t)(sampleCount - previous) >= DEDUP_SAMPLES;
}

void noteValidBase(const uint8_t id[5]) {
  if (distinctFrom(lastBaseSample, haveBaseSample)) {
    if (validBaseFrames != 255) validBaseFrames++;
    lastBaseSample = sampleCount;
    haveBaseSample = true;
  }
  if (memcmp(id, preparedId, sizeof(preparedId)) == 0 &&
      distinctFrom(lastPreparedBaseSample, havePreparedBaseSample)) {
    if (preparedBaseFrames != 255) preparedBaseFrames++;
    lastPreparedBaseSample = sampleCount;
    havePreparedBaseSample = true;
  }
}

void noteCompleteCycle(const uint8_t id[5], uint64_t suffix) {
  if (memcmp(id, preparedId, sizeof(preparedId)) != 0) return;
  if (suffix == preparedSuffix) {
    if (distinctFrom(lastExactSample, haveExactSample)) {
      if (exactCycles != 255) exactCycles++;
      lastExactSample = sampleCount;
      haveExactSample = true;
    }
  } else {
    lastWrongSuffix = suffix;
    haveWrongSuffix = true;
  }
}

void feedVerifyBit(VerifyDecoder &decoder, uint8_t bit) {
  if (decoder.suffixBits != 0) {
    decoder.suffix = (decoder.suffix << 1) | (bit & 1U);
    if (--decoder.suffixBits == 0) {
      noteCompleteCycle(decoder.id, decoder.suffix);
      decoder.header = 0;
      decoder.pos = SEARCHING;
    }
    return;
  }

  decoder.header = ((decoder.header << 1) | (bit & 1U)) & 0x03FF;
  if (decoder.pos == SEARCHING) {
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
    if (captureSuffix) {
      decoder.suffix = 0;
      decoder.suffixBits = 64;
    }
    return;
  }
  decoder.pos++;
}

void processManchesterWindow(int16_t difference, uint8_t phase) {
  const uint16_t magnitude =
      difference < 0 ? (uint16_t)-difference : (uint16_t)difference;
  VerifyDecoder &normal = decoders[phase << 1];
  VerifyDecoder &inverse = decoders[(phase << 1) | 1];
  if (magnitude < VERIFY_THRESHOLD) {
    rejectDecoder(normal);
    rejectDecoder(inverse);
    return;
  }
  const uint8_t bit = difference > 0 ? 1 : 0;
  feedVerifyBit(normal, bit);
  feedVerifyBit(inverse, bit ^ 1U);
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

void startCapture(bool withSuffix) {
  captureSuffix = withSuffix;
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
  validBaseFrames = 0;
  preparedBaseFrames = 0;
  exactCycles = 0;
  lastBaseSample = 0;
  lastPreparedBaseSample = 0;
  lastExactSample = 0;
  haveBaseSample = false;
  havePreparedBaseSample = false;
  haveExactSample = false;
  lastWrongSuffix = 0;
  haveWrongSuffix = false;

  ADMUX = _BV(REFS0);  // AVcc reference, ADC0/A0.
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);  // 500 kHz ADC.
  ADCSRB = 0;
  DIDR0 |= _BV(ADC0D);
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {}
  ADCSRA |= _BV(ADIF);

  const uint8_t savedSreg = SREG;
  cli();
  savedTimer0Mask = TIMSK0;
  TIMSK0 &= ~_BV(TOIE0);
  TIMSK2 = 0;

  // Start Timer1 and Timer2 from the same prescaler boundary, matching the
  // working reader so every ADC conversion has a fixed carrier phase.
  GTCCR = _BV(TSM) | _BV(PSRSYNC) | _BV(PSRASY);
  TCCR1A = _BV(COM1A0);
  TCCR1B = _BV(WGM12) | _BV(CS10);
  TCNT1 = 0;
  OCR1A = 63;
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS21);
  TCNT2 = 0;
  OCR2A = 127;  // 16 MHz / 8 / 128 = 64 us.
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

VerifyResult finishCapture() {
  const uint8_t savedSreg = SREG;
  cli();
  TIMSK2 = 0;
  TCCR2B = 0;
  ADCSRA &= ~_BV(ADEN);
  TIMSK0 = savedTimer0Mask;
  SREG = savedSreg;

  VerifyResult result;
  result.adcMin = captureMin;
  result.adcMax = captureMax;
  result.missed = missedSamples;
  result.late = lateSamples;
  result.validBaseFrames = validBaseFrames;
  result.preparedBaseFrames = preparedBaseFrames;
  result.exactCycles = exactCycles;
  result.lastWrongSuffix = lastWrongSuffix;
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

void printVerifyResult(const __FlashStringHelper *label,
                       const VerifyResult &result) {
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
  Serial.print(result.validBaseFrames);
  Serial.print(F(" preparedBase="));
  Serial.print(result.preparedBaseFrames);
  Serial.print(F(" exact128="));
  Serial.print(result.exactCycles);
  if (result.haveWrongSuffix) {
    Serial.print(F(" lastWrongSuffix=0x"));
    printHex64(result.lastWrongSuffix);
  } else {
    Serial.print(F(" lastWrongSuffix=none"));
  }
  Serial.println();
}

bool captureHealthy(const VerifyResult &result) {
  return result.missed == 0 && result.late == 0;
}

VerifyResult runCapture(bool withSuffix,
                        const __FlashStringHelper *label) {
  startCapture(withSuffix);
  while (sampleCount < VERIFY_SAMPLES) {
    if (withSuffix) {
      if (exactCycles >= REQUIRED_OBSERVATIONS) break;
    } else if (preparedBaseFrames >= REQUIRED_OBSERVATIONS) {
      break;
    }
    processSample(readSynchronousSample());
  }
  VerifyResult result = finishCapture();
  printVerifyResult(label, result);
  return result;
}

bool verifyExact128() {
  const VerifyResult result = runCapture(true, F("VERIFY128"));
  const bool passed =
      captureHealthy(result) &&
      result.exactCycles >= REQUIRED_OBSERVATIONS;
  Serial.println(passed ? F("VERIFY PASS: exact prepared 128-bit cycle observed.")
                        : F("VERIFY FAIL: exact healthy 128-bit proof missing."));
  return passed;
}

bool verifyRecovery64() {
  const VerifyResult result = runCapture(false, F("VERIFY64"));
  return captureHealthy(result) &&
         result.preparedBaseFrames >= REQUIRED_OBSERVATIONS;
}

int8_t hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool parseId(const char *text, uint8_t id[5]) {
  for (uint8_t i = 0; i < 5; i++) {
    const int8_t high = hexValue(text[i * 2]);
    const int8_t low = hexValue(text[i * 2 + 1]);
    if (high < 0 || low < 0) return false;
    id[i] = (high << 4) | low;
  }
  return true;
}

bool parseSuffix(const char *text, uint64_t *suffix) {
  uint64_t value = 0;
  for (uint8_t i = 0; i < 16; i++) {
    const int8_t digit = hexValue(text[i]);
    if (digit < 0) return false;
    value = (value << 4) | (uint8_t)digit;
  }
  *suffix = value;
  return true;
}

void printPrepared() {
  Serial.print(F("Prepared ID="));
  printId(preparedId);
  Serial.print(F(" suffix=0x"));
  printHex64(preparedSuffix);
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(F(" B"));
    Serial.print(i + 1);
    Serial.print(F("=0x"));
    printHex32(preparedBlocks[i]);
  }
  Serial.println();
}

void prepareElectra(const char *arguments) {
  if (strlen(arguments) != 27 || arguments[10] != ' ') {
    Serial.println(F("Usage: ELECTRA <10_HEX_DIGITS> <16_HEX_DIGITS>"));
    return;
  }
  uint8_t id[5];
  uint64_t suffix;
  if (!parseId(arguments, id) || !parseSuffix(arguments + 11, &suffix)) {
    Serial.println(F("Invalid ELECTRA data: hexadecimal digits only."));
    return;
  }
  memcpy(preparedId, id, sizeof(preparedId));
  preparedFrame = encodeEm4100(preparedId);
  preparedSuffix = suffix;
  preparedBlocks[0] = (uint32_t)(preparedFrame >> 32);
  preparedBlocks[1] = (uint32_t)preparedFrame;
  preparedBlocks[2] = (uint32_t)(preparedSuffix >> 32);
  preparedBlocks[3] = (uint32_t)preparedSuffix;
  prepared = true;
  printPrepared();
}

void destructiveWarning(const __FlashStringHelper *action) {
  Serial.println(F("WARNING: DESTRUCTIVE WRITE TO AN UNLOCKED MULTI-BLOCK TAG."));
  Serial.println(F("Keep one authorized tag centered; do not reset or remove power."));
  Serial.print(action);
  Serial.println(F(" begins in 2 seconds."));
  Serial.flush();
  delay(2000);
}

bool recoverOnce() {
  writeFreshBlock(1, preparedBlocks[0]);
  writeFreshBlock(2, preparedBlocks[1]);
  writeFreshBlock(0, CONFIG_64);
  powerCycleField();
  return verifyRecovery64();
}

bool recoverPrepared() {
  recoveryRequired = true;
  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    Serial.print(F("Recovery attempt "));
    Serial.print(attempt);
    Serial.println(F("/3: B1, B2, then B0=0x00148040."));
    if (recoverOnce()) {
      recoveryRequired = false;
      Serial.println(F("RECOVERED: prepared ID verified in 3 distinct frames."));
      return true;
    }
  }
  Serial.println(F("RECOVERY FAILED: RAM lock and prepared data retained."));
  Serial.println(F("Only RECOVER and HELP are allowed. Keep power on."));
  return false;
}

void writePrepared() {
  if (!prepared) {
    Serial.println(F("Nothing prepared. Use ELECTRA first."));
    return;
  }
  destructiveWarning(F("WRITE"));

  recoveryRequired = true;  // Set before the first mutating downlink.
  Serial.println(F("Writing B1, B2, B3, B4, then B0 last..."));
  for (uint8_t block = 1; block <= 4; block++) {
    writeFreshBlock(block, preparedBlocks[block - 1]);
  }
  writeFreshBlock(0, CONFIG_128);
  powerCycleField();

  if (verifyExact128()) {
    recoveryRequired = false;
    Serial.println(F("WRITE SUCCESS: exact 128-bit data verified 3 times."));
    return;
  }

  Serial.println(F("WRITE VERIFY FAILED: automatically restoring standard EM4100."));
  recoverPrepared();
}

void verifyCommand() {
  if (!prepared) {
    Serial.println(F("Nothing prepared. Use ELECTRA first."));
    return;
  }
  Serial.println(F("VERIFY is receive-only: no downlink command and no EEPROM write."));
  verifyExact128();
}

void recoverCommand() {
  if (!prepared) {
    Serial.println(F("No prepared base ID is available for recovery."));
    return;
  }
  destructiveWarning(F("RECOVER"));
  recoverPrepared();
}

void cancelPrepared() {
  if (recoveryRequired) {
    Serial.println(F("CANCEL blocked: verified recovery is required."));
    return;
  }
  memset(preparedId, 0, sizeof(preparedId));
  preparedFrame = 0;
  preparedSuffix = 0;
  memset(preparedBlocks, 0, sizeof(preparedBlocks));
  prepared = false;
  Serial.println(F("Prepared data cleared."));
}

void showHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  ELECTRA <10_HEX_DIGITS> <16_HEX_DIGITS>  prepare ID+suffix"));
  Serial.println(F("  VERIFY   receive-only exact 128-bit verification"));
  Serial.println(F("  WRITE    write B1-B4, B0 last, then verify"));
  Serial.println(F("  RECOVER  restore prepared ID as standard 64-bit EM4100"));
  Serial.println(F("  CANCEL   clear preparation unless recovery is required"));
  Serial.println(F("  HELP     show commands"));
  Serial.println(F("Unlocked ATA5577/T5577 with real B1-B4 only; no password/locks."));
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

bool startsWithIgnoreCase(const char *text, const char *prefix) {
  while (*prefix) {
    char a = *text++;
    char b = *prefix++;
    if (a >= 'a' && a <= 'z') a -= 'a' - 'A';
    if (b >= 'a' && b <= 'z') b -= 'a' - 'A';
    if (a != b) return false;
  }
  return true;
}

void handleCommand(char *command) {
  while (*command == ' ') command++;
  char *end = command + strlen(command);
  while (end > command && end[-1] == ' ') *--end = '\0';
  if (*command == '\0') return;

  if (recoveryRequired &&
      !equalsIgnoreCase(command, "RECOVER") &&
      !equalsIgnoreCase(command, "HELP")) {
    Serial.println(F("Blocked by recovery lock: only RECOVER or HELP."));
    return;
  }

  if (startsWithIgnoreCase(command, "ELECTRA ")) {
    prepareElectra(command + 8);
  } else if (equalsIgnoreCase(command, "ELECTRA")) {
    Serial.println(F("Usage: ELECTRA <10_HEX_DIGITS> <16_HEX_DIGITS>"));
  } else if (equalsIgnoreCase(command, "VERIFY")) {
    verifyCommand();
  } else if (equalsIgnoreCase(command, "WRITE")) {
    writePrepared();
  } else if (equalsIgnoreCase(command, "RECOVER")) {
    recoverCommand();
  } else if (equalsIgnoreCase(command, "CANCEL")) {
    cancelPrepared();
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

  Serial.println(F("Guarded ATA5577 128-bit writer ready."));
  Serial.println(F("D9 carrier, A0 envelope, 115200 baud/Newline."));
  Serial.println(F("RAM recovery state and prepared data ARE LOST ON RESET."));
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
