/*
 * Minimal 64-bit EM4100 writer for Arduino Uno / ATmega328P.
 *
 * D9 generates the 125 kHz carrier and creates T5577/T5557 downlink gaps.
 * A0 reads the passive envelope detector for Manchester RF/64 verification.
 *
 * This sketch intentionally keeps password and lock bits clear. Use it only
 * with tags and systems you own or are authorized to test. Direct D9 tank
 * drive is experimental: verify the A0 voltage before connecting it.
 */

#include <Arduino.h>
#include <string.h>

const uint8_t PIN_CARRIER = 9;
const uint32_t EM4100_CONFIG = 0x00148040UL;
const uint8_t SEARCHING = 0xFF;

const uint16_t POWER_UP_CLOCKS = 400;
const uint8_t START_GAP_CLOCKS = 30;
const uint8_t WRITE_GAP_CLOCKS = 19;
const uint8_t ZERO_CLOCKS = 24;
const uint8_t ONE_CLOCKS = 54;
const uint16_t PROGRAM_HOLD_CLOCKS = 900;

const uint8_t VERIFY_THRESHOLD = 8;
const uint32_t VERIFY_SAMPLES = 46875UL; // Up to 3 seconds at 15.625 ksample/s.
const uint8_t REQUIRED_FRAMES = 3;

struct VerifyDecoder {
  uint16_t header;
  uint8_t pos;
  uint8_t rowXor;
  uint8_t colXor;
  uint8_t nibble;
  uint8_t id[5];
};

uint8_t preparedId[5];
uint32_t preparedBlock1 = 0;
uint32_t preparedBlock2 = 0;
bool prepared = false;
bool recoveryRequired = false;

VerifyDecoder verifyDecoders[16]; // Eight sample phases, both polarities.
uint16_t verifyRing[8];
uint16_t verifyFirstSum = 0;
uint16_t verifySecondSum = 0;
uint8_t verifyRingPos = 0;
uint8_t verifyRingFill = 0;
uint32_t verifySampleCount = 0;
uint32_t lastTargetSample = 0;
uint8_t targetFrames = 0;
bool haveTargetSample = false;
uint16_t verifyMin = 1023;
uint16_t verifyMax = 0;
volatile uint16_t verifyMissed = 0;
volatile uint16_t verifyLate = 0;
uint8_t savedTimer0Mask = 0;

char serialBuffer[24];
uint8_t serialLength = 0;
bool discardSerialLine = false;

ISR(TIMER2_COMPA_vect) {
  if (ADCSRA & _BV(ADSC)) {
    verifyMissed++;
  } else {
    if (ADCSRA & _BV(ADIF)) verifyLate++;
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
  // At 125 kHz each field clock is 8 us.
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
  sendDownlinkBit(0); // Page 0 write opcode.
  sendDownlinkBit(0); // Lock bit stays clear.

  for (int8_t bit = 31; bit >= 0; bit--) {
    sendDownlinkBit((data >> bit) & 1U);
  }
  sendDownlinkBit((block >> 2) & 1U);
  sendDownlinkBit((block >> 1) & 1U);
  sendDownlinkBit(block & 1U);
  waitFieldClocks(PROGRAM_HOLD_CLOCKS);
}

void writeFreshBlock(uint8_t block, uint32_t data) {
  carrierOff();
  delay(100);
  carrierOn();
  delay(20);

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

void resetDecoder(VerifyDecoder &decoder) {
  decoder.header = 0;
  decoder.pos = SEARCHING;
  decoder.rowXor = 0;
  decoder.colXor = 0;
  decoder.nibble = 0;
  memset(decoder.id, 0, sizeof(decoder.id));
}

void beginPayload(VerifyDecoder &decoder) {
  decoder.pos = 0;
  decoder.rowXor = 0;
  decoder.colXor = 0;
  decoder.nibble = 0;
  memset(decoder.id, 0, sizeof(decoder.id));
}

void rejectFrame(VerifyDecoder &decoder) {
  decoder.header = 0;
  decoder.pos = SEARCHING;
}

void noteValidFrame(const uint8_t id[5]) {
  if (memcmp(id, preparedId, 5) != 0) return;

  // Phase candidates can report one physical frame multiple times. RF/64
  // repeats are much farther apart than this de-duplication interval.
  if (!haveTargetSample ||
      verifySampleCount - lastTargetSample >= 256UL) {
    if (targetFrames < 255) targetFrames++;
    lastTargetSample = verifySampleCount;
    haveTargetSample = true;
  }
}

void feedVerifyBit(VerifyDecoder &decoder, uint8_t bit) {
  decoder.header = ((decoder.header << 1) | bit) & 0x03FF;
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
        rejectFrame(decoder);
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
      rejectFrame(decoder);
      return;
    }
  } else {
    if (bit == 0) noteValidFrame(decoder.id);
    rejectFrame(decoder);
    return;
  }
  decoder.pos++;
}

void processVerifyWindow(int16_t difference, uint8_t phase) {
  const uint16_t magnitude =
      difference < 0 ? (uint16_t)-difference : (uint16_t)difference;
  VerifyDecoder &normal = verifyDecoders[phase << 1];
  VerifyDecoder &inverse = verifyDecoders[(phase << 1) | 1];

  if (magnitude < VERIFY_THRESHOLD) {
    rejectFrame(normal);
    rejectFrame(inverse);
    return;
  }
  const uint8_t bit = difference > 0 ? 1 : 0;
  feedVerifyBit(normal, bit);
  feedVerifyBit(inverse, bit ^ 1U);
}

void processVerifySample(uint16_t sample) {
  verifySampleCount++;
  if (sample < verifyMin) verifyMin = sample;
  if (sample > verifyMax) verifyMax = sample;

  if (verifyRingFill < 8) {
    verifyRing[verifyRingPos] = sample;
    verifyRingPos = (verifyRingPos + 1) & 0x07;
    verifyRingFill++;
    if (verifyRingFill != 8) return;

    verifyFirstSum = 0;
    verifySecondSum = 0;
    for (uint8_t i = 0; i < 4; i++) verifyFirstSum += verifyRing[i];
    for (uint8_t i = 4; i < 8; i++) verifySecondSum += verifyRing[i];
  } else {
    const uint8_t middle = (verifyRingPos + 4) & 0x07;
    const uint16_t oldFirst = verifyRing[verifyRingPos];
    const uint16_t oldSecond = verifyRing[middle];
    verifyFirstSum = verifyFirstSum - oldFirst + oldSecond;
    verifySecondSum = verifySecondSum - oldSecond + sample;
    verifyRing[verifyRingPos] = sample;
    verifyRingPos = (verifyRingPos + 1) & 0x07;
  }

  processVerifyWindow(
      (int16_t)verifySecondSum - (int16_t)verifyFirstSum,
      verifySampleCount & 0x07);
}

void startVerification() {
  for (uint8_t i = 0; i < 16; i++) resetDecoder(verifyDecoders[i]);
  verifyRingPos = 0;
  verifyRingFill = 0;
  verifyFirstSum = 0;
  verifySecondSum = 0;
  verifySampleCount = 0;
  lastTargetSample = 0;
  targetFrames = 0;
  haveTargetSample = false;
  verifyMin = 1023;
  verifyMax = 0;
  verifyMissed = 0;
  verifyLate = 0;

  ADMUX = _BV(REFS0);
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);
  ADCSRB = 0;
  DIDR0 |= _BV(ADC0D);
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {}
  ADCSRA |= _BV(ADIF);

  const uint8_t savedSreg = SREG;
  cli();
  savedTimer0Mask = TIMSK0;
  TIMSK0 &= ~_BV(TOIE0);
  GTCCR = _BV(TSM) | _BV(PSRSYNC) | _BV(PSRASY);
  TCCR1A = _BV(COM1A0);
  TCCR1B = _BV(WGM12) | _BV(CS10);
  TCNT1 = 0;
  OCR1A = 63;
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS21);
  TCNT2 = 0;
  OCR2A = 127;
  TIFR2 = _BV(OCF2A);
  TIMSK2 = _BV(OCIE2A);
  GTCCR = 0;
  SREG = savedSreg;
}

uint16_t readVerificationSample() {
  while (!(ADCSRA & _BV(ADIF))) {}
  const uint16_t sample = ADC;
  ADCSRA |= _BV(ADIF);
  return sample;
}

bool verifyPreparedId() {
  startVerification();
  while (verifySampleCount < VERIFY_SAMPLES &&
         targetFrames < REQUIRED_FRAMES) {
    processVerifySample(readVerificationSample());
  }
  TIMSK2 &= ~_BV(OCIE2A);
  while (ADCSRA & _BV(ADSC)) {}
  ADCSRA |= _BV(ADIF);
  TIMSK0 = savedTimer0Mask;

  Serial.print(F("Verify: distinct target frames="));
  Serial.print(targetFrames);
  Serial.print(F(" ADC="));
  Serial.print(verifyMin);
  Serial.print('-');
  Serial.print(verifyMax);
  Serial.print(F(" missed="));
  Serial.print(verifyMissed);
  Serial.print(F(" late="));
  Serial.println(verifyLate);
  return targetFrames >= REQUIRED_FRAMES;
}

void printHex32(uint32_t value) {
  for (int8_t nibble = 7; nibble >= 0; nibble--) {
    Serial.print((uint8_t)((value >> (nibble * 4)) & 0x0F), HEX);
  }
}

void printPrepared() {
  Serial.print(F("Prepared ID: "));
  for (uint8_t i = 0; i < 5; i++) {
    if (preparedId[i] < 16) Serial.print('0');
    Serial.print(preparedId[i], HEX);
  }
  Serial.print(F("  B1=0x"));
  printHex32(preparedBlock1);
  Serial.print(F(" B2=0x"));
  printHex32(preparedBlock2);
  Serial.print(F(" B0=0x"));
  printHex32(EM4100_CONFIG);
  Serial.println();
}

int8_t hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
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
  if (recoveryRequired) {
    Serial.println(F("ID blocked: verified recovery is required."));
    return;
  }
  uint8_t id[5];
  if (!parseId(text, id)) {
    Serial.println(F("Invalid ID. Use exactly 10 hexadecimal characters."));
    return;
  }
  memcpy(preparedId, id, sizeof(preparedId));
  const uint64_t frame = encodeEm4100(preparedId);
  preparedBlock1 = (uint32_t)(frame >> 32);
  preparedBlock2 = (uint32_t)frame;
  prepared = true;
  printPrepared();
}

bool writeAndVerify() {
  // Set this before the first EEPROM command so this running session requires
  // a complete B1/B2/B0 rewrite after an unverified mutation.
  recoveryRequired = true;
  writeFreshBlock(1, preparedBlock1);
  writeFreshBlock(2, preparedBlock2);
  writeFreshBlock(0, EM4100_CONFIG);

  carrierOff();
  delay(100);
  carrierOn();
  delay(20);
  return verifyPreparedId();
}

bool recoverPrepared(uint8_t attempts) {
  for (uint8_t attempt = 0; attempt < attempts; attempt++) {
    Serial.print(F("Recovery attempt "));
    Serial.print(attempt + 1);
    Serial.print('/');
    Serial.println(attempts);
    if (writeAndVerify()) {
      recoveryRequired = false;
      prepared = false;
      Serial.println(F("RECOVERED: requested 64-bit EM4100 frame verified."));
      return true;
    }
  }
  Serial.println(F("RECOVERY FAILED: prepared data retained; keep the tag still"));
  Serial.println(F("and use RECOVER. Best-effort recovery cannot guarantee repair."));
  return false;
}

void writePrepared() {
  if (!prepared) {
    Serial.println(F("Nothing prepared. Use ID <10_HEX_DIGITS>."));
    return;
  }
  if (recoveryRequired) {
    Serial.println(F("WRITE blocked: use RECOVER."));
    return;
  }

  Serial.println(F("WARNING: writing is destructive. Power loss or tag movement"));
  Serial.println(F("can leave the tag unreadable. Keep only the target tag nearby."));
  Serial.println(F("Writing B1, B2, then B0 in 2 seconds."));
  Serial.flush();
  delay(2000);

  if (writeAndVerify()) {
    recoveryRequired = false;
    prepared = false;
    Serial.println(F("SUCCESS: requested ID verified in 3 distinct frames."));
    return;
  }

  Serial.println(F("WRITE VERIFY FAILED: attempting verified recovery."));
  recoverPrepared(2);
}

void recoverCommand() {
  if (!prepared) {
    Serial.println(F("No prepared recovery data is available."));
    return;
  }
  Serial.println(F("Best-effort recovery rewrites B1, B2, and B0."));
  recoverPrepared(3);
}

void cancelPrepared() {
  if (recoveryRequired) {
    Serial.println(F("CANCEL blocked: verified recovery is required."));
    return;
  }
  prepared = false;
  memset(preparedId, 0, sizeof(preparedId));
  preparedBlock1 = 0;
  preparedBlock2 = 0;
  Serial.println(F("Prepared data cleared."));
}

void showHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  ID <10_HEX_DIGITS>  prepare a 40-bit ID"));
  Serial.println(F("  WRITE               write B1, B2, B0 and verify"));
  Serial.println(F("  RECOVER             best-effort rewrite and verification"));
  Serial.println(F("  CANCEL              clear data unless recovery is required"));
  Serial.println(F("  HELP                show commands"));
  Serial.println(F("No password or lock bits are set. Authorized tags only."));
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

  if ((command[0] == 'I' || command[0] == 'i') &&
      (command[1] == 'D' || command[1] == 'd') &&
      command[2] == ' ') {
    prepareId(command + 3);
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

  Serial.println(F("Experimental 64-bit EM4100 writer ready."));
  Serial.println(F("D9 carrier; A0 envelope. Verify A0 voltage before connection."));
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
