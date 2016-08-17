// MIT License
//
// Copyright (c) 2016 Joonas Javanainen <joonas.javanainen@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <string.h>

#include "nelmax.h"

static struct NelmaX NELMAX = {0};
static bool pgdOutput = false;

const int PGD = 5;
const int PGC = 6;
const int MCLR = 7;

enum PicRegister: byte {
  TBLPTRU = 0xf8,
  TBLPTRH = 0xf7,
  TBLPTRL = 0xf6,
  EEADR   = 0xa9,
  EEDATA  = 0xa8,
  EECON1  = 0xa6
};

inline void delayP2A() { delayMicroseconds(1); } // P2A: 40-400 ns
inline void delayP2B() { delayMicroseconds(1); } // P2B: 40-400 ns
inline void delayP5() { delayMicroseconds(1); } // P5: 40 ns
inline void delayP5A() { delayMicroseconds(1); } // P5A: 40 ns
inline void delayP6() { delayMicroseconds(1); } // P6: 20 ns
inline void delayP9() { delay(1); } // P9: 1 ms
inline void delayP9A() { delay(5); } // P9A: 5 ms
inline void delayP10() { delayMicroseconds(200); } // P10: 200 μs
inline void delayP11() { delay(15); } // P11: 12-15 ms
inline void delayP13() { delayMicroseconds(1); } // P13: 100 ns
inline void delayP15() { delayMicroseconds(400); } // P15: 400 μs
inline void delayP16() { } // P16: 0 s
inline void delayP18() { delay(1); } // P18: 1 ms
inline void delayP19() { delayMicroseconds(1); } // P19: 3-10 ns
inline void delayP20() { delayMicroseconds(1); } // P20: 40 ns

void setup() {
  pinMode(PGC, INPUT);
  pinMode(PGD, INPUT);
  pinMode(MCLR, OUTPUT);
  digitalWrite(PGC, LOW);
  digitalWrite(PGD, LOW);
  digitalWrite(MCLR, LOW);

  Serial.begin(115200);
}

void cmdPing(int payloadSize) {
  byte handshake[8];
  memcpy(handshake, nelmax_payload(&NELMAX), payloadSize);
  nelmax_write_array(&NELMAX, handshake, payloadSize);
}

void cmdRead() {
  byte len = nelmax_payload(&NELMAX)[0];
  byte usb = nelmax_payload(&NELMAX)[1];
  byte msb = nelmax_payload(&NELMAX)[2];
  byte lsb = nelmax_payload(&NELMAX)[3];

  coreLoadTBLPTR(usb, msb, lsb);
  for (int i = 0; i < len; i++) {
    byte value = tableReadPostIncr();
    nelmax_write(&NELMAX, value);
  }
}

void cmdWriteConfiguration(int payloadSize) {
  byte addrLsb = nelmax_payload(&NELMAX)[0];
  byte len = payloadSize - 1;

  coreAccessProgramMemory();
  coreEnableConfigAccess();
  coreEnableWrites();

  coreLoadRegister(TBLPTRU, 0x30);
  coreLoadRegister(TBLPTRH, 0x00);

  for (int i = 0; i < len; i++) {
    coreLoadRegister(TBLPTRL, addrLsb + i);
    tableWriteProgramConfigId(nelmax_payload(&NELMAX)[1 + i], nelmax_payload(&NELMAX)[1 + i]);
  }

  coreDisableWrites();
}

bool dispatchCommand(byte command, int payloadSize) {
  switch (command) {
    case 'P':
      cmdPing(payloadSize);
      return true;
    case 'S':
      cmdStart();
      return true;
    case 'F':
      cmdFinish();
      return true;
    case 'X':
      if (payloadSize == 2) {
        cmdBulkErase();
        return true;
      }
      break;
    case 'W':
      if (payloadSize == 2 + 64) {
        cmdWriteRow();
        return true;
      }
      break;
    case 'C':
      if (payloadSize >= 1) {
        cmdWriteConfiguration(payloadSize);
        return true;
      }
      break;
    case 'R':
      if (payloadSize == 1 + 3) {
        cmdRead();
        return true;
      }
      break;
    case 'D':
      if (payloadSize == 1 + 1) {
        cmdReadDataEeprom();
        return true;
      }
      break;
    case 'M':
      if (payloadSize >= 1) {
        cmdWriteDataEeprom(payloadSize);
        return true;
      }
      break;
  }
  return false;
}

void cmdBulkErase() {
  byte data_h = nelmax_payload(&NELMAX)[0];
  byte data_l = nelmax_payload(&NELMAX)[1];

  coreLoadTBLPTR(0x3c, 0x00, 0x05);
  tableWrite(data_h, data_h);
  coreLoadTBLPTR(0x3c, 0x00, 0x04);
  tableWrite(data_l, data_l);

  coreNop();
  pgWrite4(B0000);
  delayP11();
  delayP10();
  pgWrite16(0x0000);
  delayP5A();
}

void cmdReadDataEeprom() {
  byte len = nelmax_payload(&NELMAX)[0];
  byte addr = nelmax_payload(&NELMAX)[1];

  coreAccessDataMemory();
  coreDisableConfigAccess();

  for (int i = 0; i < len; i++) {
    coreLoadRegister(EEADR, addr + i);
    coreInitiateEepromRead();

    coreLoadTablat(EEDATA);
    coreNop();

    byte value = shiftTablatRegister();
    nelmax_write(&NELMAX, value);
  }
}

void cmdWriteDataEeprom(int payloadSize) {
  byte addr = nelmax_payload(&NELMAX)[0];
  int len = payloadSize - 1;

  coreAccessDataMemory();
  coreDisableConfigAccess();

  for (int i = 0; i < len; i++) {
    byte data = nelmax_payload(&NELMAX)[1 + i];

    coreLoadRegister(EEADR, addr + i);
    coreLoadRegister(EEDATA, data);
    coreEnableWrites();
    coreInitiateEepromWrite();
    coreNop();
    coreNop();

    byte eecon1;
    do {
      coreLoadTablat(EECON1);
      coreNop();

      eecon1 = shiftTablatRegister();
    } while(eecon1 & B00000010);

    delayP10();
    coreDisableWrites();
  }
}

void cmdWriteRow() {
  byte msb = nelmax_payload(&NELMAX)[0];
  byte lsb = nelmax_payload(&NELMAX)[1];

  coreAccessProgramMemory();
  coreDisableConfigAccess();
  coreEnableWrites();

  coreLoadTBLPTR(0x00, msb, lsb);

  int pos;
  for (pos = 0; pos < 62; pos += 2) {
    tableWriteIncr2(nelmax_payload(&NELMAX)[2 + pos], nelmax_payload(&NELMAX)[2 + pos + 1]);
  }
  tableWriteProgram(nelmax_payload(&NELMAX)[2 + pos], nelmax_payload(&NELMAX)[2 + pos + 1]);

  coreDisableWrites();
}

void cmdStart() {
  digitalWrite(MCLR, LOW);
  delayP13();
  digitalWrite(MCLR, HIGH);
  digitalWrite(MCLR, LOW);
  delayP18();
  pinMode(PGC, OUTPUT);
  pinMode(PGD, OUTPUT);
  pgdOutput = true;
  pgWriteKeySequence(0x4D434850);
  delayP20();
  digitalWrite(MCLR, HIGH);
  delayP15();
}

void cmdFinish() {
  digitalWrite(PGD, LOW);
  digitalWrite(PGC, LOW);
  delayP16();
  pinMode(PGC, INPUT);
  pinMode(PGD, INPUT);
  pgdOutput = false;
  digitalWrite(MCLR, LOW);
}

inline void pulsePGC() {
  digitalWrite(PGC, HIGH);
  delayP2B();
  digitalWrite(PGC, LOW);
  delayP2A();
}

inline void pgWrite1(bool data) {
  digitalWrite(PGD, data);
  pulsePGC();
}

inline void pgWrite16(word data) {
  for (int i = 0; i < 16; i++) {
    pgWrite1((1 << i) & data);
  }
}

inline void pgWrite8(byte data) {
  for (int i = 0; i < 8; i++) {
    pgWrite1((1 << i) & data);
  }
}

inline void pgWrite4(byte data) {
  if (pgdOutput) {
    for (int i = 0; i < 4; i++) {
      pgWrite1((1 << i) & data);
    }
  } else {
    // Handle high-impedance delay
    digitalWrite(PGC, HIGH);
    delayP19();
    pinMode(PGD, OUTPUT);
    pgdOutput = true;
    digitalWrite(PGD, data & 1);
    delayP2B();
    digitalWrite(PGC, LOW);
    delayP2A();

    for (int i = 1; i < 4; i++) {
      pgWrite1((1 << i) & data);
    }
  }
}

inline void pgWriteKeySequence(uint32_t data) {
  for (int i = 31; i >= 0; i--) {
    pgWrite1((1 << i) & data);
  }
}

void coreInstr(word data) {
  pgWrite4(B0000);
  delayP5();
  pgWrite16(data);
  delayP5A();
}

inline void coreLoadRegister(PicRegister reg, byte data) {
  coreInstr(0x0e00 | data); // MOVLW data
  coreInstr(0x6e00 | reg);  // MOVWF reg
}
inline void coreLoadTBLPTR(byte usb, byte msb, byte lsb) {
  coreLoadRegister(TBLPTRU, usb);
  coreLoadRegister(TBLPTRH, msb);
  coreLoadRegister(TBLPTRL, lsb);
}
inline void coreLoadTablat(PicRegister reg) {
  coreInstr(0x5000 | reg); // MOVF EECON1, W, 0
  coreInstr(0x6ef5);       // MOVWF TABLAT
}
inline void coreEnableWrites() {
  coreInstr(0x84a6); // BSF EECON1, WREN
}
inline void coreDisableWrites() {
  coreInstr(0x94a6); // BCF EECON1, WREN
}
inline void coreAccessProgramMemory() {
  coreInstr(0x8ea6); // BSF EECON1, EEPGD
}
inline void coreAccessDataMemory() {
  coreInstr(0x9ea6); // BCF EECON1, EEPGD
}
inline void coreEnableConfigAccess() {
  coreInstr(0x8ca6); // BSF EECON1, CFGS
}
inline void coreDisableConfigAccess() {
  coreInstr(0x9ca6); // BCF EECON1, CFGS
}
inline void coreNop() {
  coreInstr(0x0000); // NOP
}
inline void coreInitiateEepromRead() {
  coreInstr(0x80a6); // BSF EECON1, RD
}
inline void coreInitiateEepromWrite() {
  coreInstr(0x82a6); // BSF EECON1, WR
}

byte execReadCommand(byte command) {
  pgWrite4(command);
  delayP5();

  digitalWrite(PGD, LOW);
  for (int i = 0; i < 8; i++) {
    pulsePGC();
  }
  pinMode(PGD, INPUT);
  pgdOutput = false;
  delayP6();

  byte value = 0;
  for (int i = 0; i < 8; i++) {
    pulsePGC();
    if (digitalRead(PGD)) {
      value |= 1 << i;
    }
  }
  delayP5A();

  // PGD cannot be set to output here because
  // the PIC chip sets PGD to hi-Z mode only after
  // the first high PGC during the next command
  return value;
}

byte shiftTablatRegister() {
  return execReadCommand(B0010);
}

byte tableRead() {
  return execReadCommand(B1000);
}

byte tableReadPostIncr() {
  return execReadCommand(B1001);
}

byte tableReadPostDecr() {
  return execReadCommand(B1010);
}

byte tableReadPreIncr() {
  return execReadCommand(B1011);
}

void execWriteCommand(byte command, byte msb, byte lsb) {
  pgWrite4(command);
  delayP5();
  pgWrite8(msb);
  pgWrite8(lsb);
  delayP5A();
}

void tableWrite(byte msb, byte lsb) {
  execWriteCommand(B1100, msb, lsb);
}

void tableWriteIncr2(byte msb, byte lsb) {
  execWriteCommand(B1101, msb, lsb);
}

void tableWriteProgram(byte msb, byte lsb) {
  execWriteCommand(B1111, msb, lsb);

  digitalWrite(PGD, LOW);
  pulsePGC();
  pulsePGC();
  pulsePGC();
  digitalWrite(PGC, HIGH);
  delayP9();
  digitalWrite(PGC, LOW);
  delayP10();
  pgWrite16(0x0000);
}

void tableWriteProgramConfigId(byte msb, byte lsb) {
  execWriteCommand(B1111, msb, lsb);

  digitalWrite(PGD, LOW);
  pulsePGC();
  pulsePGC();
  pulsePGC();
  digitalWrite(PGC, HIGH);
  delayP9A();
  digitalWrite(PGC, LOW);
  delayP10();
  pgWrite16(0x0000);
}

void loop() {
  byte command;
  size_t payloadSize;

  int maxBytes = Serial.available();
  while (maxBytes--) {
    byte inputByte = Serial.read();
    if (nelmax_read(&NELMAX, inputByte, &command, &payloadSize)) {
      if (dispatchCommand(command, payloadSize)) {
        size_t responseSize = nelmax_encode_response(&NELMAX);
        Serial.write(nelmax_encoded_packet(&NELMAX), responseSize);
      }
    }
  }
}
