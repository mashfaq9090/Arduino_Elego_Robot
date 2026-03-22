#pragma once
#include <Arduino.h>
 
// ===================== CONFIG =====================
#define IR_PIN 9
 
// ===================== KEY CODES =====================
// These are placeholder values — replace with YOUR remote's actual codes
// Run the test sketch first and map each button you want to use
#define IR_UP        0xB946FF00  // ← replace with your UP arrow code
#define IR_DOWN      0xEA15FF00  // ← replace with your DOWN arrow code
#define IR_LEFT      0xBB44FF00  // ← replace with your LEFT / KEY_4 code
#define IR_RIGHT     0xBC43FF00  // ← replace with your RIGHT / KEY_6 code
#define IR_STOP      0xBF40FF00  // ← replace with your STOP / OK
#define IR_NONE      0x00000000  // returned when no signal
 
// ===================== API =====================
inline void ir_init() {
  IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);
  Serial.println("[IR] Ready on pin " + String(IR_PIN));
}

inline unsigned long ir_read() {
  if (!IrReceiver.decode()) return IR_NONE;
  unsigned long code = IrReceiver.decodedIRData.decodedRawData;
  Serial.print("[IR] 0x");
  Serial.println(code, HEX);
  IrReceiver.resume();
  return code;
}


 