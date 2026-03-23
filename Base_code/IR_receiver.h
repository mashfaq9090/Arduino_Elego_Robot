#pragma once
#include <Arduino.h>
 
// ===================== CONFIG =====================
#define IR_PIN 9
 
// ===================== KEY CODES =====================
#define IR_UP        0xB946FF00  
#define IR_DOWN      0xEA15FF00  
#define IR_LEFT      0xBB44FF00  
#define IR_RIGHT     0xBC43FF00  
#define IR_STOP      0xBF40FF00  
#define IR_Sound_1      0xE916FF00  // KEY_1
#define IR_Sound_2      0xE619FF00  // KEY_2
#define KEY_3      0xF20DFF00  // KEY_3
#define KEY_4      0xF30CFF00  // KEY_4
#define KEY_5      0xE718FF00  // KEY_5
#define KEY_6      0xA15EFF00  // KEY_6
#define KEY_7      0xF708FF00  // KEY_7
#define KEY_8      0xE31CFF00  // KEY_8
#define KEY_9      0xA55AFF00  // KEY_9
#define KEY_0      0xAD52FF00  // KEY_0
#define KEY_star      0xBD42FF00  // KEY_*
#define KEY_hash      0xB54AFF00  // KEY_#
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


 