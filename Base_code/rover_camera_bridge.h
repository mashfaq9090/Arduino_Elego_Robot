/*
 * rover_camera_bridge.h
 * Arduino side of ESP32 UART bridge
 *
 * UART Protocol:
 *   Send TO ESP32:
 *     'P' — take picture
 *     'V' — start stream
 *     'X' — stop stream
 *
 *   Receive FROM ESP32 (Web UI drive commands):
 *     'F' — forward
 *     'B' — backward
 *     'L' — left
 *     'R' — right
 *     'S' — stop
 *     'T' — trigger sonar sweep
 *
 * Uses SoftwareSerial on pins 10(RX) 11(TX)
 * Hardware Serial0 is reserved for USB debug on Uno
 */

#pragma once
#include <SoftwareSerial.h>

// Pins — confirm against your V4 wiring
#define CAM_RX_PIN 10
#define CAM_TX_PIN 11
#define CAM_BAUD   115200

SoftwareSerial CamSerial(CAM_RX_PIN, CAM_TX_PIN);

// ---- Commands TO ESP32 ----

void cam_take_picture() {
  CamSerial.write('P\n');
  Serial.println("[CAM] Take picture");
}

void cam_start_stream() {
  CamSerial.write('V\n');
  Serial.println("[CAM] Stream started");
}

void cam_stop_stream() {
  CamSerial.write('X\n');
  Serial.println("[CAM] Stream stopped");
}

// ---- Read commands FROM ESP32 ----
// Call this every loop tick
// Returns the command char or 0 if nothing available

char cam_read_command() {
  if (CamSerial.available()) {
    char cmd = CamSerial.read();
    Serial.print("[CAM] Received: "); Serial.println(cmd);
    return cmd;
  }
  return 0;
}

// ---- Initialize ----

void cam_init() {
  CamSerial.begin(CAM_BAUD);
  Serial.println("[CAM] UART bridge ready");
}