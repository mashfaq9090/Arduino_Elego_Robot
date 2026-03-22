#pragma once 

#include "attitude.h"
#include <Wire.h>
#include <Arduino.h>

// ===================== ATTITUDE SYSTEM =====================

FullAttitude attitude;  // global — accessible everywhere

// Sensitivity constants for MPU-6050 at default config
const float GYRO_SENSITIVITY  = 131.0;  // LSB/(deg/s) at ±250 deg/s
const float ACCEL_SENSITIVITY = 16384.0; // LSB/g at ±2g
const float G_TO_MS2          = 9.81;   // convert g to m/s²
float gyroXOffset  = 0;
float gyroYOffset  = 0;
float gyroZOffset = 0;
float gyroZ_lpf    = 0;
const float LPF_ALPHA = 0.1;  

// Internal timing
static unsigned long attitude_last_time = 0;

// Reset everything to zero — call after gyro calibration or before a new run
void resetAttitude() {
  attitude.angle.x    = 0;
  attitude.angle.y    = 0;
  attitude.angle.z    = 0;
  attitude.angle.k_z    = 0;

  attitude.accel.x    = 0;
  attitude.accel.y    = 0;
  attitude.accel.z    = 0;

  attitude.position.velocity_x = 0;
  attitude.position.velocity_y = 0;
  attitude.position.distance_x = 0;
  attitude.position.distance_y = 0;
  attitude.position.total_dist = 0;

  attitude_last_time = millis();
}

/*
 * updateAttitude():
 *   Reads all 6 axes from MPU-6050 in one I2C burst (efficient)
 *   Integrates gyro → angular displacement (X, Y, Z degrees)
 *   Converts accel → m/s²  with gravity removed on Z
 *   Double integrates accel → velocity → position (drifts over time)
 *
 *   CALL THIS EVERY LOOP TICK — accuracy degrades if called infrequently
 */
void updateAttitude() {
  unsigned long now = millis();
  float dt = (now - attitude_last_time) / 1000.0;  // seconds since last call
  attitude_last_time = now;

  // Guard against bad dt (first call or very long gap)
  if (dt <= 0 || dt > 0.5) return;

  // ---- READ ALL 6 AXES IN ONE BURST (registers 0x3B to 0x48) ----
  Wire.beginTransmission(GYRO);
  Wire.write(0x3B);  // starting register — ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(GYRO, 14, true);  // 14 bytes: accel(6) + temp(2) + gyro(6)

  // Accelerometer raw (registers 0x3B-0x40)
  int16_t raw_ax = Wire.read() << 8 | Wire.read();
  int16_t raw_ay = Wire.read() << 8 | Wire.read();
  int16_t raw_az = Wire.read() << 8 | Wire.read();

  // Temperature (registers 0x41-0x42) — discard
  Wire.read(); Wire.read();
//--------------------------------------------------------------------------------------------------------------------------

  // Gyroscope raw (registers 0x43-0x48)
  int16_t raw_gx = Wire.read() << 8 | Wire.read();
  int16_t raw_gy = Wire.read() << 8 | Wire.read();
  int16_t raw_gz = Wire.read() << 8 | Wire.read();

  // ---- GYRO → ANGULAR DISPLACEMENT ----
  // // Subtract calibration offset on Z (already have it), X and Y assumed ~0
  float gyro_x_dps = (raw_gx) / GYRO_SENSITIVITY;          // deg/s
  float gyro_y_dps = (raw_gy) / GYRO_SENSITIVITY;
  float gyro_z_dps = -(raw_gz - gyroZOffset) / GYRO_SENSITIVITY;  // reuse existing offset

  attitude.angle.x += gyro_x_dps * dt;  // integrate to degrees
  attitude.angle.y += gyro_y_dps * dt;
  attitude.angle.z += gyro_z_dps * dt;  // matches your existing currentAngle

  // Keep angles in -180 to +180 range
  if (attitude.angle.z >  180) attitude.angle.z -= 360;
  if (attitude.angle.z < -180) attitude.angle.z += 360;

  // ---- GYRO → ANGULAR DISPLACEMENT ----
  // Apply calibration offsets on all 3 axes
  // float gyro_x_dps =  (raw_gx - gyroXOffset) / GYRO_SENSITIVITY;
  // float gyro_y_dps =  (raw_gy - gyroYOffset) / GYRO_SENSITIVITY;
  // float gyro_z_raw = -(raw_gz - gyroZOffset) / GYRO_SENSITIVITY;

  // // Low pass filter on Z — kills noise, keeps real rotation signal
  // // This is what lets you drop the stuck threshold below 0.2
  // gyroZ_lpf = LPF_ALPHA * gyro_z_raw + (1.0 - LPF_ALPHA) * gyroZ_lpf;

  // attitude.angle.x += gyro_x_dps * dt;
  // attitude.angle.y += gyro_y_dps * dt;
  // attitude.angle.z += gyroZ_lpf  * dt;  // ← uses filtered rate

  // // Keep Z in -180 to +180
  // if (attitude.angle.z >  180) attitude.angle.z -= 360;
  // if (attitude.angle.z < -180) attitude.angle.z += 360;

//--------------------------------------------------------------------------------------------------------------------------
  // ---- ACCEL → m/s² (gravity removed on Z) ----
  attitude.accel.x =  (raw_ax / ACCEL_SENSITIVITY) * G_TO_MS2;
  attitude.accel.y =  (raw_ay / ACCEL_SENSITIVITY) * G_TO_MS2;
  attitude.accel.z = ((raw_az / ACCEL_SENSITIVITY) - 1.0) * G_TO_MS2; // -1g removes gravity

  // ---- ACCEL → VELOCITY → POSITION (dead reckoning) ----
  // NOTE: This drifts. Even 0.1 m/s² noise accumulates fast.
  // Reliable for short moves (~2-3 seconds), not long runs.
  // Zero the position with resetAttitude() before each measured move.

  // Apply a deadband — ignore accel noise below 0.2 m/s²
  float ax_filtered = (abs(attitude.accel.x) > 0.2) ? attitude.accel.x : 0;
  float ay_filtered = (abs(attitude.accel.y) > 0.2) ? attitude.accel.y : 0;

  // Integrate accel → velocity
  attitude.position.velocity_x += ax_filtered * dt;
  attitude.position.velocity_y += ay_filtered * dt;

  // Integrate velocity → position (meters)
  attitude.position.distance_x += attitude.position.velocity_x * dt * 100.0;  // ← multiply by 100 to convert meters to cm;
  attitude.position.distance_y += attitude.position.velocity_y * dt* 100.0;  // ← multiply by 100 to convert meters to cm;

  // Scalar total distance
  attitude.position.total_dist = sqrt(
    attitude.position.distance_x * attitude.position.distance_x +
    attitude.position.distance_y * attitude.position.distance_y
  )* 100.0;  // ← multiply by 100 to convert meters to cm;
}

// Full attitude printout — call in loop() for live monitoring
void attitude_debug() {
  Serial.println("========== ATTITUDE ==========");
  Serial.print("Angle  X: "); Serial.print(attitude.angle.x, 2);
  Serial.print("°  Y: ");     Serial.print(attitude.angle.y, 2);
  Serial.print("°  Z: ");     Serial.println(attitude.angle.k_z, 3);

  Serial.print("Accel  X: "); Serial.print(attitude.accel.x, 3);
  Serial.print(" Y: ");       Serial.print(attitude.accel.y, 3);
  Serial.print(" Z: ");       Serial.println(attitude.accel.z, 3);

  Serial.print("Vel    X: "); Serial.print(attitude.position.velocity_x, 3);
  Serial.print(" Y: ");       Serial.println(attitude.position.velocity_y, 3);

  Serial.print("Dist   X: "); Serial.print(attitude.position.distance_x, 3);
  Serial.print("cm  Y: ");     Serial.print(attitude.position.distance_y, 3);
  Serial.print("cm  Total: "); Serial.print(attitude.position.total_dist, 3);
  Serial.println("cm");
  Serial.println("==============================");
}