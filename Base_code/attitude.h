#pragma once 

#include "pin_def.h"

//extern float gyroZOffset; 
// extern float gyroXOffset  = 0;
// extern float gyroYOffset  = 0;
// extern float gyroZ_lpf    = 0;
// extern const float LPF_ALPHA = 0.1;       

// ===================== ATTITUDE DATA STRUCTS =====================

// Full angular displacement (degrees) from gyro integration
struct GyroAttitude {
  float x;  // roll  — rotation around X axis
  float y;  // pitch — rotation around Y axis
  float z;  // yaw   — rotation around Z axis (already had this)
  float k_z;
};

// Raw acceleration in m/s² (gravity removed on Z)
struct AccelData {
  float x;  // forward/backward acceleration
  float y;  // left/right acceleration
  float z;  // up/down acceleration (0 = level, gravity removed)
};

// Dead-reckoning position estimate (meters) — drifts over time
struct PositionEstimate {
  float velocity_x;   // current velocity m/s (intermediate, needed for integration)
  float velocity_y;
  float distance_x;   // total displacement on X axis in meters
  float distance_y;   // total displacement on Y axis in meters
  float total_dist;   // scalar total distance traveled in meters
};

// Single struct that holds all attitude data — pass this around
struct FullAttitude {
  GyroAttitude    angle;     // XYZ angular displacement in degrees
  AccelData       accel;     // XYZ acceleration in m/s²
  PositionEstimate position; // dead-reckoning position estimate
};

// Declare the global attitude object — accessible from main .ino
extern FullAttitude attitude;

// Function declarations
void resetAttitude();
void updateAttitude();
void attitude_debug();