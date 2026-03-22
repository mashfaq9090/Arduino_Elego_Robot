#pragma once

#include <FastLED.h>
#include <Servo.h>

// ====== PROGRAM VARIABLES ======
// int16_t gyroZ;                // Raw gyro Z-axis reading
// float gyroZOffset = 0;        // Calibration offset
// float currentAngle = 0;       // Current angle in degrees
// unsigned long lastTime = 0;   // Last read time
// CRGB leds[NUM_LEDS];          // Current LED Color values
// Servo scanServo;              // Servo

extern int16_t gyroZ;                
extern float gyroZOffset;        
extern float currentAngle;       
extern unsigned long lastTime;   
extern CRGB leds[NUM_LEDS];          
extern Servo scanServo;
extern bool gyro_avail;

void ledOn(CRGB color);
void ledOff();
void setServoAngle(int angle);
void centerServo();

bool setupGyro();
void calibrateGyro();
int getDistance_accuracy();
int getDistance();
<<<<<<< HEAD
int seenColorVals[3];
=======
const int SERVO_CENTER_ANGLE = 135;
void handle_ir(unsigned long code);
void discreate_sweep();
void sonar_debug();
void line_debug();

>>>>>>> ba6d1a8 (New features, Code Regactored)


