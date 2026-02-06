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

void ledOn(CRGB color);
void ledOff();
void setServoAngle(int angle);
void centerServo();

bool setupGyro();
void calibrateGyro();
int16_t readGyroZ();
void updateGyroAngle();
void resetAngle();
float getAngle();
int getDistance_accuracy();
int getDistance();
