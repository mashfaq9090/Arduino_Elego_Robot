#pragma once

#include <FastLED.h>
#include <Servo.h>

// ====== PROGRAM VARIABLES ======
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
const int SERVO_CENTER_ANGLE = 135;
void handle_ir(unsigned long code);
void discreate_sweep();
void sonar_debug();
void line_debug();


