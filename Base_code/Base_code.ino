/*
 * CSCI 1063U - Elegoo Smart Car V4.0
 *
 * Starter Code for motor, pin, and sensor setup
 * Provided to students for use and understanding
 * 
 */

#include "pin_def.h"
#include "utils.h"
#include <FastLED.h>
#include <Servo.h>
#include <Wire.h>
int seenColorVals[6];
int mode = 0;  // 0 = idle/stopped, 1 = forward/scanning, 2 = wall detected
bool avoid = true;
int turns = 0;
int BASESPEED = 90;
void setup() {
  // setup LED
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(50);  // 0-255

  // Motor pins
  pinMode(PWR_R, OUTPUT);
  pinMode(PWR_L, OUTPUT);
  pinMode(MTR_L, OUTPUT);
  pinMode(MTR_R, OUTPUT);
  pinMode(MTR_ENABLE, OUTPUT);

  // Ultrasonic sensor pins
  pinMode(US_OUT, OUTPUT);
  pinMode(US_IN, INPUT);

  // Line tracking sensor pins
  pinMode(LINE_L, INPUT);
  pinMode(LINE_C, INPUT);
  pinMode(LINE_R, INPUT);

  // Button pin
  pinMode(BUTTON, INPUT_PULLUP);

  // Enable motor driver
  digitalWrite(MTR_ENABLE, HIGH);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize Servo motor
  scanServo.attach(SERVO);
  centerServo();  // Center position
  // Wait for button press
  while (digitalRead(BUTTON) == HIGH) {
  }
  ledOn(CRGB(98, 98, 98));

  delay(500);

  // Initialize Gyro - hard stop if failed
  if (!setupGyro()) {
    ledOn(CRGB::Red);
    while (true);  // Hard stop
  }
  ledOff();
  calibrateGyro();
  mode = 1;
  seenColorVals[0] = (analogRead(LINE_L) - 170);  // inaccuracy offset
  seenColorVals[1] = (analogRead(LINE_R));
  seenColorVals[2] = (analogRead(LINE_C));
}

//===================MOTOR FUNCTIONS============================

void stop() {
  analogWrite(PWR_R, 0);
  analogWrite(PWR_L, 0);
}

void forward(int speed) {
  stop();
  digitalWrite(MTR_L, HIGH);
  digitalWrite(MTR_R, HIGH);
  analogWrite(PWR_R, speed);
  analogWrite(PWR_L, speed + 3);
}
void backward(int speed) {
  stop();
  digitalWrite(MTR_L, LOW);
  digitalWrite(MTR_R, LOW);
  analogWrite(PWR_R, speed);
  analogWrite(PWR_L, speed + 3);
}

void turnRaw(char direction, int ms = 370, int speed = 145) {
  stop();
  if (direction == 'l') {
    digitalWrite(MTR_L, HIGH);
    digitalWrite(MTR_R, LOW);
  } else if (direction == 'r') {
    digitalWrite(MTR_L, LOW);
    digitalWrite(MTR_R, HIGH);
  }
  analogWrite(PWR_R, speed);
  analogWrite(PWR_L, speed);
  delay(ms);
  stop();
}

void turnGyro(char direction, int degrees = 15, int speed = 145) {
  stop();
  if (direction == 'l') {
    degrees = -degrees;
    digitalWrite(MTR_L, HIGH);
    digitalWrite(MTR_R, LOW);
    resetAngle();
    while (getAngle() > degrees) {
      analogWrite(PWR_R, speed);
      analogWrite(PWR_L, speed);
      updateGyroAngle();
    }
  } else if (direction == 'r') {
    digitalWrite(MTR_L, LOW);
    digitalWrite(MTR_R, HIGH);
    resetAngle();
    while (getAngle() < degrees) {
      analogWrite(PWR_R, speed);
      analogWrite(PWR_L, speed);
      updateGyroAngle();
    }
  }

  stop();
}

//=============================User Variable============================
int straight_distance = 0;
int r_distance = 10000;
int l_distance = 10000;
unsigned long lastServoMove = 0;
int sweepStep = 0;
int lastCourseChange = 0; // 0 is none, 1 is line sensor, 2 is servo sensor
int angles[] = { 90, 10, 10, 90, 90, 170, 170 };
//              0   1   2   3    4   5    6

bool go = true;
int turn_speed = 200;
int turn_duration = 120;
//=======================================================================


//===========================THE CODE======================================

void changeColors(){
  seenColorVals[0] = seenColorVals[3];
  seenColorVals[1] = seenColorVals[4];
  seenColorVals[2] = seenColorVals[5];
  seenColorVals[0] = (analogRead(LINE_L));
  seenColorVals[1] = (analogRead(LINE_R));
  seenColorVals[2] = (analogRead(LINE_C));
}

bool detectNewColor() {
  lastCourseChange = 1;
  int maxVal = max(seenColorVals[2], max(seenColorVals[0], seenColorVals[1])) + 80;
  int minVal = min(seenColorVals[2], min(seenColorVals[0], seenColorVals[1])) - 80;
  int left = analogRead(LINE_L);
  int center = analogRead(LINE_C);
  int right = analogRead(LINE_R);

  if (avoid) {
    BASESPEED = 80;
    if (minVal > center || center > maxVal) {
      stop();
      return true;
    }
    if (minVal > right || right > maxVal) {
      stop();
      return true;
    }
    if (minVal > left || left > maxVal) {
      stop();
      return true;
    }
    return false;
  }

  else {
    BASESPEED = 80;
    if (minVal > center || center > maxVal) {
      backward(20);
      delay(200);
      stop();
      for (int i = 0; i < 4; i++) {
        turnGyro('r');
        if (minVal < center && center < maxVal) return false;
      }
      turnGyro('l', 90, 90);
      for (int i = 0; i < 4; i++) {
        turnGyro('l');
        if (minVal < center && center < maxVal) return false;
      }
      changeColors();
      return true;
    }
    if (minVal > right || right > maxVal) {
      turnGyro('l', 10);
      return true;
    }
    if (minVal > left || left > maxVal) {
      turnGyro('r', 10);
      return true;
    }
    return false;
  }
}
void servoSweep() {

  // Only move the servo every 500ms
  if (millis() - lastServoMove > 150) {
    setServoAngle(angles[sweepStep]);
    if ((sweepStep == 1) || (sweepStep == 2)) { r_distance = getDistance(); }
    if (angles[sweepStep] == 90) { straight_distance = getDistance(); }
    if ((sweepStep == 5) || (sweepStep == 6)) { l_distance = getDistance(); }

    sweepStep++;
    sweepStep %= 6;

    lastServoMove = millis();
  }
}
void loop() {
  switch (mode) {

    case 0:{
      stop();
      ledOn(CRGB(120, 20, 120));
      unsigned long start = millis();
      unsigned long wait = 3000;  // wait 3 secs
      while (detectNewColor()) {

        if (digitalRead(BUTTON) == LOW) {
          changeColors();
          break;
        }

        if (millis() - start > wait) {
          turnGyro('r', 180, 90);
          break;
        }
      }
      mode = 1;
      break;
      }
    case 1:{
      ledOn(CRGB(60, 230, 60));
      forward(BASESPEED);
      delay(200);
      servoSweep();
      //prioritizes the the last used course correction mode last, defaults to obstacle first
      if (lastCourseChange == 2){
        if (detectNewColor()) {
          mode = 0;
          stop();
        }
        if ((r_distance < 25) || (l_distance < 25) || (straight_distance < 20)) {
          mode = 2;
          stop();
        }
      }
      else{
        if ((r_distance < 25) || (l_distance < 25) || (straight_distance < 20)) {
          mode = 2;
          stop();
        }
        if (detectNewColor()) {
          mode = 0;
          stop();
        }
      }
      break;
      }
    case 2:{
      ledOn(CRGB(230, 90, 20));
      servoSweep();
      delay(200);
      courseCorrection();
      if ((r_distance > 25) && (l_distance > 25) && (straight_distance > 20)) {
        mode = 1;
        turns = 0;
      }
      if (turns >= 3) {
        mode = 0;
        turns = 0;
        avoid = !avoid;
      }
      turns++;
      break;
      }
  }


  //============================debug====================
  // Serial.print("R: ");
  // Serial.print(r_distance);
  // Serial.print(" | L: ");
  // Serial.print(l_distance);
  // Serial.print(" | Straight: ");
  // Serial.println(straight_distance);

  //======================================================
}
//=====================================================================

//===============================SLAM==========================
void courseCorrection() {
  lastCourseChange = 2;
  if (r_distance < l_distance) {
    // Serial.println("=========LEFT=========");
    turnGyro('l', 30);
    centerServo();
    delay(10);
    sweepStep = 0;
    r_distance = 10000;
    l_distance = 10000;
    straight_distance = 10000;
  }

  else if (r_distance > l_distance) {
    // Serial.println("=========RIGHT=========");
    turnGyro('r', 30);
    centerServo();
    delay(10);
    sweepStep = 0;
    r_distance = 10000;
    l_distance = 10000;
    straight_distance = 10000;
  }

  else if (r_distance == l_distance) {
    r_distance = 10000;
    l_distance = 10000;
    straight_distance = 10000;
    ledOn(CRGB::Red);
    setServoAngle(90);
    delay(200);
    if (getDistance() < 10) {
      setServoAngle(0);
      delay(500);
      r_distance = getDistance();
      setServoAngle(180);
      delay(500);
      l_distance = getDistance();
      ledOn(CRGB::Pink);
    }
  }
}
//=====================================================================