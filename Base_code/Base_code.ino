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
int seenColorVals[3];
int state = 0;  // 0 = idle/stopped, 1 = forward/scanning, 2 = wall detected
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

  delay(500);

  // Initialize Gyro - hard stop if failed
  if (!setupGyro()) {
    ledOn(CRGB::Red);
    while (true)
      ;  // Hard stop
  }

  calibrateGyro();
  state = 1;
  seenColorVals[0] = (analogRead(LINE_L));
  seenColorVals[1] = (analogRead(LINE_R));
  seenColorVals[2] = (analogRead(LINE_C));
  // Serial.println(seenColorVals[0]);
  // Serial.println(seenColorVals[1]);
  // Serial.println(seenColorVals[2]);
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
//int angles[] = {90, 50, 40, 30, 90, 150, 160, 170, 90};
//int angles[] = {30, 50, 70, 90, 110, 130, 150, 170, 150, 130, 110, 90, 70, 50};
//int angles[] = {90, 40, 70, 90, 90, 90, 140, 170};
//              0   1   2   3   4   5   6     7
int angles[] = { 90, 10, 10, 90, 90, 170, 170 };
//              0   1   2   3    4   5    6

bool go = true;
int turn_speed = 200;
int turn_duration = 120;
//=======================================================================


//===========================THE CODE======================================



bool detectNewColor() {
  int maxVal = max(seenColorVals[2], max(seenColorVals[0], seenColorVals[1])) + 130;
  int minVal = min(seenColorVals[2], min(seenColorVals[0], seenColorVals[1])) - 130;
  int left = analogRead(LINE_L);
  int center = analogRead(LINE_C);
  int right = analogRead(LINE_R);
  // delay(300);
  Serial.println(avoid);
  // Serial.println(maxVal);
  // Serial.println(minVal);
  // Serial.print("C: ");
  // Serial.println(analogRead(LINE_C));
  // Serial.print("r: ");
  // Serial.println(analogRead(LINE_R));
  // Serial.print("l: ");
  // Serial.println(analogRead(LINE_L));
  if (avoid) {
    BASESPEED = 90;
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
    resetAngle();
    return false;
  }

  else {
    BASESPEED = 40;
    if (minVal > center || center > maxVal) {
      backward(20);
      delay(200);
      stop();
      for (int i = 0; i < 4; i++) {
        turnGyro('r');
        if (minVal < center && center < maxVal) return;
      }
      turnGyro('l', 90, 90);
      for (int i = 0; i < 4; i++) {
        turnGyro('l');
        if (minVal < center && center < maxVal) return;
      }
      if (minVal > center || center > maxVal) {
        state = 0;
        avoid = !avoid;
      }
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

  switch (state) {

    case 0:
      stop();
      ledOn(CRGB(120, 20, 200));
      while (detectNewColor()) {
        if (digitalRead(BUTTON) == LOW) {
          seenColorVals[0] = (analogRead(LINE_L));
          seenColorVals[1] = (analogRead(LINE_R));
          seenColorVals[2] = (analogRead(LINE_C));
          avoid = !avoid;
          break;
        }
      }
      state = 1;
      break;

    case 1:
      ledOn(CRGB(60, 230, 60));
      forward(BASESPEED);
      delay(450);
      servoSweep();
      if (detectNewColor()) {
        state = 0;
        stop();
      }
      if ((r_distance < 25) || (l_distance < 25) || (straight_distance < 20)) {
        state = 2;
        stop();
      }
      break;

    case 2:
      ledOn(CRGB(230, 90, 20));
      servoSweep();
      delay(200);
      courseCorrection();
      if ((r_distance > 25) && (l_distance > 25) && (straight_distance > 20)) {
        state = 1;
        turns = 0;
      }
      if (turns >= 10) {
        state = 0;
        turns = 0;
        avoid = !avoid;
      }
      turns++;
      // Serial.print(turns);
      break;
  }

  // Serial.print(millis());

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