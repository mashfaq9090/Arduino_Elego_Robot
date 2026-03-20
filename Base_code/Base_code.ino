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
int mode = 0;  // -1 = danger, 0 = idle/stopped, 1 = forward/scanning, 2 = wall detected
// bool avoid = true;
int turns = 0;
int BASESPEED = 70;
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
  centerServo();  // left position
  // Wait for button press
  while (digitalRead(BUTTON) == HIGH) {
  }
  ledOn(CRGB(98, 98, 98));

  delay(500);

  // Initialize Gyro - hard stop if failed
  if (!setupGyro()) {
    ledOn(CRGB::Red);
    while (true)
      ;  // Hard stop
  }
  ledOff();
  calibrateGyro();
  mode = 1;
  // analog reading left is broken ( senor cover/chasis has fallen off somehow)
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
int c_dist = 0;
int r_dist = 10000;
int l_dist = 10000;
int servoAngle = 0;
unsigned long lastServoMove = 0;
int sweepStep = 0;
unsigned long currTime = millis();
int actionNum = 0;
int actionList[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int lastCourseChange = 0;  // 0 is none, 1 is line sensor, 2 is servo sensor
int angles[] = { 90, 10, 10, 90, 90, 170, 170 };
//              0   1   2   3    4   5    6

int turn_speed = 200;
int turn_duration = 120;
//=======================================================================


//===========================THE CODE======================================

//
void detectLine(bool follow) {
  int left = analogRead(LINE_L);
  int right = analogRead(LINE_R) + 30;
  if (follow){
    if(left > 500){
      turnGyro('r');
    }
    if (right < 500){
      turnGyro('l');
    }
  }
  else{
    if (left < 500 && right <500){
      backward(BASESPEED);
      delay(200);
      turnGyro('r');
    }
    
    if (left < 500){
      backward(BASESPEED);
      delay(200);
      turnGyro('r');
    }
    if (right < 500){
      backward(BASESPEED);
      delay(200);
      turnGyro('l');
    }
    
  }

}
// may or may not work needs testing
bool detectLedge() {
  int left = analogRead(LINE_L);
  int right = analogRead(LINE_R) + 30;


  int currAvg = (left + right) / 2;
  lastCourseChange = 1;

  BASESPEED = 70;

  if (currAvg > 950) {


    backward(40);
    delay(500);
    turnGyro(50);
      stop();

    return true;
  }
  return false;
}

void servoSweep() {

  // Only move the servo every 500ms
  if (millis() - lastServoMove > 150) {
    setServoAngle(angles[sweepStep]);
    if ((sweepStep == 1) || (sweepStep == 2)) { r_dist = min(getDistance(), 10000); }
    if (angles[sweepStep] == 90) { c_dist = min(getDistance(), 10000); }
    if ((sweepStep == 5) || (sweepStep == 6)) { l_dist = min(getDistance(), 10000); }

    sweepStep++;
    sweepStep %= 6;
    servoAngle = angles[sweepStep];
    lastServoMove = millis();
  }
}
void sweepTo(int degrees) {
  if (degrees == servoAngle) { return; }
  for (int i = 0; i <= 3; i++) {
    if (degrees > servoAngle) {
      setServoAngle(servoAngle + (degrees / 3));
    }
    if (degrees < servoAngle) {
      setServoAngle(servoAngle - (degrees / 3));
    }
    delay(100);
  }
  servoAngle = degrees;
}

void loop() {
  
  
  switch (mode) {

    case -1:
      {
        stop();
        ledOff();
        digitalWrite(MTR_ENABLE, LOW);
      }

    case 0:
      {
        stop();
        ledOn(CRGB(120, 20, 120));
        unsigned long start = millis();
        unsigned long wait = 3000;  // wait 3 secs
        while (detectLedge()) {

          if (digitalRead(BUTTON) == LOW) {
            goBack();
            break;
          }
          if (millis() - start > wait) {
            mode = -1;
            return;
          }
        }
        mode = 1;
        currTime = millis();
        break;
      }
    case 1:
      {

        currTime = millis();
        ledOn(CRGB(60, 230, 60));
        forward(BASESPEED);
        delay(200);
        servoSweep();
        //finds last empty array

        if (actionNum == 10) {
          goBack();
          mode = -1;
        }
        //prioritizes the the last used course correction mode last, defaults to obstacle first
        if (lastCourseChange == 2) {
          if (detectLedge()) {
            actionList[actionNum] = millis() - currTime;
            actionNum++;
            mode = 0;
            stop();
          }
          if ((r_dist < 25) || (l_dist < 25) || (c_dist < 20)) {
            actionList[actionNum] = millis() - currTime;
            actionNum++;
            mode = 2;
            stop();
          }
        } else {
          if ((r_dist < 25) || (l_dist < 25) || (c_dist < 20)) {
            actionList[actionNum] = millis() - currTime;
            actionNum++;
            mode = 2;
            stop();
          }
          if (detectLedge()) {
            actionList[actionNum] = millis() - currTime;
            actionNum++;
            mode = 0;
            stop();
          }
        }
        break;
      }

    case 2:
      {
        ledOn(CRGB(230, 90, 20));
        servoSweep();
        delay(200);
        courseCorrection();
        if ((r_dist > 25) && (l_dist > 25) && (c_dist > 20)) {
          mode = 1;
          turns = 0;
        }
        if (turns >= 3) {
          turns = 0;
          int tries = 0;
          while ((r_dist < 25 || l_dist < 25 || c_dist < 20) && tries <= 3) {
            backward(BASESPEED);
            delay(2000);
            stop();
            servoSweep();
            tries++;
          }
          if (tries == 4) {
            mode = -1;
          } else {
            mode = 1;
          }
        }
        turns++;
        break;
      }
  }


  //============================debug====================
  // Serial.print("R: ");
  // Serial.print(r_dist);
  // Serial.print(" | L: ");
  // Serial.print(l_dist);
  // Serial.print(" | Straight: ");
  // Serial.println(c_dist);

  //======================================================
}
//=====================================================================

//===============================SLAM==========================
void courseCorrection() {
  if (c_dist < 25) {
    turnGyro('r', 90);
    actionList[actionNum] = 90;
    actionNum++;
    if (c_dist < 25) {
      turnGyro('l', 180);
      actionList[actionNum] = -180;
      actionNum++;
    }
  }

  lastCourseChange = 2;
  if (r_dist < l_dist) {
    // Serial.println("=========LEFT=========");
    turnGyro('l', 20);
    actionList[actionNum] = -20;
    actionNum++;
    centerServo();
    delay(10);
    sweepStep = 0;
    r_dist = 10000;
    l_dist = 10000;
    c_dist = 10000;
  }

  else if (r_dist > l_dist) {
    // Serial.println("=========RIGHT=========");
    turnGyro('r', 20);
    actionList[actionNum] = 20;
    actionNum++;
    centerServo();
    delay(10);
    sweepStep = 0;
    r_dist = 10000;
    l_dist = 10000;
    c_dist = 10000;
  }

  else if (r_dist == l_dist) {
    r_dist = 10000;
    l_dist = 10000;
    c_dist = 10000;
    ledOn(CRGB::Red);
    centerServo();

    delay(200);
    if (getDistance() < 10) {
      sweepTo(0);
      delay(500);
      r_dist = getDistance();
      sweepTo(180);
      delay(500);
      l_dist = getDistance();
      ledOn(CRGB::Pink);
    }
  }
}

void goBack() {
  ledOn(CRGB::Chocolate);
  // Serial.println(actionList);
  for (int i = 9; i > 0; i--) {
    
    if (actionList[i] % 2 != 0) {
      backward(BASESPEED);
      delay(actionList[i]);
    }

    if (actionList[i] % 2 == 0 && actionList[i] > 0) {
      turnGyro('l', -(actionList[i]), 145);
    }

    if (actionList[i] % 2 == 0 && actionList[i] < 0) {
      turnGyro('r', actionList[i], 145);
    }
  }
}

//=====================================================================