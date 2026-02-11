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


void setup() {

  // setup LED
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(50); // 0-255
  
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
  centerServo();   // Center position
  
  // Wait for button press
  while (digitalRead(BUTTON) == HIGH) {

  }

  delay(500);

  // Initialize Gyro - hard stop if failed
  if (!setupGyro()) {
    ledOn(CRGB::Red);
    while (true);  // Hard stop
  }

  calibrateGyro();
  
}

//===================MOTOR FUNCTIONS============================

void stop(){
  analogWrite(PWR_R, 0);
  analogWrite(PWR_L, 0); 
}

void forward(int speed){
  stop();
  digitalWrite(MTR_L, HIGH);
  digitalWrite(MTR_R, HIGH);
  analogWrite(PWR_R, speed);
  analogWrite(PWR_L, speed + 3);
}

void turn_raw(char direction, int ms=370, int speed=145 ){
  stop();
  if (direction=='l'){
    digitalWrite(MTR_L,HIGH);
    digitalWrite(MTR_R,LOW);
  }else if (direction=='r') {
    digitalWrite(MTR_L,LOW);
    digitalWrite(MTR_R,HIGH);
  }
  analogWrite(PWR_R, speed);
  analogWrite(PWR_L, speed);
  delay(ms);
  stop();
}

void turnGyro(char direction, int targetDegrees = 15, int speed = 120, int ms = 90) {
  stop();
  resetAngle();
  
  unsigned long startTime = millis();
  const unsigned long timeout = 2000; // safety timeout (2 seconds)

  // Start turning continuously
  if (direction == 'l') {
    digitalWrite(MTR_L, HIGH);
    digitalWrite(MTR_R, LOW);
  } else if (direction == 'r') {
    digitalWrite(MTR_L, LOW);
    digitalWrite(MTR_R, HIGH);
  }

  analogWrite(PWR_R, speed);
  analogWrite(PWR_L, speed);

  while (true) {
    updateGyroAngle();
    int angle = getAngle();
    //Serial.println(angle);

    // Left turn condition
    if (direction == 'l' && angle <= -targetDegrees){
      stop();
      //Serial.println("=====BREAK========");
      break;
    }

    // Right turn condition
    if (direction == 'r' && angle >= targetDegrees){
      stop();
      //Serial.println("=====BREAK========");
      break;
    }

    // Timeout safety
    if (millis() - startTime > timeout){
      break;
    }
  }

  stop();
  delay(ms);
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
int angles[] = {90, 10, 10, 90, 90, 170, 170};
//              0   1   2   3    4   5    6
bool go = true;
int turn_speed = 200;
int turn_duration = 120;
int turn_gyro_speed = 50;
bool sonar_triger = false;
bool line_triger = false;
int line_right = 0;
int line_left = 0;
int line_middle = 0;
int base_speed = 35;
//=======================================================================


//===========================THE CODE======================================

void loop() {

  line_left = analogRead(LINE_L);
  line_middle = analogRead(LINE_C);
  line_right = analogRead(LINE_R);


  if (go) {
    forward(base_speed);
    //turnGyro('l', 25, 50, 90);
    go = false;
  }

  // Only move the servo every 500ms
  if (millis() - lastServoMove > 300) {
    setServoAngle(angles[sweepStep]);
    if ((sweepStep == 1) || (sweepStep == 2)) {r_distance = getDistance();}
    if (angles[sweepStep] == 90) {straight_distance = getDistance();}
    if ((sweepStep == 5) || (sweepStep == 6)) {l_distance = getDistance();}

    sweepStep++;
    if (sweepStep > 6) {
      sweepStep = 0; // Reset sweep
    }
    
    lastServoMove = millis();
  }

  sonar_triger = (r_distance < 25) || (l_distance < 25) || (straight_distance < 20);
  line_triger = (line_left > 500 || line_right > 500);

  if (sonar_triger || line_triger){
    stop();
    course_corection();
    go = true;
  } 
  //============================debug====================
  // Serial.print("R: ");
  // Serial.print(r_distance);
  // Serial.print(" | L: ");
  // Serial.print(l_distance);
  // Serial.print(" | Straight: ");
  // Serial.println(straight_distance);

  // //=========================line - debug================
  // Serial.print("R: ");
  // Serial.print(line_right);
  // Serial.print(" | L: ");
  // Serial.print(line_left);
  // Serial.print(" | middle: ");
  // Serial.println(line_middle);

  //======================================================
}
//=====================================================================

//===============================SLAM==========================
void course_corection() {

  if (line_right > line_left){
    Serial.println("=========LEFT=========");
    turnGyro('l', 25, turn_gyro_speed, 90);
    
  }

  if (line_left > line_right){
    Serial.println("=========RIGHT=========");
    turnGyro('r', 25, turn_gyro_speed, 90);

  }


  if (r_distance < l_distance){
    Serial.println("=========LEFT=========");
    turn_raw('l', turn_speed, turn_duration);
    centerServo();
    delay(10);
    sweepStep = 0;
    r_distance = 10000;
    l_distance = 10000;
    straight_distance = 10000;
  }

  if (r_distance > l_distance){
    Serial.println("=========RIGHT=========");
    turn_raw('r', turn_speed, turn_duration);
    centerServo();
    delay(10);
    sweepStep = 0;
    r_distance = 10000;
    l_distance = 10000;
    straight_distance = 10000;
  }

  if ((r_distance == l_distance) && (straight_distance < 20)){
    r_distance = 10000;
    l_distance = 10000;
    straight_distance = 10000;
    ledOn(CRGB::Red);
    setServoAngle(90); //looking straight
    delay(200);
    if (getDistance() < 10){
      setServoAngle(0); //looking right
      delay(500);
      r_distance = getDistance();
      setServoAngle(180); //looking left
      delay(500);
      l_distance = getDistance();
      setServoAngle(90);
      delay(325);
      ledOn(CRGB::Purple);
      course_corection(); //recursion untill clears out of trap
    }
  }
}
//=====================================================================