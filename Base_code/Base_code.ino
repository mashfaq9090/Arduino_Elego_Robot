/*
 * CSCI 1063U - Elegoo Smart Car V4.0
 *
 * 
*/

/*
 ----> Line sensor threshold needs to be mannually calibrated depending on maze structure. 
 This code follows light runway with dark barriers

 ---> The Rover runs with or without gyroscope(controled via gyro_using variable)...incase not using gyroscope 
 the turn_raw must be mannual calibrated

 ---> Fell free to change the code as needed :) 

*/
#include "pin_def.h"
#include "utils.h"
#include <FastLED.h>
#include <Servo.h>
#include <Wire.h>


void setup() {
  //random seed 
  bool gyro_using = false;
  randomSeed(analogRead(0));

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

  if (!setupGyro() && gyro_using) {
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

void backward(int speed){
  stop();
  digitalWrite(MTR_L, LOW);
  digitalWrite(MTR_R, LOW);
  analogWrite(PWR_R, speed);
  analogWrite(PWR_L, speed + 3);
}



void turn_raw(char direction, int ms=370, int speed=145 ){ //time based turning...calibrate time and speed manually
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

bool go = true;
//-----------Use only when you want to globally contro the turning parameter---------------
int turn_speed = 200;
int turn_duration = 120;
int turn_gyro_speed = 50;
//----------------------------------------------------------------------------------
bool sonar_triger = false; //use only when implementing sonar based navigation
bool line_triger = false;
bool turn_90triger = false;
int line_right = 0;
int line_left = 0;
int line_middle = 0;
int base_speed = 50;
int distance = 10000;
int left_or_right = 0;
int r_distance, straight_distance, l_distance; //used for 
//=======================================================================


//===========================THE CODE======================================

void loop() {

  line_left = analogRead(LINE_L) - 170; //calibrating...this one is usually a bit high than others
  line_middle = analogRead(LINE_C);
  line_right = analogRead(LINE_R);
  distance = getDistance();
  
  if (go) {
    forward(base_speed);
    go = false;
  }


  turn_90triger = (line_left > 600 && line_right > 600 && line_middle > 600); //trigers full 90 degree rotation when all sensors are in the tape
  line_triger = (line_left > 600 || line_right > 600); // for attitude adjustment 


//=============================Box entrance logic=================
  if (distance < 18){
      while (distance > 8){
        forward(100);
        distance = getDistance();
        Serial.println(distance);
      }
      stop();
      digitalWrite(MTR_ENABLE, LOW);
  }


//================Course Correction Triger Logic===========
  if (line_triger || turn_90triger){
    stop();
    course_corection();
    go = true;
  } 

}

//=================Course_correction========================
void course_corection() {

  left_or_right = random(2);  // random logic only used for custom maze....
  //better use right only turn for Original maze...alrhough random should work for both

  if (line_left > 600 && line_right > 600 && line_middle > 600){ 
    Serial.println("=========TURN 90=========");
    backward(35);
    delay(200);
    stop();
    delay(400);
    if (left_or_right == 0) {
      //turn_raw('l', 600, 100); //------------> optimized for 90 degree turn
      turn_raw('r', 550, 100);  //------------> Used for Custom maze
    } 

    if (left_or_right == 1) {
      //turn_raw('l', 600, 100); //------------> optimized for 90 degree turn
      turn_raw('l', 550, 100); //------------> Used for Custom maze
    } 
    return;
  }

  //for attitude corection 
  if (line_right > line_left && abs(line_right - line_left) > 160){
    Serial.println("=========LEFT=========");
    //turnGyro('l', 10, 100, 50);
    turn_raw('l', 140, 90);
    return;
    
  }

  if (line_left > line_right && abs(line_right - line_left) > 160){
    Serial.println("=========RIGHT=========");
    //turnGyro('r', 10, 100, 50);
    turn_raw('r', 140, 90); 
    return;

  }

}

//Servo Sweep ---> angle_array accepts an set of exact angle the servo needs to traverse like the following
// //int angles[] = {90, 40, 70, 90, 90, 90, 140, 170};
void servo_sweep(int& r_distance, int& straight_distance, int& l_distance, int* angle_array, int array_len){
  unsigned long lastServoMove = 0;
  int sweepStep = array_len;

    if (millis() - lastServoMove > 300) {
      setServoAngle(angle_array[sweepStep]);
      if ((sweepStep == 1) || (sweepStep == 2)) {r_distance = getDistance();}
      if (angle_array[sweepStep] == 90) {straight_distance = getDistance();}
      if ((sweepStep == 5) || (sweepStep == 6)) {l_distance = getDistance();}

      sweepStep++;
      if (sweepStep > 6) {
        sweepStep = 0; // Reset sweep
      }
      
      lastServoMove = millis();
    }
}



//============================ Sonar Sensor debug====================
void sonar_debug(){
  Serial.print("R: ");
  Serial.print(r_distance);
  Serial.print(" | L: ");
  Serial.print(l_distance);
  Serial.print(" | Straight: ");
  Serial.println(straight_distance);
  Serial.println("distance" + String(distance));
}

//=========================Line Sensor debug================
void line_debug(){
  Serial.print("R: ");
  Serial.print(line_right);
  Serial.print(" | L: ");
  Serial.print(line_left);
  Serial.print(" | middle: ");
  Serial.print(line_middle);
  Serial.println("");
}


// ===================Gyro Sensor Debug====================== 
void gyro_debug(){   
  updateGyroAngle();
  int angle = getAngle();
  Serial.print("Angle: ------> ");
  Serial.println(angle);
  Serial.print("*");
  Serial.println("");
}