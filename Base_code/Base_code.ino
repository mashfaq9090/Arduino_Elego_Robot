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
#include <IRremote.hpp>  
#include "IR_receiver.h"
#include "pin_def.h"
#include "utils.h"
#include <FastLED.h>
#include <Servo.h>
#include <Wire.h>
#include "attitude.h"
#include "rover_camera_bridge.h"

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
int r_distance, straight_distance, l_distance = 0; 
int angle_array[] = {90, 40, 70, 90, 90, 90, 140, 170};
bool is_going_forward = false;
int stuck_counter = 0;
bool debug = false;
bool line_logic = true;
bool sonar_logic = false;
//=======================================================================


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
  Serial.begin(1000000);

  // Initialize Servo motor
  scanServo.attach(SERVO);
  centerServo();   // Center position
  ledOff();
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
<<<<<<< HEAD
  seenColorVals[0] = (analogRead(LINE_L)-170);
  seenColorVals[1] = (analogRead(LINE_R));
  seenColorVals[2] = (analogRead(LINE_C));
  
=======
  ir_init();

>>>>>>> ba6d1a8 (New features, Code Regactored)
}

//===================MOTOR FUNCTIONS============================

void stop(){
  analogWrite(PWR_R, 0);
  analogWrite(PWR_L, 0); 
}

/*
 * stop_graceful():
 *   Ramps speed down from current_speed to 0 smoothly
 *   current_speed — the speed the rover is currently running at
 *   step_delay    — ms between each speed decrement (lower = faster brake)
 */
void stop_graceful(int current_speed, int step_delay = 15) {
  for (int s = current_speed; s >= 0; s -= 5) {
    analogWrite(PWR_R, s + 3);
    analogWrite(PWR_L, s);
    delay(step_delay);
  }
  // Hard zero after ramp to guarantee full stop
  analogWrite(PWR_R, 0);
  analogWrite(PWR_L, 0);
}

void forward(int speed){
  stop();
  digitalWrite(MTR_L, HIGH);
  digitalWrite(MTR_R, HIGH);
  analogWrite(PWR_R, speed+ 3);
  analogWrite(PWR_L, speed + 3);
}

/*
 * forward_to_obstacle():
 *   Drives forward at full speed, then scales speed down
 *   as distance shrinks, stops at stop_dist (cm)
 *
 *   speed     — cruising speed (full speed when far away)
 *   stop_dist — distance in cm to stop at
 */
void forward_to_obstacle(int speed, int stop_dist) {
  const int SLOW_DIST = 40; // cm — begin slowing below this distance

  while (true) {
    int dist = getDistance();

    // Stop condition — reached target distance
    if (dist <= stop_dist) {
      stop();
      return;
    }

    // Scale speed down as distance shrinks toward stop_dist
    int current_speed;
    if (dist <= SLOW_DIST) {
      current_speed = map(dist, stop_dist, SLOW_DIST, 30, speed);
      current_speed = constrain(current_speed, 30, speed);
    } else {
      current_speed = speed;  // full speed when far away
    }

    // Drive forward at calculated speed
    digitalWrite(MTR_L, HIGH);
    digitalWrite(MTR_R, HIGH);
    analogWrite(PWR_R, current_speed);
    analogWrite(PWR_L, current_speed + 3);

    // Serial.print("dist: "); Serial.print(dist);
    // Serial.print("cm | speed: "); Serial.println(current_speed);

    delay(50); // small tick delay for smooth speed updates
  }
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

bool detectNewColor() {
  int maxVal = max(seenColorVals[2], max(seenColorVals[0], seenColorVals[1])) + 130;
  int minVal = min(seenColorVals[2], min(seenColorVals[0], seenColorVals[1])) - 130;
  Serial.println(avoid);
  unsigned long startTime = millis();
  const unsigned long timeout = 2000;
  bool avoid = true;
  if (avoid) {
    if (minVal > analogRead(LINE_C) || analogRead(LINE_C) > maxVal) {
      while (digitalRead(BUTTON) == HIGH) {
        ledOn(CRGB::Red);
        stop();
        if (millis() - startTime > timeout){
          turn_raw('l', 140, 90);
          turn_raw('l', 140, 90);
          break;
      }
      }
      
      return true;
    }
    if (minVal > analogRead(LINE_R) || analogRead(LINE_R) > maxVal) {
      while (digitalRead(BUTTON) == HIGH) {
        ledOn(CRGB::Red);
        stop();
        if (millis() - startTime > timeout){
          turn_raw('l', 140, 90);
          turn_raw('l', 140, 90);
          break;
      }
      }
      return true;
    }
    if (minVal > analogRead(LINE_L) || analogRead(LINE_L) > maxVal) {
      while (digitalRead(BUTTON) == HIGH) {
        ledOn(CRGB::Red);
        stop();
        if (millis() - startTime > timeout){
          turn_raw('l', 1200, 100);
  
          break;
      }
      }
    return false;
  }

  else {
    
    if (minVal > analogRead(LINE_C) || analogRead(LINE_C) > maxVal) {
      backward(30);
      return true;
    }
    if (minVal > analogRead(LINE_R) || analogRead(LINE_R) > maxVal) {
      turn_raw('l',50,50);
      return true;
    }
    if (minVal > analogRead(LINE_L) || analogRead(LINE_L) > maxVal) {
      turn_raw('r',50,50);
      return true;
    }
    return false;
  }
}
}

void turnGyro(char direction, int targetDegrees = 15, int speed = 120, int ms = 90) { 
  is_going_forward = false;
  stop();
  resetAttitude();
  
  
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
    updateAttitude();
    int angle = attitude.angle.z;
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

<<<<<<< HEAD
//=============================User Variable============================
bool new_color= false;
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
=======
/*
 * straight_line_pd():
 *   PD controller for gyro-guided straight driving
 *
 *   P term — corrects based on current angle (how far off are we?)
 *   D term — corrects based on rate of change (are we getting worse?)
 *
 *   Tune KP first (start at 2.0, increase until it fights drift)
 *   Then tune KD to dampen oscillation (start at 0.5)
 */

// PD tuning constants — adjust these
float KP = 8;   // proportional gain — increase if still drifting
float KD = 0.8;   // derivative gain   — increase if oscillating
>>>>>>> ba6d1a8 (New features, Code Regactored)

float last_angle = 0;  // needed for D term — declare globally above loop()

void straight_line_pd(int speed, bool debug = false) {
  updateAttitude();
  float angle = -(attitude.angle.z);

  is_going_forward = true;

  // P term — how far off are we right now
  float p_term = angle * KP;

  // D term — how fast is the angle changing (rate of drift)
  float d_term = (angle - last_angle) * KD;
  last_angle = angle;

  // Combined correction
  int correction = constrain((int)(p_term + d_term), -45, 45);

  digitalWrite(MTR_L, HIGH);
  digitalWrite(MTR_R, HIGH);

  if (correction >= 0) {
    // Drifting right — boost left motor
    analogWrite(PWR_R, constrain(speed,             0, 255));
    analogWrite(PWR_L, constrain(speed + 3 + correction,    0, 255));
  } else {
    // Drifting left — boost right motor
    analogWrite(PWR_R, constrain(speed  - correction, 0, 255));
    analogWrite(PWR_L, constrain(speed + 3,                  0, 255));
  }

  if (debug == true){
    Serial.print("angle: ");   Serial.print(angle);
    Serial.print(" | P: ");    Serial.print(p_term);
    Serial.print(" | D: ");    Serial.print(d_term);
    Serial.print(" | corr: "); Serial.println(correction);
  }
}

//===========================THE CODE===================================================================||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

void loop() {
<<<<<<< HEAD
  
  line_left = analogRead(LINE_L) - 170; //calibrating...this one is usually a bit high than others
  line_middle = analogRead(LINE_C);
  line_right = analogRead(LINE_R);
  distance = getDistance();
  
  if (go) {
    ledOn(CRGB(50,250,50));
    forward(base_speed);
    go = false;
  }


  turn_90triger = (line_left > 600 && line_right > 600 && line_middle > 600); //trigers full 90 degree rotation when all sensors are in the tape
  line_triger = (line_left > 600 || line_right > 600); // for attitude adjustment 


//=============================Box entrance logic=================
  if (distance < 18){
    ledOn(CRGB(100,255,255));
      while (distance > 8){
        forward(100);
        distance = getDistance();
        Serial.println(distance);
      }
      stop();
      digitalWrite(MTR_ENABLE, LOW);
  }


//================Course Correction Triger Logic===========
  if (detectNewColor()){
    new_color=true;
  }

  if (turn_90triger ){
    ledOn(CRGB(100,5,70));
=======

  unsigned long ir_code = ir_read();
  if (ir_code != IR_NONE) handle_ir(ir_code);

  updateAttitude();

  line_left = analogRead(LINE_L); //calibrating...this one is usually a bit high than others
  line_middle = analogRead(LINE_C) + 100;
  line_right = analogRead(LINE_R);
  distance = getDistance();

  //white bg and black border ----- '>'
  //black bg and white border ----- '<'
  // if black the sensor high, if white sensor low
  turn_90triger = (line_left < 600 && line_right < 600); //trigers full 90 degree rotation when all sensors are in the tape
  line_triger = (line_left < 600 || line_right < 600); // for attitude adjustment 


//================Course Correction Triger Logic===========

  if ((line_triger || turn_90triger) && line_logic){
>>>>>>> ba6d1a8 (New features, Code Regactored)
    stop();
    course_corection_line();
  } 

  if(distance < 12 && sonar_logic){
    stop_graceful(base_speed, 3);
    ledOff();
    ledOn(CRGB::Blue);
    discreate_sweep();
    course_correct_sonar();
    resetAttitude();
    last_angle = 0;
    delay(500);
    ledOff();
  }

  //line_debug();
  //sonar_debug();
  //gyro_debug();

  straight_line_pd(base_speed);
  //stuck_detection();
  // //attitude_debug();
  // //delay(50);

}
//==================================================================================================|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
void discreate_sweep(){
  setServoAngle(90);
  delay(500);
  //cam_take_picture();
  delay(300);
  setServoAngle(0);
  delay(500);
  //cam_take_picture();
  delay(300);
  r_distance = getDistance();
    delay(60);
  r_distance = getDistance();
  setServoAngle(180);
  delay(500);
  //cam_take_picture();
  delay(300);
  l_distance = getDistance();
  delay(60);
  l_distance = getDistance();
  setServoAngle(90);
  delay(500);
  sonar_debug();
}

//=================Course_correction========================
void course_corection_line() {
  is_going_forward = false;

  left_or_right = random(2);  // random logic only used for custom maze....
  //better use right only turn for Original maze...alrhough random should work for both

  if (line_left < 600 && line_right < 600){ 
    if (debug){ Serial.println("===IR======TURN 90======IR===");}
    backward(50);
    delay(220);
    stop();
    delay(400);

    if (left_or_right == 0) {
      //turn_raw('l', 600, 100); //------------> optimized for 90 degree turn
      turn_raw('r', 550, 100);  //------------> Used for Custom maze
      resetAttitude();
      last_angle = 0;
    } 

    if (left_or_right == 1) {
      //turn_raw('l', 600, 100); //------------> optimized for 90 degree turn
      turn_raw('l', 550, 100); //------------> Used for Custom maze
      resetAttitude();
      last_angle = 0;
    } 
    return;
  }

  //for attitude corection 
  if (line_right < line_left && abs(line_right - line_left) > 160){
    if (debug){ Serial.println("====IR=====LEFT=====IR====");}
    //turnGyro('l', 10, 100, 50);
    turn_raw('l', 140, 90);
    resetAttitude();
    last_angle = 0;
    return;
    
  }

  if (line_left < line_right && abs(line_right - line_left) > 160){
    if (debug){ Serial.println("====IR=====RIGHT=====IR====");}
    //turnGyro('r', 10, 100, 50);
    turn_raw('r', 140, 90); 
    resetAttitude();
    last_angle = 0;
    return;

  }

}

void course_correct_sonar(){
  is_going_forward = false;
  stuck_counter = 0;
  if (r_distance < 15 && l_distance < 15){
    sonar_trapped();
    return;
  }

  if (r_distance > l_distance && abs(r_distance - l_distance) > 3){
    turnGyro('r',90, 50, 100);
    //resetAttitude();
    delay(400);
    if (debug){ Serial.println("<<<<SS=========RIGHT=========SS>>>>");}
    return;
    
  }

  if (r_distance < l_distance && abs(r_distance - l_distance) > 3){
    turnGyro('l', 90, 50, 100);
    //resetAttitude();
    delay(400);
    if (debug){ Serial.println("<<<<SS=========LEFT=========SS>>>>");}
    return;
  }
}

void sonar_trapped(){
  stop();
  backward(40);
  Serial.println("------------BACKED OFF: sonar trapped------------");
  delay(400);
  stop();
  discreate_sweep();
  course_correct_sonar();
}


void stuck_detection(){
  static float last_stuck_angle = 0;
  static unsigned long stuck_timer = 0;

  if (is_going_forward) {
    stuck_counter++;
    float angle_change = abs(attitude.angle.z - last_stuck_angle);
    last_stuck_angle = attitude.angle.z;
    Serial.println("Last angle -> " + String(last_stuck_angle) + " |Current angle -> " + String(attitude.angle.z) + " |Angle change -> " + String(angle_change) + 
    " |Stuck Counter -> " + String(stuck_counter));
    if (angle_change >= 0.2) {
      Serial.println("TIMER RESET — angle_change: " + String(angle_change, 3));
    }
    if (angle_change < 0.2 & stuck_counter > 50) {
      // Barely any rotation this tick — possible stuck
      if (millis() - stuck_timer > 4500) {
        ledOn(CRGB::DarkRed);
        Serial.println("=== STUCK DETECTED : From: loop <gyro> ===");
        stop();
        backward(base_speed);
        delay(400);
        ledOff();
        stop();
        discreate_sweep();
        course_correct_sonar();
        resetAttitude();
        stuck_timer = millis();  // reset timer after recovery
      }
    } else {
      // Rover is rotating — definitely not stuck
      stuck_timer = millis();  // keep resetting timer while moving
    }
  }
}

//============================ Sonar Sensor debug====================
void sonar_debug(){
  Serial.println("=========*****************==From: sonar_debug=======");
  Serial.print("R: ");
  Serial.print(r_distance);
  Serial.print(" | L: ");
  Serial.print(l_distance);
  // Serial.print(" | Straight: ");
  // Serial.println(straight_distance);
  Serial.println(" | distance: " + String(distance));
  Serial.println("=========*****************=========");
}

//=========================Line Sensor debug================
void line_debug(){
  Serial.println("=========*****************==From: line_debug=======");
  Serial.print("R: ");
  Serial.print(line_right);
  Serial.print(" | L: ");
  Serial.print(line_left);
  Serial.print(" | middle: ");
  Serial.println(line_middle);
  Serial.println("=========*****************=========");

}



const int IR_TURN_MS    = 100;   // ms per nudge — tune for ~15°
const int IR_TURN_SPEED = 100;   // motor speed during IR turns

void handle_ir(unsigned long code) {
  switch (code) {

    case IR_STOP:
      Serial.println("[IR] STOP");
      is_going_forward = false;
      stop();
      break;

    case IR_UP:
      Serial.println("[IR] FORWARD");
      is_going_forward = true;
      forward(base_speed);
      break;

    case IR_DOWN:
      Serial.println("[IR] BACKWARD");
      is_going_forward = false;
      backward(base_speed);
      break;

    case IR_LEFT:
      Serial.println("[IR] NUDGE LEFT");
      is_going_forward = false;
      turn_raw('l', IR_TURN_MS, IR_TURN_SPEED);
      // resetAll();
      break;

    case IR_RIGHT:
      Serial.println("[IR] NUDGE RIGHT");
      is_going_forward = false;
      turn_raw('r', IR_TURN_MS, IR_TURN_SPEED);
      // resetAll();
      break;
  }
}