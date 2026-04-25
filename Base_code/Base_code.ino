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
#include "motor_music.h" 

//=============================User Variable============================

bool go = true;
//-----------Use only when you want to globally contro the turning parameter---------------
// int turn_speed = 200;
// int turn_duration = 120;
// int turn_gyro_speed = 50;
//----------------------------------------------------------------------------------
// bool sonar_triger = false; //use only when implementing sonar based navigation
bool line_triger = false;
bool turn_90triger = false;
int line_right = 0;
int line_left = 0;
int line_middle = 0;
uint8_t base_speed = 60;
//int distance = 10000;
uint8_t left_or_right = 0;
int r_distance, straight_distance, l_distance = 10000; 
// int angle_array[] = {90, 40, 70, 90, 90, 90, 140, 170};
bool is_going_forward = false; //
uint8_t stuck_counter = 0;
bool debug = false;
bool line_logic = true; //-------------------------------------------- 
bool sonar_logic = true; //-------------------------------------------
bool remote_stop = false; //----- Should be false
bool stuck_logic_on = false; //----Should be false
uint8_t toggle = 0;
// bool line_back_and_forth = false;
bool white_follow = false;
// int sonar_trap_count = 0;
uint8_t state_swapped = 0;
bool first_maze = true;
bool second_maze = false;
int first_maze_counter = 0;
//==========================================================================================================================================
//===========================================================================================================================================

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
  ledOn(CRGB::DarkGreen);

  delay(500);

  // Initialize Gyro - hard stop if failed 

  if (!setupGyro() && gyro_using) {
    ledOn(CRGB::Red);
    while (true);  // Hard stop
  }

  calibrateGyro();
  ir_init();

  ledOff();

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
 *   speed     — cruising speed 
 *   stop_dist — distance in cm to stop at
 */
void forward_to_obstacle(int speed, int stop_dist) {
  const int SLOW_DIST = 40; // cm — begin slowing below this distance

  while (true) {
    int dist = getDistance();

    // Stop condition 
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

    delay(50); 
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


/*
 * ramp_adjusted_speed()
 * Returns boosted speed when rover is on an incline
 */

const float RAMP_THRESHOLD = 5.0;   // degrees — ignore small tilts
const float RAMP_BOOST     = 3.0;   // extra PWM per degree of incline
const int   RAMP_MAX_BOOST = 100;    // max extra speed added

int ramp_adjusted_speed(int base) {
  float pitch = attitude.angle.y;
  float tilt  = abs(pitch);

  if (tilt < RAMP_THRESHOLD) return base;  // flat ground — no boost

  int boost = constrain((int)((tilt - RAMP_THRESHOLD) * RAMP_BOOST),
                         0, RAMP_MAX_BOOST);

  // Serial.print("[RAMP] pitch: "); Serial.print(pitch, 1);
  // Serial.print("° | boost: +"); Serial.println(boost);

  return constrain(base + boost, 0, 255);
}


/*
 * straight_line_pd():
 *   PD controller for gyro-guided straight driving
 *
 *   P term — corrects based on current angle 
 *   D term — corrects based on rate of change 
 *
 *   Tune KP first 
 */

// PD tuning constants — adjust these
const float KP = 8;   // proportional gain 
const float KD = 0.8;   // derivative gain  
float last_angle = 0; 

void straight_line_pd(int speed, bool debug = false) {
  updateAttitude();
  float angle = -(attitude.angle.z);
  ledOn(CRGB::Green);
  is_going_forward = true;

  float p_term = angle * KP;
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
  ledOff();
}

//===========================THE CODE================================|||||||||||||||||||||||||=========================================||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//===========================THE CODE================================|||||||||||||||||||||||||=========================================||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
void loop() {

  unsigned long ir_code = ir_read();
  if (ir_code != IR_NONE) handle_ir(ir_code);

  updateAttitude();

  line_left = analogRead(LINE_L); 
  line_middle = analogRead(LINE_C) + 100; //calibrating...this one is usually a bit high than others
  line_right = analogRead(LINE_R);
  straight_distance = getDistance();

  //white bg and black border ----- '>' ----trigers when see black
  //black bg and white border ----- '<' ----trigers when see white
  // if black the sensor high, if white sensor low

  if (line_left < 600 && line_right < 600){    
    white_follow = true;
    first_maze = false;
    second_maze = true;
    sonar_logic = false;
    //Serial.println("---###--White Line Follow Logic HIT----------");
    base_speed = 33;
  }

  if(white_follow == true){
    turn_90triger = (line_left > 600 && line_right > 600); // ---- trigers when see black
    line_triger = (line_left > 600 || line_right > 600);  // ---- trigers when see black
  }

  if (white_follow == true && (line_left > 600 && line_right > 600)){
    stop();
    delay(100);
    sonar_logic = true;
    delay(100);
    white_follow = false;
    line_triger = false;
    turn_90triger = false;
    state_swapped = state_swapped + 1;
    base_speed = 60;
    second_maze = true;
    if (state_swapped == 2){
      forward(40);
      delay(60);
      for (int c = 0; c < 10; c++){
        ledOff();
        delay(50);
        ledOn(CRGB::DeepSkyBlue);
      }
      stop();
      play_mario_complete();
      stop();
      digitalWrite(MTR_ENABLE, LOW);

    }
  }

//================Course Correction Triger Logic===========

  if ((line_triger || turn_90triger) && line_logic){
    stop();
    course_corection_line();
  } 

  // Fatal error...sensor disconencted....l0gic
  if(straight_distance == 0){
    stop();
    digitalWrite(MTR_ENABLE, HIGH);
    for (int c = 0; c < 5; c++){
      ledOff();
      delay(50);
      ledOn(CRGB::Red1);
    }
    while (digitalRead(BUTTON) == HIGH) {

    }
    ledOff();
  }


  if (first_maze == true) {

    straight_line_pd(50);
    delay(400);
    stop();

    discreate_sweep();

    // center servo after sweep
    setServoAngle(85);
    delay(60);

    int diff = r_distance - l_distance;

    // deadband check first
    if (abs(diff) <= 12) return;

    // move forward once (shared)
    forward(65);
    delay(700);
    stop();
    delay(50);

    // decide direction
    char turn_dir = (diff > 0) ? 'r' : 'l';
    turnGyro(turn_dir, 89, 65, 100);

    resetAttitude();
    last_angle = 0;

    delay(500);
  }

  if(second_maze == true) {straight_line_pd(ramp_adjusted_speed(base_speed));}

  if(straight_distance < 5 && sonar_logic && second_maze == true){
    stop();
    discreate_sweep();
    resetAttitude();
    last_angle = 0;
    if (second_maze == true){
      bool r_open = r_distance > 13;
      bool l_open = l_distance > 13;
      if (r_open && l_open) {
          //prefer left
        forward(35);
        delay(200);
        turnGyro('l', 90, 60, 100);
      } else {
        uint8_t turned = course_correct_sonar();
      }
    resetAttitude();
    last_angle = 0;
    delay(500);
   }

  }
}
//==============================================================|||||||||||||||||||||||||===============================||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//==============================================================|||||||||||||||||||||||||===============================||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

void discreate_sweep(){
  setServoAngle(85);
  delay(500);
  //cam_take_picture();
  setServoAngle(0);
  delay(500);
  //cam_take_picture();
  r_distance = getDistanceReliable();
  //   delay(60);
  // r_distance = getDistance();
  setServoAngle(180);
  delay(500);
  //cam_take_picture();
  l_distance = getDistanceReliable();
  // delay(60);
  // l_distance = getDistance();
  setServoAngle(85);
  delay(500);
}

//=================Course_correction========================-----------------------------------------------------------------------------------
void course_corection_line() {
  is_going_forward = false;

  left_or_right = random(2);  // random logic only used for custom maze....
  //better use right only turn for Original maze...alrhough random should work for both
  if (!white_follow){
    if (line_left < 600 && line_right < 600){ 
      ledOn(CRGB::Yellow);
      //if (debug){ Serial.println("===IR======TURN 90======IR===");}
      stop();
      backward(base_speed);
      delay(220);
      stop_graceful(base_speed, 5);

      resetAttitude();
      last_angle = 0;
      delay(400);
      ledOff();

      if (!sonar_logic){
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
        
      } else {
        discreate_sweep();
        course_correct_sonar();
        return;
      }
    }

    //for attitude corection 
    if (line_right < line_left && abs(line_right - line_left) > 160){
      ledOn(CRGB::White);
      //if (debug){ Serial.println("====IR=====LEFT=====IR====");}
      //turnGyro('l', 10, 100, 50);
      turn_raw('l', 100, 50);
      resetAttitude();
      last_angle = 0;
      ledOff();
      return;
      
    }

    if (line_left < line_right && abs(line_right - line_left) > 160){
      ledOn(CRGB::White);
      //if (debug){ Serial.println("====IR=====RIGHT=====IR====");}
      //turnGyro('r', 10, 100, 50);
      turn_raw('r', 100, 50); 
      resetAttitude();
      last_angle = 0;
      ledOff();
      return;

    }
  }

  if (white_follow){

    if (line_right > line_left && abs(line_right - line_left) > 160){
      ledOn(CRGB::White);
      //if (debug){ Serial.println("====IR=====LEFT=====IR====");}
      //turnGyro('l', 10, 100, 50);
      turn_raw('l', 100, 55);
      resetAttitude();
      last_angle = 0;
      ledOff();
      return;
      
    }

    if (line_left > line_right && abs(line_right - line_left) > 160){
      ledOn(CRGB::White);
      //if (debug){ Serial.println("====IR=====RIGHT=====IR====");}
      //turnGyro('r', 10, 100, 50);
      turn_raw('r', 100, 55); 
      resetAttitude();
      last_angle = 0;
      ledOff();
      return;
    }
  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------

uint8_t course_correct_sonar() {
    if (r_distance <15 && l_distance <15 ) {
        //sonar_trapped();
        return;
    }
    if (r_distance > 8 && r_distance >= l_distance) {
        turnGyro('r', 90, 60, 100);
        return;
    }
    if (l_distance > 8) {
        turnGyro('l', 90, 60, 100);
        return ;
    }
    return;
}

void sonar_trapped(){
  stop();
  //sonar_trap_count = sonar_trap_count + 1;
  ledOn(CRGB::DeepPink);
  backward(base_speed);
  //Serial.println("------------BACKED OFF: sonar trapped------------");
  delay(400);
  stop();
  resetAttitude();
  last_angle = 0;
  //discreate_sweep();
  ledOff();
  //course_correct_sonar();
  stop();
  turnGyro('r', 90, 50, 100);
  resetAttitude();
  last_angle = 0;
  turnGyro('r', 90, 50, 100);
  resetAttitude();
  last_angle = 0;
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
  Serial.println(" | distance: " + String(straight_distance));
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
      remote_stop = false;
      stop();
      resetAttitude();
      last_angle = 0;
      toggle ++;
      break;

    case IR_UP:
      Serial.println("[IR] FORWARD");
      is_going_forward = true;
      forward(base_speed);
      delay(1000);
      stop();
      break;

    case IR_DOWN:
      Serial.println("[IR] BACKWARD");
      is_going_forward = false;
      backward(base_speed);
      delay(1000);
      stop();
      break;

    case IR_LEFT:
      Serial.println("[IR] NUDGE LEFT");
      is_going_forward = false;
      turn_raw('l', IR_TURN_MS, IR_TURN_SPEED);
      resetAttitude();
      last_angle = 0;
      break;

    case IR_RIGHT:
      Serial.println("[IR] NUDGE RIGHT");
      is_going_forward = false;
      turn_raw('r', IR_TURN_MS, IR_TURN_SPEED);
      resetAttitude();
      last_angle = 0;
      break;

    case IR_Sound_1:
      play_mario_complete();
      last_angle = 0;
      break;

    case IR_Sound_2:
      sound2();
      last_angle = 0;
      break;

    case KEY_star:
      stop();
      resetAttitude();
      last_angle = 0;
      stuck_logic_on = true;
      break;
  }
}