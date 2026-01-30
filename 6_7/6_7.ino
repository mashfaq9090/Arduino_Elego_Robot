#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_AIN_1 7
#define PIN_Motor_BIN_1 8
#define PIN_Motor_STBY 3
#define DATA_PIN 4 // std pin for led

// line tracking
#define PIN_SENSOR_LEFT A2
#define PIN_SENSOR_MIDDLE A1
#define PIN_SENSOR_RIGHT A0

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_Motor_STBY, OUTPUT);
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_SENSOR_LEFT, INPUT);
  pinMode(PIN_SENSOR_MIDDLE, INPUT);
  pinMode(PIN_SENSOR_RIGHT, INPUT);
  digitalWrite(PIN_Motor_STBY, HIGH);

  Serial.begin(9600);
}
void stop(){
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0); 
}

void forward(int speed, int ms){
  stop();
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, speed);
  analogWrite(PIN_Motor_PWMB, speed);
  delay(ms);
  stop();

}

void turn_raw(char direction, int ms=370, int speed=145, int ratio=1){
  stop();
  if (direction=='l'){
    digitalWrite(PIN_Motor_AIN_1,HIGH);
    digitalWrite(PIN_Motor_BIN_1,LOW);
    analogWrite(PIN_Motor_PWMA, speed);
    analogWrite(PIN_Motor_PWMB, speed/ratio);  
  }else if (direction=='r') {
    digitalWrite(PIN_Motor_AIN_1,LOW);
    digitalWrite(PIN_Motor_BIN_1,HIGH);
    analogWrite(PIN_Motor_PWMA, speed/ratio);
    analogWrite(PIN_Motor_PWMB, speed);
  }
  delay(ms);
  stop();
}
void turn_arc(char direction, int ms=370, int speed=145, int ratio=1){
  stop();
  if (direction=='l'){
    digitalWrite(PIN_Motor_AIN_1,HIGH);
    digitalWrite(PIN_Motor_BIN_1,HIGH);
    analogWrite(PIN_Motor_PWMA, speed);
    analogWrite(PIN_Motor_PWMB, speed/ratio);  
  }else if (direction=='r') {
    digitalWrite(PIN_Motor_AIN_1,HIGH);
    digitalWrite(PIN_Motor_BIN_1,HIGH);
    analogWrite(PIN_Motor_PWMA, speed/ratio);
    analogWrite(PIN_Motor_PWMB, speed);
  }
  delay(ms);
  stop();
}
int TURN_MULTIPLIER=.34;



void show_vals(){
int left = analogRead(PIN_SENSOR_LEFT);
  int middle = analogRead(PIN_SENSOR_MIDDLE);
  int right = analogRead(PIN_SENSOR_RIGHT);
  Serial.print("left ");
  Serial.println(left);
  Serial.print("mid ");
  Serial.println(middle);
  Serial.print("right ");
  Serial.println(right);
}

void follow_white() {
  int left = analogRead(PIN_SENSOR_LEFT);
  int middle = analogRead(PIN_SENSOR_MIDDLE);
  int right = analogRead(PIN_SENSOR_RIGHT);
  
  
  if (middle <500 ){
      if ( right >500) {
        //int turn_speed = (middle-left)*TURN_MULTIPLER
        turn_raw('l',40,60);
      Serial.println("right");
    }
    if ( left >500){
      turn_raw('r',40,60);
      Serial.println("left");
    }

    forward(50,20);
  }
}
void six_seven(){
  turn_arc('r', 5750,145, 3);
  delay(250);
  forward(145,1000);
  delay(250);
  turn_raw('r',260,145,1);
  delay(250);
  forward(145,900);
  delay(250);
  turn_raw('r', 425,145,1);
  delay(250);
  forward(145,1200);
  
}
void loop() {
  
 six_seven();
 digitalWrite(PIN_Motor_STBY, LOW);

}
