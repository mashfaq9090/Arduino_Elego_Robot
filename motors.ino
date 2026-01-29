#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_AIN_1 7
#define PIN_Motor_BIN_1 8
#define PIN_Motor_STBY 3

// line tracking
#define PIN_ITR20001_LEFT A2
#define PIN_ITR20001_MIDDLE A1
#define PIN_ITR20001_RIGHT A0

void setup() {
  // put your setup code here, to run once:
  PinMode(PIN_Motor_STBY, OUTPUT);
  PinMode(PIN_Motor_PWMA, OUTPUT);
  PinMode(PIN_Motor_PWMB, OUTPUT);
  PinMode(PIN_Motor_AIN_1, OUTPUT);
  PinMode(PIN_Motor_BIN_1, OUTPUT);
  digitalWrite(PIN_Motor_STBY, HIGH);

  Serial.begin(9600);
  // Serial.print(...);
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
  delay(ms)
  stop();

}
void turn_raw(char direction, int ms=370, int speed=145 ){
  stop();
  if (direction=='l'){
    digitalWrite(PIN_Motor_AIN_1,HIGH);
    digitalWrite(PIN_Motor_BIN_1,LOW);
  }else if (direction=='r') {
    digitalWrite(PIN_Motor_AIN_1,LOW);
    digitalWrite(PIN_Motor_BIN_1,HIGH);
  }
  analogWrite(PIN_Motor_PWMA, speed);
  analogWrite(PIN_Motor_PWMB, speed);
  delay(ms);
  stop();
}
void loop() {
  // put your main code here, to run repeatedly:
  turn_raw('l');
  forward(100,200);

}
