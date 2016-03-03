/*
 * Author:  Sundy 
 * TIME :   2 March,2016
 * Subject: COMP5228 ESI
**/
#include "SR04.h"
#define FORWARD   1
#define BACKWARD  2
#define TURNRIGHT 3
#define TURNLEFT  4
#define STOP     5

/************step motor**********************/
#define LEFT1_WHEEL1  7
#define LEFT2_WHEEL1  8 
#define RIGHT1_WHEEL2 12
#define RIGHT2_WHEEL2 13
#define MOTOR1_PWM    10
#define MOTOR2_PWM    11

/************UtraSonic sensor****************/
#define SOUND1_TRIG_PIN 2
#define SOUND1_ECHO_PIN 3

/************light sensor********************/
#define LIGHT1_AO   0
#define LIGHT2_A1   1

#define THRESHOLD 50
#define L_RIGHT    1
#define L_LEFT     2
#define L_FORWARD  3

/******general function********/
SR04 ReadSound1Dist() {
  return SR04(SOUND1_ECHO_PIN, SOUND1_TRIG_PIN);
}

long ReadLight1Analog(int lightID) {
  if (lightID == 0) {
    return analogRead(LIGHT1_AO) + 66;
  } else if (lightID == 1) {
    return analogRead(LIGHT2_A1);
  }
}

void UartInit() {
  Serial.begin(9600);
}

void MotorInit() {
  /************step motor**********************/
  pinMode(LEFT1_WHEEL1, OUTPUT);
  pinMode(LEFT2_WHEEL1, OUTPUT);
  pinMode(RIGHT1_WHEEL2, OUTPUT);
  pinMode(RIGHT2_WHEEL2, OUTPUT);

  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
}

void MotorCtl(int direction) {
  /**************motor test***********************/
  switch (direction) {
    case FORWARD:
      //forward
      digitalWrite(7, HIGH);
      digitalWrite(8, LOW);
      digitalWrite(12, HIGH);
      digitalWrite(13, LOW);
      break;
    case BACKWARD:
      //backward
      digitalWrite(7, LOW);
      digitalWrite(8, HIGH);
      digitalWrite(12, LOW);
      digitalWrite(13, HIGH);
      break;
    case TURNRIGHT:
      //turn right
      digitalWrite(7, HIGH);
      digitalWrite(8, LOW);
      digitalWrite(12, LOW);
      digitalWrite(13, HIGH);
      break;
    case TURNLEFT:
      //turn left
      digitalWrite(7, LOW);
      digitalWrite(8, HIGH);
      digitalWrite(12, HIGH);
      digitalWrite(13, LOW);
      break;
    case STOP:
      //stop
      digitalWrite(7, LOW);
      digitalWrite(8, LOW);
      digitalWrite(12, LOW);
      digitalWrite(13, LOW);
      break;
  }
}


void SetMotorSpeed(int value) {
  int LeftSpeed = 0;
  int RightSpeed = 0;
  LeftSpeed += value;
  RightSpeed += value + 5;
  analogWrite(MOTOR1_PWM, LeftSpeed);
  analogWrite(MOTOR2_PWM, RightSpeed);
}


void MotorTest() {
  MotorCtl(FORWARD);
  delay(4000);
  MotorCtl(BACKWARD);
  delay(4000);
  MotorCtl(TURNRIGHT);
  delay(4000);
  MotorCtl(TURNLEFT);
  delay(4000);
  MotorCtl(STOP);
  delay(4000);
}

void UtraSonicTest() {
  int distance = ReadSound1Dist().Distance();
  Serial.print(distance);
  Serial.println("cm");
  delay(1000);
}

long UtraSoundTest() {
  digitalWrite(SOUND1_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SOUND1_TRIG_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SOUND1_TRIG_PIN, LOW);
  long distance = pulseIn(SOUND1_ECHO_PIN, HIGH);
  distance = distance / 29 / 2;
}

void LightSensorTest() {
  int LightValue1 = ReadLight1Analog(0); //读取模拟接口的值
  int LightValue2 = ReadLight1Analog(1); //读取模拟接口的值
  Serial.print("Light1:");
  Serial.println(LightValue1);         //输出模拟接口的值
  Serial.print("Light2:");
  Serial.println(LightValue2);         //输出模拟接口的值
  delay(1000);
}

bool ObsInFront() {
  long distance = ReadSound1Dist().Distance();
  Serial.print(distance);
  Serial.println("cm");
  if ((distance < 50))
    return true;
  else
    return false;
}

void TurnToSafe() {
  while (ObsInFront()) {
    SetMotorSpeed(100);
    MotorCtl(TURNRIGHT);
  }
}

void setup() {
  // put your setup code here, to run once:
  UartInit();
  MotorInit();
  /************UtraSonic sensor****************/


  /************Light Sensor********************/
}

int LightDirection() {
  //LightSensorTest();
  if (ReadLight1Analog(1) > (ReadLight1Analog(0)  + THRESHOLD)) {
    return L_RIGHT;
  } else if (ReadLight1Analog(0) > (ReadLight1Analog(1)  + THRESHOLD)) {
    return L_LEFT;
  } else if ((ReadLight1Analog(1) - 10) < ReadLight1Analog(0) < (ReadLight1Analog(1)  + 10)) {
    return L_FORWARD;
  } else if ((ReadLight1Analog(0) - 10) < ReadLight1Analog(1) < (ReadLight1Analog(0)  + 10)) {
    return L_FORWARD;
  }
  return 0;
}

int TurnAround() {
  int dir = LightDirection();
  switch (dir) {
    case 1:
      MotorCtl(TURNRIGHT);
      delay(150);
      break;
    case 2:
      MotorCtl(TURNLEFT);
      delay(150);
      break;
  }
  return dir;
}

  void FollowLight() {
  int dir = LightDirection();
  switch (dir) {
    case 1:
      MotorCtl(TURNRIGHT);
      SetMotorSpeed(100);
      delay(100);
      break;
    case 2:
      MotorCtl(TURNLEFT);
      SetMotorSpeed(100);
      delay(100);
      break;
    case 3:
      MotorCtl(FORWARD);
      SetMotorSpeed(100);
      delay(400);
      break;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  bool NotSafe = ObsInFront();
  while (NotSafe) {
    SetMotorSpeed(90);
    MotorCtl(TURNRIGHT);
    delay(100);
    MotorCtl(STOP);
    NotSafe = ObsInFront();
    delay(100);
  }
  FollowLight();
}

