#include "SR04.h"
#include "pt.h"

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
#define SOUND1_ECHO_PIN 5
#define DANGER  40

/************light sensor********************/
#define LIGHT1_AO   0
#define LIGHT2_A1   1
#define LIGHT_ERROR 15

#define THRESHOLD 50
#define L_RIGHT    1
#define L_LEFT     2
#define L_FORWARD  3

static struct pt pt1, pt2;

// variables:
int sensorValue = 0;      // the sensor value
int sensorValue1 = 0;     // the sensor value

int sensorMin = 0;        // minimum sensor value
int sensorMax = 1023;     // maximum sensor value

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
  RightSpeed += value;
  analogWrite(MOTOR1_PWM, LeftSpeed);
  analogWrite(MOTOR2_PWM, RightSpeed);
}

int GetLightSensorValue(int SenID) {
  int SensorValue = analogRead(SenID);
  if (SenID == 0) {
    SensorValue = map(SensorValue, sensorMin, sensorMax, 0, 250);
  } else if (SenID == 1) {
    SensorValue = map(SensorValue, sensorMin, sensorMax, 0, 250);
  }
  //  Serial.print(SenID + ":");
  //  Serial.println(SensorValue);

  SensorValue = (SensorValue)/ 2 + 100;
  return SensorValue;
}

SR04 ReadSound1Dist() {
  return SR04(SOUND1_ECHO_PIN, SOUND1_TRIG_PIN);
}

bool ObsInFront() {
  long distance = ReadSound1Dist().Distance();
  Serial.print(distance);
  Serial.println("cm");
  if ((distance < DANGER) && (distance != 0))
    return true;
  else
    return false;
}

static int AvoidCollision(struct pt *pt, int time) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > time);
    Serial.println("Thread1......");
    bool NotSafe = ObsInFront();
    while (NotSafe) {
      SetMotorSpeed(90);
      MotorCtl(TURNRIGHT);
      delay(100);
      MotorCtl(STOP);
      NotSafe = ObsInFront();
      delay(100);
    }
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

static int SeekLight(struct pt *pt, int time) {
  static unsigned long timestamp = 0;
  MotorCtl(FORWARD);
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > time);
    Serial.println("Thread2:");
    int LeftSensor = GetLightSensorValue(LIGHT1_AO);
    int RightSensor = GetLightSensorValue(LIGHT2_A1);
        Serial.print(LeftSensor);
        Serial.print(":");
        Serial.println(RightSensor);

    int LeftMotor = 255 - LeftSensor ;
    int RightMotor = 255 - RightSensor + 8;
    if ((LeftMotor <= 50) && (RightMotor <= 50)) {
      LeftMotor += 50;
      RightMotor += 50;
    }
    Serial.println(LeftMotor);
    Serial.println(RightMotor);
    analogWrite(MOTOR1_PWM, LeftMotor);
    analogWrite(MOTOR2_PWM, RightMotor);
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

static int noSeekSource(struct pt *pt, int time){
  static unsigned long timestamp = 0;
  MotorCtl(FORWARD);
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > time);
    Serial.println("Thread2:");
    analogWrite(MOTOR1_PWM, 80);
    analogWrite(MOTOR2_PWM, 80);
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

void xTaskCreate(int (*xTaskName)(struct pt *, int), struct pt *ptr, int time) {
  (* xTaskName)(ptr, time);
}

void TaskInit() {
  PT_INIT(&pt1);
  PT_INIT(&pt2);
}

void setup() {
  // put your setup code here, to run once:
  UartInit();
  MotorInit();
  TaskInit();
}

void loop() {
  // put your main code here, to run repeatedly:
  xTaskCreate(AvoidCollision, &pt1, 300);
  xTaskCreate(noSeekSource, &pt2, 500);
}


