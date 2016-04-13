#include "SR04.h"
#include "pt.h"

#define FORWARD   1
#define BACKWARD  2
#define TURNRIGHT 3
#define TURNLEFT  4
#define STOP      5

/************step motor**********************/
#define LEFT1_WHEEL1  7
#define LEFT2_WHEEL1  8
#define RIGHT1_WHEEL2 12
#define RIGHT2_WHEEL2 13
#define MOTOR1_PWM    10
#define MOTOR2_PWM    11

/************UtraSonic sensor****************/
//Three sound sensor shsare the same triger pin of digital pin2
#define SOUND_TRIG_PIN 2
//left UtraSonic Sensor
#define SOUND1_ECHO_PIN 3
//Front UtraSonic Sensor
#define SOUND2_ECHO_PIN 5
//Right UtraSonic Sensor
#define SOUND3_ECHO_PIN 9

#define DANGER_LONG  40
#define DANGER_SHORT 20
#define DANGER_MID   30
#define DELTA        5

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

void RunStraight() {
  int distFront = ReadSoundDist(2);
  int distLeft =  ReadSoundDist(1);
  int distRight =  ReadSoundDist(3);

  MotorCtl(FORWARD);

  if (distFront > DANGER_SHORT) {
    if (DANGER_SHORT < distRight && distRight < DANGER_MID) {
      analogWrite(MOTOR1_PWM, 60);
      analogWrite(MOTOR2_PWM, 50);
    }

    if (distRight > DANGER_MID) {
      analogWrite(MOTOR1_PWM, 60);
      analogWrite(MOTOR2_PWM, 40);
    }

    if (distRight < DANGER_SHORT) {
      analogWrite(MOTOR1_PWM, 40);
      analogWrite(MOTOR2_PWM, 60);
    }
  }
  delay(800);
}


void RunLeft() {
  MotorCtl(TURNLEFT);
  SetMotorSpeed(80);

  delay(400);
}

void RunRight() {
  MotorCtl(TURNRIGHT);
  SetMotorSpeed(60);

  delay(300);
}

void RunBack() {
  MotorCtl(BACKWARD);
  SetMotorSpeed(60);
  delay(400);
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

  SensorValue = (SensorValue) / 2 + 100;
  return SensorValue;
}

SR04 ReadSound1Dist(int id) {
  switch (id) {
    case 1:
      return SR04(SOUND1_ECHO_PIN, SOUND_TRIG_PIN);
      break;

    case 2:
      return SR04(SOUND2_ECHO_PIN, SOUND_TRIG_PIN);
      break;

    case 3:
      return SR04(SOUND3_ECHO_PIN, SOUND_TRIG_PIN);
      break;
  }
}

long ReadSoundDist(int id) {
  long distance = 0;
  switch (id) {
    case 1:
      return ReadSound1Dist(1).Distance();
      break;

    case 2:
      return ReadSound1Dist(2).Distance();
      break;

    case 3:
      return ReadSound1Dist(3).Distance();
      break;
  }
}

void UltraSonicTest() {
  long distance1 = ReadSound1Dist(1).Distance();
  long distance2 = ReadSound1Dist(2).Distance();
  long distance3 = ReadSound1Dist(3).Distance();
  Serial.print(distance1);  Serial.print(":");
  Serial.print(distance2);  Serial.print(":");
  Serial.print(distance3);  Serial.println("");
  delay(600);
}

bool ObsOnLeft() {
  long distLeft = ReadSoundDist(1);
  long distRight = ReadSoundDist(3);
  if ((distLeft + 10) < distRight)
    return true;
  return false;
}

bool ObsInFront() {
  long distance = ReadSoundDist(2);
  Serial.print(distance);
  Serial.println("cm");
  if ((distance < DANGER_LONG) && (distance != 0))
    return true;
  else
    return false;
}

bool ObsOnRight() {
  long distLeft = ReadSoundDist(1);
  long distRight = ReadSoundDist(3);
  if ((distRight + 10) < distLeft)
    return true;
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
//      
//      MotorCtl(TURNRIGHT);
//      delay(100);

      if (ObsOnLeft()) {
        MotorCtl(TURNRIGHT);
      }

      if (ObsOnRight()) {
        MotorCtl(TURNLEFT);
      }

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
      LeftMotor += 70;
      RightMotor += 70;
    }
    Serial.println(LeftMotor);
    Serial.println(RightMotor);
    analogWrite(MOTOR1_PWM, LeftMotor);
    analogWrite(MOTOR2_PWM, RightMotor);
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

void xTaskCreate(int (*xTaskName)(struct pt *, int), struct pt *ptr, int time) {
  (* xTaskName)(ptr, time);
}

void TaskInit() {
  PT_INIT(&pt1);
//  PT_INIT(&pt2);
}

void setup() {
  // put your setup code here, to run once:
  UartInit();
  MotorInit();
  TaskInit();
}

void loop() {
//  RunStraight();
//  RunLeft();
//  RunRight();
//  RunBack();
  //  RunBack();
  // put your main code here, to run repeatedly:
    xTaskCreate(AvoidCollision, &pt1, 300);
    xTaskCreate(SeekLight, &pt2, 500);
}


