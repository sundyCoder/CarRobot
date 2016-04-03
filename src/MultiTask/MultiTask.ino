
#include "pt.h"
static struct pt pt1, pt2;

static int protothread1(struct pt *pt) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > 500);
    Serial.println("Thread1......");
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

static int protothread2(struct pt *pt) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > 1000);
    Serial.println("Thread2......");
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  PT_INIT(&pt1);
  PT_INIT(&pt2);
}

void loop() {
  // put your main code here, to run repeatedly:
    protothread1(&pt1);
    protothread2(&pt2);
}


