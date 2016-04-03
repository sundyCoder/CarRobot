
#include "pt.h"
static struct pt pt1, pt2;

static int protothread1(struct pt *pt,int time) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > time);
    Serial.println("Thread1......");
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

static int protothread2(struct pt *pt,int time) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > time);
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

void xTaskCreate(int (*xTaskName)(struct pt *,int),struct pt *ptr,int time) {  
  (* xTaskName)(ptr,time);
}

void loop() {
  // put your main code here, to run repeatedly:
  xTaskCreate(protothread1,&pt1,1000);
  xTaskCreate(protothread2,&pt2,1000);
}


