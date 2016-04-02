
const int sensorPin = A0;
const int sensorPin1 = A1;

// variables:
int sensorValue = 0;         // the sensor value
int sensorValue1 = 0;         // the sensor value

int sensorMin = 0;        // minimum sensor value
int sensorMax = 1023;           // maximum sensor value

void setup() {
  Serial.begin(9600);
}

void loop() {
  // read the sensor:
  sensorValue = analogRead(sensorPin);
  sensorValue1 = analogRead(sensorPin1);

  // apply the calibration to the sensor reading
  sensorValue = map(sensorValue, sensorMin, sensorMax, 0, 255) + 15;
  sensorValue1 = map(sensorValue1, sensorMin, sensorMax, 0, 255);
  sensorValue = (sensorValue)/2;
  sensorValue1 = (sensorValue1)/2;
  Serial.println();
  Serial.print("Light1:");
  Serial.println(sensorValue1);
  Serial.print("Light:");
  Serial.println(sensorValue);
  
  delay(30);
}

