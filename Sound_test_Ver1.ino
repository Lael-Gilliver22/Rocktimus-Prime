const int analogInPin1 = A1;
const int analogInPin3 = A3;

int sensorValue1 = 0;
int outputValue1 = 0;

int sensorValue3 = 0;
int outputValue3 = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  sensorValue1 = analogRead(analogInPin1);
  outputValue1 = map(sensorValue1, 0, 1023, 0, 255);

  sensorValue3 = analogRead(analogInPin3);
  outputValue3 = map(sensorValue3, 0, 1023, 0, 255);

  Serial.print("Sensor_1:");
  Serial.print(outputValue1);
  Serial.print(" , ");
  Serial.print("Sensor_2:");
  Serial.print(outputValue3);
  Serial.print(" , ");


  if(outputValue1 - outputValue3 < -2){
    Serial.println("Right");
  }else if(outputValue1 - outputValue3 > 2){
    Serial.println("Left");
  }else{
    Serial.println("Front");
  }

  delay(10);
}
