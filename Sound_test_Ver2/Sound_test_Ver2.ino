const int analogInPin = A0;

int sensorValue = 0;
int outputValue = 0;

int lastValues[25];
int count = 0;
int i = 0;
int averageValue = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for (i=0; i<25; i++){
    sensorValue = analogRead(analogInPin);
    outputValue = map(sensorValue, 0, 1023, 0, 255);
    lastValues[i] = outputValue;
  }
  averageValue = findAverage(lastValues);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorValue = analogRead(analogInPin);
  outputValue = map(sensorValue, 0, 1023, 0, 255);
  averageValue = findAverage(lastValues);

  Serial.print("Sensor:");
  Serial.print(outputValue);
  Serial.print(" , averageValue:");
  Serial.println(averageValue);

  /*if(outputValue > averageValue - 2 && outputValue < averageValue + 2){
    lastValues[count] = outputValue;
  }*/

  lastValues[count] = outputValue;

  count = count + 1;
  if(count == 25){
    count = count % 25;
  }
}

int findAverage(int arr[25]){
  int result = 0;
  int sum = 0;
  for (i=0;i<25;i++){
    sum = arr[i];
  }
  result = sum;
  return result;
}
