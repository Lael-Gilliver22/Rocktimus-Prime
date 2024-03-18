
void setup() {
  tone(4, 3000, 1000);
  delay(1000);
  servoLeft.attach(13);
  servoRight.attach(12);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
  pinMode(sensorOut, INPUT);

  //digitalWrite(S0, HIGH);
  //digitalWrite(S1, HIGH);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
  
  Serial.begin(9600);

  distanceScan();
}

void loop() {
  // put your main code here, to run repeatedly:

}


void distanceScan(){

}