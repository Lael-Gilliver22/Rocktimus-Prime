#include <Servo.h>

Servo servoLeft;
Servo servoRight;
int servoRotateSpeed = 20;
#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define sensorOut 2
int tolerance = 0;
int redFrequency = 0;

void lightScan();

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

  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
  
  Serial.begin(9600);

  lightScan();
}

void loop() {}

void lightScan() {
  int currentRed = pulseIn(sensorOut, LOW);
  int lastRed = currentRed;
  int lowestRed = 9999;
  int countlow = 0;
  int counthigh = 0;
  bool facingLight = false;
  
  servoLeft.writeMicroseconds(1500 - servoRotateSpeed);
  servoRight.writeMicroseconds(1500 - servoRotateSpeed);
  
  while (!facingLight) {
    currentRed = pulseIn(sensorOut, LOW);
    Serial.print("currentRed");
    Serial.print(currentRed);
    Serial.print("LowestRed");
    Serial.print(lowestRed);
    Serial.print("countLow");
    Serial.print(countlow);
    Serial.print("countHigh");
    Serial.println(counthigh);
    Serial.println("");
    if (currentRed <= lowestRed){
      counthigh = 0;
      if (countlow < 5){
        countlow += 1;
      }
      else{
        countlow = 0;
        lowestRed = currentRed;
      }
    }
    else if(currentRed > lowestRed){ //-1 tolerance
      countlow = 0;
      if (counthigh < 5){
        counthigh += 1;
      }
      else{
        counthigh = 0;
        facingLight = true;
      }
    }
  }
  //turn back until reaching lowest value
  facingLight = false;
  servoLeft.writeMicroseconds(1500 + servoRotateSpeed);
  servoRight.writeMicroseconds(1500 + servoRotateSpeed);
  while (!facingLight) {
    currentRed = pulseIn(sensorOut, LOW);
    if (currentRed <= lowestRed){
      facingLight = true;
    }
  }

  delay(500);
  servoLeft.writeMicroseconds(1500 + 3*servoRotateSpeed);
  servoRight.writeMicroseconds(1500 - 3*servoRotateSpeed);
  delay(2000);
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
}
