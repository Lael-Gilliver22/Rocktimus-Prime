#include <Servo.h>

Servo servoLeft;
Servo servoRight;
int servoRotateSpeed = 30;
#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define sensorOut 2
int tolerance = 0;
int redFrequency = 0;

int currentRed = pulseIn(sensorOut, LOW);
int lastRed = currentRed;
int lowestRed = 9999;
int countlow = 0;
int counthigh = 0;
bool facingLight = false;

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

  //digitalWrite(S0, HIGH);
  //digitalWrite(S1, HIGH);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
  
  Serial.begin(9600);

  lightScan();
}

void loop() {}

void lightScan() {
  
  servoLeft.writeMicroseconds(1500 - servoRotateSpeed);
  servoRight.writeMicroseconds(1500 - servoRotateSpeed);
  
  loopUntilRedIncrease("left");
  //turn back until reaching lowest value
  loopUntilRedIncrease("right");

  servoLeft.writeMicroseconds(1500 - servoRotateSpeed);
  servoRight.writeMicroseconds(1500 - servoRotateSpeed);
  facingLight = false;
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

void distanceScan(){

}


void loopUntilRedIncrease(String direction){
  facingLight = false;
  if (direction == "left"){
    servoLeft.writeMicroseconds(1500 - servoRotateSpeed);
    servoRight.writeMicroseconds(1500 - servoRotateSpeed);
  }
  else{
    servoLeft.writeMicroseconds(1500 + servoRotateSpeed);
    servoRight.writeMicroseconds(1500 + servoRotateSpeed);
  }
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
    else if(currentRed > lowestRed){ 
      countlow = 0;
      if (counthigh < 5){
        counthigh += 1;
      }
      else{
        counthigh = 0;
        if (currentRed >= lowestRed+10){
          Serial.println("Final Red");
          Serial.print(currentRed);
          facingLight = true;
        }
      }
    }
    if (currentRed > 600){//going in wrong direction, will not find red light. Best to spin 90 degrees, or to break out the loop
      Serial.println("Going too far away!");
    }
  }
}