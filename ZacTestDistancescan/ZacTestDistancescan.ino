#include <Servo.h>
//--------Distance Stuff----------------
int stopDist = 48;
int slowdist = 60;
int slowSpeed = 10;
int fastSpeed = 100;
int currentSpeed = fastSpeed;
int currentDist = 0;
int lowestDist = 9999;
int lastDist = currentDist;
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X sensor;
#define HIGH_ACCURACY
bool stopped = false;
//--------------------------------------
Servo servoLeft;
Servo servoRight;
int servoRotateSpeed = 25;
#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define sensorOut 2
int tolerance = 0;
int redFrequency = 0;

int currentRed = pulseIn(sensorOut, LOW);
int lastRed = currentRed;
int initialRed = currentRed;
int lowestRed = 9999;
int countlow = 0;
int counthigh = 0;
bool facingLight = false;
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

//--------Distance Stuff----------------
  Wire.begin();
  
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  #if defined HIGH_ACCURACY
    // increase timing budget to 200 ms
    sensor.setMeasurementTimingBudget(200000);
  #endif
//--------------------------------------

  distanceScan();
}

void loop() {
  // put your main code here, to run repeatedly:

}


void distanceScan(){
  while (stopped == false){
    Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) {
     Serial.print(" TIMEOUT"); 
    }
  Serial.println();
  loopUntilDistanceIncrease("left");
  loopUntilDistanceIncrease("right");
  
}

void loopUntilDistanceIncrease(String direction){
  facingLight = false;
  currentDist = pulseIn(sensorOut, LOW);
  initialDist = currentDist;
  if (direction == "left"){
    servoLeft.writeMicroseconds(1500 - servoRotateSpeed);
    servoRight.writeMicroseconds(1500 - servoRotateSpeed);
  }
  else{
    servoLeft.writeMicroseconds(1500 + servoRotateSpeed);
    servoRight.writeMicroseconds(1500 + servoRotateSpeed);
  }
  while (!facingLight) {
    currentDist = pulseIn(sensorOut, LOW);
    printReadings();
    if (currentDist <= lowestDist){
      counthigh = 0;
      if (countlow < 5){
        countlow += 1;
      }
      else{
        countlow = 0;
        lowestDist = currentDist;
      }
    }
    else if(currentDist > lastDist){ 
      countlow = 0;
      if (counthigh < 5){
        counthigh += 1;
      }
      else{
        counthigh = 0;
        if (currentDist >= lowestDist+(0.2*lowestDist)){ //deliberately overshoot to scan values further
          Serial.println("Final Dist");
          Serial.print(currentDist);
          facingLight = true; // actually will be overshot
        }
      }
    }
    lastDist = currentDist;
    if (currentDist <= stopdist){
      currentSpeed = 0;
    }
    else if (){
      currentSpeed = slowSpeed;
    }
    else{
      currentSpeed = fastSpeed;
    }
  }
}