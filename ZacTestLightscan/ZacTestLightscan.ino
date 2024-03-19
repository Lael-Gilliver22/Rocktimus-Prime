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
int initialRed = currentRed;
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
  /*
  servoLeft.writeMicroseconds(1500 - servoRotateSpeed);
  servoRight.writeMicroseconds(1500 - servoRotateSpeed);
  
  loopUntilRedIncrease("left");
  Serial.println("LEFT DONE");
  //turn back until reaching lowest value
  loopUntilRedIncrease("right");
  Serial.println("RIGHT DONE");
  */
  //small move forward to compensate for bot backing up during rotation
  /*
  servoLeft.writeMicroseconds(1500 + servoRotateSpeed);
  servoRight.writeMicroseconds(1500 - servoRotateSpeed);
  delay (500);
  */
  spinscan();
  servoLeft.writeMicroseconds(1500 - servoRotateSpeed/2);
  servoRight.writeMicroseconds(1500 - servoRotateSpeed/2);
  facingLight = false;
  while (!facingLight) {
    currentRed = pulseIn(sensorOut, LOW);
    printReadings();
    if (currentRed <= lowestRed + 5){ //5 tolerance to prevent overshoots
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
  currentRed = pulseIn(sensorOut, LOW);
  initialRed = currentRed;
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
    printReadings();
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
    else if(currentRed > lastRed){ 
      countlow = 0;
      if (counthigh < 5){
        counthigh += 1;
      }
      else{
        counthigh = 0;
        if (currentRed >= lowestRed+(0.2*lowestRed)){
          Serial.println("Final Red");
          Serial.print(currentRed);
          facingLight = true;
        }
      }
    }
    if ((currentRed > (initialRed * 1.2)) && (direction == "left")){//going in wrong direction, will not find red light. Best to spin 90 degrees, or to break out the loop. NEEDS IMPROVING
      Serial.println("Going too far away!");
      //assume this will only happen on the first left turn, so exit the loop
      facingLight = true;
    }
    lastRed = currentRed;
  }
}

void spinscan(){
  lowestRed = 9999;
  countlow = 0;
  servoLeft.writeMicroseconds(1500 + servoRotateSpeed);
  servoRight.writeMicroseconds(1500 + servoRotateSpeed);
  unsigned long startTime = millis(); // Get the current time

  while (millis() - startTime < 8000) {

      // This will execute for 10 seconds
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  currentRed = pulseIn(sensorOut, LOW);
    //printReadings();
    if (currentRed <= lowestRed){
      if (countlow < 5){ //must be lower 3 times in a row to eliminate random jumps in value
        countlow += 1;
      }
      else{
        countlow = 0;
        lowestRed = currentRed;
        Serial.println("NEW LOWEST RED!!!!!");
        printReadings();
      }
    }
  }
  Serial.println("Turning to find red light");
  while (currentRed != lowestRed) {
    currentRed = pulseIn(sensorOut, LOW);
  }
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
}

void printReadings(){
  Serial.print("currentRed");
  Serial.print(currentRed);
  Serial.print(" LowestRed");
  Serial.print(lowestRed);
  Serial.print(" countLow");
  Serial.print(countlow);
  Serial.print(" countHigh");
  Serial.println(counthigh);
  Serial.println("");
}