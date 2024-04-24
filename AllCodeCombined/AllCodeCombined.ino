//Servo
#include <Servo.h>
int speedMultiplier = 1; //set to 0.5 to slow to half speed when close to the end
Servo servoLeft;
Servo servoRight;
int servoRotateSpeed = 30;
//--------------------------------------

//Other
const int activesoundSystem = 0;
const int activelightSystem = 1;
const int activestopSystem = 2;
int activeSystem = activesoundSystem; //0 is sound, 1 is light, 3 is finish
//--------------------------------------

//LED
const int redPin = 9;
const int greenPin = 10;
const int yellowPin = 11;
//--------------------------------------

//Distance sensor
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X sensor;
#define HIGH_ACCURACY
const int stopDist = 45;
const int slowDist = 80;
int reasonableDistance = 100; //Ignores light readings when distance sensor reads higher than this. Should update to be LowestDistance + some number
const int activateLightDistance = 100; //will switch to sound subsystem within this distance and when a large enough red light reading (activateLightLightread)
int lowestDist = 99999;
int currentDist = 0;
//--------------------------------------

//Sound sensors
const int analogLeftPin = A0;   // Analog input pin for the left sensor
const int analogRightPin = A1;  // Analog input pin for the right sensor
const int analogBackPin = A2;   // Analog input pin for the back sensor
const int soundTolerance = 5;

int leftSensorValue = 0;
int rightSensorValue = 0;
int backSensorValue = 0;
//--------------------------------------

//Light sensor
#define S0 4
#define S1 5
#define S2 7
#define S3 6
#define sensorOut 8 //CHECK THIS IS WIRED CORRECT
const int tolerance = 0;
const int activateLightLightread = 4000; //Light reading must be below this and within activateLightDistance to activate the sound substem
int redFrequency = 0; 
int currentRed = pulseIn(sensorOut, LOW);
int lastRed = currentRed;
int initialRed = currentRed;
int lowestRed = 9999;
int countlow = 0;
int counthigh = 0;
bool facingLight = false;
//--------------------------------------

void setup() {
  Serial.begin(115200);
  //Distance sensor setup
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

  //Sound sensors setup

  //Light sensor setup
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);


  //Servo
  tone(4, 3000, 1000);
  delay(1000);
  servoLeft.attach(13);
  servoRight.attach(12);

  //LED
  pinMode(redPin,OUTPUT);
  digitalWrite(redPin,LOW);
  pinMode(greenPin,OUTPUT);
  digitalWrite(greenPin,LOW);
  pinMode(yellowPin,OUTPUT);
  digitalWrite(yellowPin,LOW);
  Serial.print("SETUP COMPLETE");
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (activeSystem == activesoundSystem){
    LEDcontrol(0, 0, 1);
    soundSystem();
    
    //SOUND SYSTEM CODE HERE
  }
  else if (activeSystem == activelightSystem){
    LEDcontrol(0, 1, 0);
    lightSystem();
    //LIGHT SYSTEM CODE HERE
  }
  else{
    LEDcontrol(1, 1, 1);
    //Finished, at destination, just check the distance. If the distance increases (target moved away again) reattach servos and return to sound subsystem
  }
}


void soundSystem(){
  readSound();
  /*
  Serial.print(50); // To freeze the lower limit
  Serial.print(" ");
  Serial.print(200); // To freeze the upper limit
  Serial.print(" ");
  
  Serial.print(leftSensorValue);
  Serial.print(",");
  Serial.print(rightSensorValue);
  Serial.print(",");
  Serial.println(backSensorValue);
  */
  delay(2);
  if ((backSensorValue >= (leftSensorValue + soundTolerance)) && (backSensorValue >= (rightSensorValue + soundTolerance))){ // &&  is AND
    Serial.println("ROTATE 180");
  }
  else if(rightSensorValue >= (leftSensorValue + soundTolerance)){
    Serial.println("RIGHT TURN");
    turn_right();
  }
  else if(leftSensorValue>= (rightSensorValue + soundTolerance)){
    Serial.println("LEFT TURN");
    turn_left();
  }
  else{
    //Serial.println("FORWARD");
    servoLeft.writeMicroseconds(1600);
    servoRight.writeMicroseconds(1400);
  }
  currentRed = readLight();
  Serial.print("RED, ");
  Serial.println(currentRed);
  //checkActivateLightSubsystem()

  //SOUND SYSTEM END
}

void lightSystem(){
  spinScan();
  //LIGHT SYSTEM
}

int readDistance() {
  int TOFdistance = (sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT"); 
  } 
  return(TOFdistance);
}

int readLight() {
  return (pulseIn(sensorOut, LOW));
}

int readSound() {
  delay(2);
  leftSensorValue = analogRead(analogLeftPin);
  //leftSensorValue = map(leftSensorValue, 0, 1023, 0, 255);
  delay(2);
  rightSensorValue = analogRead(analogRightPin);
  //rightSensorValue = map(rightSensorValue, 0, 1023, 0, 255);
  delay(2);
  backSensorValue = analogRead(analogBackPin);
  //backSensorValue = map(backSensorValue, 0, 1023, 0, 255);
}

void stop(){
  //detach servos or set speed to 0 or what?
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  servoLeft.detach();
  servoRight.detach();
}

void reAttach(){
  servoLeft.attach(13);
  servoRight.attach(12);
}

void checkActivateLightSubsystem(){
  int distanceReading = readDistance();
  int lightReading = readLight();
  if (distanceReading <= activateLightDistance){
    if(lightReading <= activateLightLightread){
      activeSystem = lightSystem;
    }
  }
}

void LEDcontrol (bool redOn, bool greenOn, bool yellowOn){ //eg to turn all on, LEDcontrol(True, True, True) or LEDcontrol(1,1,1)
  if (redOn){
    digitalWrite(redPin, HIGH);
  }
  else{
    digitalWrite(redPin, LOW);
  }
  if (greenOn){
    digitalWrite(greenPin, HIGH);
  }
  else{
    digitalWrite(greenPin, LOW);
  }
  if (yellowOn){
    digitalWrite(yellowPin, HIGH);
  }
  else{
    digitalWrite(yellowPin, LOW);
  }
}


void turn_left() {
  servoLeft.writeMicroseconds(1400);
  servoRight.writeMicroseconds(1400);
  delay(150);
  Serial.println("left");
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(150);
}

void turn_right() {
  servoLeft.writeMicroseconds(1600);
  servoRight.writeMicroseconds(1600);
  delay(150);
  Serial.println("right");
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(150);
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
    if (currentRed <= lowestRed){ //CHECK IF DISTANCE SENSOR IS REASONABLE TO STOP IT DETECTING THE SUN
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

