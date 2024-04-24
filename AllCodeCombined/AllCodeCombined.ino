//Servo
#include <Servo.h>
int speedMultiplier = 1; //set to 0.5 to slow to half speed when close to the end
Servo servoLeft;
Servo servoRight;
int servoRotateSpeed = 30;
//--------------------------------------

//Other
const int soundSystem = 0;
const int lightSystem = 1;
const int stopSystem = 2;
int activeSystem = 0; //0 is sound, 1 is light, 3 is finish
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

//--------------------------------------

//Light sensor
#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define sensorOut 2 //CHECK THIS IS WIRED CORRECT
const int tolerance = 0;
const int activateLightLightread = 50; //Light reading must be below this and within activateLightDistance to activate the sound substem
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
  Serial.begin(9600);
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
}

void loop() {
  // put your main code here, to run repeatedly:
  if (activeSystem == soundSystem){
    LEDcontrol(0, 0, 1);
    pass()
  }
  else if (activeSystem == lightSystem){
    LEDcontrol(0, 1, 0);
    pass()
  }
  else{
    LEDcontrol(1, 1, 1);
    //Finished, at destination, just check the distance. If the distance increases (target moved away again) reattach servos and return to sound subsystem
  }
}


void soundSystem{
  pass()
}

void lightSystem{
  pass()
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

int readSound(int soundSensor) { //take in sensor and return for that sensor, or program it to return 3 values (1 for each sensors) each time readSound is called?
  int soundSensor = soundSensor
  if (Soundsensor == 0){ //left
    pass()
  }
  else if (Soundsensor == 1){ //right
    pass()
  }
  else if (Soundsensor == 2){ //rear
    pass()
  }
  else{
    Serial.println("ONLY  LEFT, RIGHT, REAR sensors are valid inputs")
  }
}

void stop(){
  //detach servos or set speed to 0 or what?
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  servoLeft.detach(13);
  servoRight.detach(12);
}

void reAttach(){
  servoLeft.attach(13);
  servoRight.attach(12);
}

void checkActivateLightSubsystem(){
  int distanceReading = readDistance();
  int lightReading = readLight();
  if (distanceReading <= activateLightdistance){
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