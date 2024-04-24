//Other
int speedMultiplier = 1; //set to 0.5 to slow to half speed when close to the end
Servo servoLeft;
Servo servoRight;
int servoRotateSpeed = 30;
activeSystem = "sound";
//--------------------------------------

//Distance sensor
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X sensor;
#define HIGH_ACCURACY
const int stopDist = 45;
const int slowDist = 80;
const int reasonableDistance = 100; //Ignores light readings when distance sensor reads higher than this
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
int tolerance = 0;
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


  //Other
  tone(4, 3000, 1000);
  delay(1000);
  servoLeft.attach(13);
  servoRight.attach(12);


}

void loop() {
  // put your main code here, to run repeatedly:
  if (activeSystem == "sound"){
    pass
  }
  else if (activeSystem == "light"){
    pass
  }
  else{
    
    //Finished, at destination, just check the distance. If the distance increases (target moved away again) reattach servos and return to sound subsystem
  }
}


void soundSystem{
  pass
}

void lightSystem{
  pass
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

int readSound(sensor) { //take in sensor and return for that sensor, or program it to return 3 values (1 for each sensors) each time readSound is called?
  if (sensor == "left"){
    pass
  }
  else if (sensor == "right"){
    pass
  }
  else if (sensor == "rear"){
    pass
  }
  else{
    Serial.println("ONLY  LEFT, RIGHT, REAR sensors are valid inputs")
  }
}

void stop{
  //detach servos or set speed to 0 or what?
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  servoLeft.detach(13);
  servoRight.detach(12);
}

void reAttach{
  servoLeft.attach(13);
  servoRight.attach(12);
}