//Other
int speedMultiplier = 1; //set to 0.5 to slow to half speed when close to the end

//Distance sensor
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X sensor;
#define HIGH_ACCURACY
const int stopDist = 45;
const int slowDist = 80;

//Sound sensors

//Light sensor



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

  //Other
}

void loop() {
  // put your main code here, to run repeatedly:

}


int readDistance() {
  int TOFdistance = (sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT"); 
  } 
  return(TOFdistance);
}

int readLight() {
  //
}

int readSound() {
  //
}


