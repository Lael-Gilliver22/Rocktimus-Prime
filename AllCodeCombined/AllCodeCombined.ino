//Switched to Lukes Sound Sensing Code
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
int reasonableDistance = 500; //Ignores light readings when distance sensor reads higher than this. Should update to be LowestDistance + some number
const int activateLightDistance = 450; //will switch to sound subsystem within this distance and when a largt enough red light reading (activateLightLightread)
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

int leftOutputValue = 0;
int rightOutputValue = 0;
int backOutputValue = 0;

int leftLastValues[25];
int rightLastValues[25];
int backLastValues[25];
int count = 0;
int i = 0;
int leftAverageValue = 0;
int rightAverageValue = 0;
int backAverageValue = 0;

// Constants for outlier detection
const int arraySize = 100;
const float threshold = 2.5;

// Variables for time tracking
unsigned long lastOutlierTime = 0;
const unsigned long outlierCheckInterval = 5000; // 5 seconds
//--------------------------------------

//Light sensor
#define S0 4
#define S1 5
#define S2 7
#define S3 6
#define sensorOut 8 //CHECK THIS IS WIRED CORRECT
const int tolerance = 0;
const int activateLightLightread = 650; //Light reading must be below this and within activateLightDistance to activate the sound substem
int redFrequency = 0; 
int currentRed = pulseIn(sensorOut, LOW);
int lastRed = currentRed;
int initialRed = currentRed;
int lowestRed = 9999;
int countlow = 0;
int counthigh = 0;
int countIncrease = 0;
bool facingLight = false;
bool loopUntil = false;
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
    // Initial filling of lastValues arrays
  for (i = 0; i < 25; i++) {
    leftSensorValue = analogRead(analogLeftPin);
    rightSensorValue = analogRead(analogRightPin);
    backSensorValue = analogRead(analogBackPin);

    leftOutputValue = map(leftSensorValue, 0, 1023, 0, 255);
    rightOutputValue = map(rightSensorValue, 0, 1023, 0, 255);
    backOutputValue = map(backSensorValue, 0, 1023, 0, 255);

    leftLastValues[i] = leftOutputValue;
    rightLastValues[i] = rightOutputValue;
    backLastValues[i] = backOutputValue;
  }

    // Calculate initial average values
  leftAverageValue = findAverage(leftLastValues);
  rightAverageValue = findAverage(rightLastValues);
  backAverageValue = findAverage(backLastValues);

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
  Serial.println("LOOP");
  if (activeSystem == activesoundSystem){
    LEDcontrol(0, 0, 1);
    Serial.println("RUN SOUND");
    soundSystem();
    
    //SOUND SYSTEM CODE HERE
  }
  else if (activeSystem == activelightSystem){
    LEDcontrol(0, 1, 0);
    Serial.println("RUN light");
    lightSystem();
    //LIGHT SYSTEM CODE HERE
  }
  else{
    LEDcontrol(1, 1, 1);
    Serial.println("RUN distance");
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
  leftOutputValue = map(leftSensorValue, 0, 1023, 0, 255);
  rightOutputValue = map(rightSensorValue, 0, 1023, 0, 255);
  backOutputValue = map(backSensorValue, 0, 1023, 0, 255);

  // Calculate average values using lastValues arrays
  leftAverageValue = findAverage(leftLastValues);
  rightAverageValue = findAverage(rightLastValues);
  backAverageValue = findAverage(backLastValues);

  /*
    // Print sensor and average values
  Serial.print("Left Sensor: ");
  Serial.print(leftOutputValue);
  Serial.print(", Left Average Value: ");
  Serial.print(leftAverageValue);
  Serial.print(" | Right Sensor: ");
  Serial.print(rightOutputValue);
  Serial.print(", Right Average Value: ");
  Serial.print(rightAverageValue);
  Serial.print(" | Back Sensor: ");
  Serial.print(backOutputValue);
  Serial.print(", Back Average Value: ");
  Serial.println(backAverageValue);
  */
    // Store the current outputValues in lastValues arrays
  leftLastValues[count] = leftOutputValue;
  rightLastValues[count] = rightOutputValue;
  backLastValues[count] = backOutputValue;
    // Increment count and handle wraparound
  count = (count + 1) % 25;

   // Outlier detection using Z-score for all sensors
  float leftZScore = (leftOutputValue - leftAverageValue) / calculateStdDev(leftLastValues, 25, leftAverageValue);
  float rightZScore = (rightOutputValue - rightAverageValue) / calculateStdDev(rightLastValues, 25, rightAverageValue);
  float backZScore = (backOutputValue - backAverageValue) / calculateStdDev(backLastValues, 25, backAverageValue);

  // Check if the Z-scores are greater than the threshold
  if ((abs(backZScore) > threshold) && ((abs(backZScore) > abs(leftZScore)) || (abs(backZScore) > abs(rightZScore)))) {
    Serial.print("Back Sensor Outlier detected! Z-Score: ");
    Serial.println(backZScore);
    turn_left();
    waitAfterMove(); // Wait after turning left
    lastOutlierTime = millis(); // Update the last outlier detection time
  }
  else if (abs(leftZScore) > threshold && abs(leftZScore) > abs(rightZScore)) {
    Serial.print("Left Sensor Outlier detected! Z-Score: ");
    Serial.println(leftZScore);
    turn_left();
    waitAfterMove(); // Wait after turning left
    lastOutlierTime = millis(); // Update the last outlier detection time
  }
  else if (abs(rightZScore) > threshold) {
    Serial.print("Right Sensor Outlier detected! Z-Score: ");
    Serial.println(rightZScore);
    turn_right();
    waitAfterMove(); // Wait after turning right
    lastOutlierTime = millis(); // Update the last outlier detection time
  }
  else {
    // Check if it's been 5 seconds since the last outlier detection
    if (millis() - lastOutlierTime > outlierCheckInterval) {
      move_forward(); // Move forward if no outliers detected for 5 seconds
      waitAfterMove(); // Wait after moving forward
      lastOutlierTime = millis(); // Reset the timer
    }
  }
  
  currentRed = readLight();
  Serial.print("RED, ");
  Serial.println(currentRed);
  Serial.println("Check activate light subsystem");
  checkActivateLightSubsystem();

  //SOUND SYSTEM END
}

void lightSystem(){
  //spinScan();
  Serial.println("LightSystem");
  checkDistance();
  if (facingLight == false){
    Serial.println("NOT FACING LIGHT, DO LIGHTSCAN");
    lightScan();
  }
  else{
    Serial.println("IS FACING LIGHT, FORWARD!!");
    //here go forwards unless there is an increase detected in red. then scan again
    servoLeft.writeMicroseconds(1500 + servoRotateSpeed);
    servoRight.writeMicroseconds(1500 - servoRotateSpeed);
    checkRedIncrease();
    checkDistance();
  }
  //LIGHT SYSTEM
}

int readDistance() {
  Serial.print("Reading Distance");
  delay(20);
  int TOFdistance = (sensor.readRangeSingleMillimeters());
  Serial.println(TOFdistance);
  if (sensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT"); 
  } 
  Serial.print("Returning distance");
  return(TOFdistance);
}

int readLight() {
  Serial.print("Reading Light");
  Serial.println(pulseIn(sensorOut, LOW));
  return (pulseIn(sensorOut, LOW));
}

int readSound() {
  Serial.println("Reading Sound");
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
  Serial.println("AHHHHHH STOPPING");
  //detach servos or set speed to 0 or what?
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  servoLeft.detach();
  servoRight.detach();
}

void reAttach(){
  Serial.println("Re-attaching Servos");
  servoLeft.attach(13);
  servoRight.attach(12);
}

void checkActivateLightSubsystem(){
  Serial.println("checking activate light subsystem");
  int distanceReading = readDistance();
  int lightReading = readLight();
  Serial.print("distanceReading");
  Serial.print("lightReading");
  Serial.println("CHECKING ACTIVATE LIGHT SUBSYSTEM ABOVE VALUES");
  if (1){//distanceReading <= activateLightDistance){
    if(lightReading <= activateLightLightread){
      Serial.println("should activate light subsystem NOW");
      facingLight = false;
      activeSystem = activelightSystem;
    }
    else{
      Serial.println("Not enough light intensity for light subsystem");
    }
  }
  else{
    Serial.println("too far for light subsystem");
  }
}

void LEDcontrol (bool redOn, bool greenOn, bool yellowOn){ //eg to turn all on, LEDcontrol(True, True, True) or LEDcontrol(1,1,1)
  Serial.println("LED Change");
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

void move_forward() {
  servoLeft.writeMicroseconds(1700);
  servoRight.writeMicroseconds(1300);
  delay(1000);
  Serial.println("forwards");
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(1000);
}

void turn_left() {
  Serial.println("turning left");
  servoLeft.writeMicroseconds(1400);
  servoRight.writeMicroseconds(1400);
  delay(150);
  Serial.println("left");
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(150);
}

void turn_right() {
  Serial.println("turning right");
  servoLeft.writeMicroseconds(1600);
  servoRight.writeMicroseconds(1600);
  delay(150);
  Serial.println("right");
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(150);
}

void rotate_180() {
  Serial.println("turning 180");
  servoLeft.writeMicroseconds(1600);
  servoRight.writeMicroseconds(1600);
  delay(1500); //TEST 180 TURN SPEED
  Serial.println("180");
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(150);
}

//spinscan is available if regular lightScan does not work.
void spinScan(){
  Serial.println("doing spinscan");
  lowestRed = 99999;
  countlow = 0;
  //Turning Right
  servoLeft.writeMicroseconds(1500 + servoRotateSpeed);
  servoRight.writeMicroseconds(1500 + servoRotateSpeed);
  unsigned long startTime = millis(); // Get the current time

  while (millis() - startTime < 8000) {
  // This will execute for 8 seconds
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  currentRed = readLight();
    //printReadings();
    if (currentRed <= lowestRed){ //CHECK IF DISTANCE SENSOR IS REASONABLE TO STOP IT DETECTING THE SUN
      if(readDistance()<reasonableDistance){
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
  }
  Serial.println("Turning to find red light");
  while (currentRed >= (lowestRed)) { //Add tolerance to lowest red?
    if (readDistance()<reasonableDistance){
      Serial.println("WITHIN REASONABLE DISTANCE TO FIND RED LIGHT");
      currentRed = readLight();
    }
  }
  Serial.println("FOUND THE RED LIGHT, stop");
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(150);
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

void lightScan(){
  Serial.println("running lightScan");
  //spinScan();
  
  currentRed = pulseIn(sensorOut, LOW);
  lastRed = currentRed;
  initialRed = currentRed;
  lowestRed = 9999;
  countlow = 0;
  counthigh = 0;
  loopUntilRedIncrease("left");
  Serial.println("LEFT DONE");
  //turn back until reaching lowest value
  loopUntilRedIncrease("right");
  Serial.println("RIGHT DONE");
  
  facingLight = false;
  servoLeft.writeMicroseconds(1500 - servoRotateSpeed/1.5);
  servoRight.writeMicroseconds(1500 - servoRotateSpeed/1.5);
  while (!facingLight) {
    currentRed = readLight();
    printReadings();
    if (currentRed <= lowestRed + 5){ //5 tolerance to prevent overshoots
      if (readDistance()<reasonableDistance){
        facingLight = true;
      }
    }
  }
  
}

void loopUntilRedIncrease(String direction){
  Serial.println("running loop until red increase");
  loopUntil = false;
  currentRed = readLight();
  initialRed = currentRed;
  if (direction == "left"){
    servoLeft.writeMicroseconds(1500 - servoRotateSpeed/1.5);
    servoRight.writeMicroseconds(1500 - servoRotateSpeed/1.5);
  }
  else{
    servoLeft.writeMicroseconds(1500 + servoRotateSpeed/1.5);
    servoRight.writeMicroseconds(1500 + servoRotateSpeed/1.5);
  }
  while (!loopUntil) {
    currentRed = readLight();
    printReadings();
    if (currentRed <= lowestRed){
      if (readDistance()<reasonableDistance){
        counthigh = 0;
        if (countlow < 5){
          countlow += 1;
        }
        else{
          countlow = 0;
          lowestRed = currentRed;
        }
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
          loopUntil = true;
        }
      }
    }
    if ((currentRed > (initialRed * 1.2)) && (direction == "left")){//going in wrong direction, will not find red light. Best to spin 90 degrees, or to break out the loop. NEEDS IMPROVING
      Serial.println("Going too far away!");
      //assume this will only happen on the first left turn, so exit the loop
      loopUntil = true;
    }
    lastRed = currentRed;
  }
}

void checkRedIncrease(){
  Serial.println("checking for red increase");
  //facingLight should already be true
  currentRed = pulseIn(sensorOut, LOW);
  if (currentRed > lastRed){
    countIncrease += 1;
  }
  else{
    countIncrease = 0;
  }
  if (countIncrease == 5){
    Serial.print("Re do lightscan, drove away from light");
    facingLight = false;
    countIncrease = 0;
  }
}

void checkDistance(){
  Serial.println("checking for distance to stop");
  int distanceReading = readDistance();
  if (distanceReading <= stopDist){
    Serial.println("EMERGENCY!!! WITHIN STOP DISTANCE");
    stop();
  }
  else{
    Serial.println("Resume Moving Now");
    if (distanceReading <= slowDist){
      servoRotateSpeed = 10;
    }
    else{
      servoRotateSpeed = 30;
    }
    reAttach();
  }
  // backup: check the light sensor
  int lightReading = readLight();
  int stopLight = 33;
  if (lightReading <= stopLight){
    //stop();
  }
}

int findAverage(int arr[25]) {
  int result = 0;
  int sum = 0;
  for (i = 0; i < 25; i++) {
    sum += arr[i];
  }
  result = sum / 25;
  return result;
}

// Function to calculate the standard deviation of an array
float calculateStdDev(int values[], int size, float mean) {
  float sumSqDiff = 0;
  for (i = 0; i < size; i++) {
    float diff = values[i] - mean;
    sumSqDiff += diff * diff;
  }
  return sqrt(sumSqDiff / size);
}

void waitAfterMove() {
  delay(5000); // 5 seconds, may be unnecesary? good for testing
}