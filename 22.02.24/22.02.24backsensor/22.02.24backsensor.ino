#include <Servo.h> // include servo library

Servo servoLeft; // Declare left and right servos
Servo servoRight;

const int analogLeftPin = A0;   // Analog input pin for the left sensor
const int analogRightPin = A1;  // Analog input pin for the right sensor
const int analogBackPin = A2;   // Analog input pin for the back sensor

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

// Prototype functions
void move_forward();
void turn_left();
void turn_right();
void waitAfterMove();

void setup() {
  servoLeft.attach(13);
  servoRight.attach(12);

  Serial.begin(9600);

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
}

void loop() {
  // Sensor readings and output calculations for all sensors
  leftSensorValue = analogRead(analogLeftPin);
  rightSensorValue = analogRead(analogRightPin);
  backSensorValue = analogRead(analogBackPin);

  leftOutputValue = map(leftSensorValue, 0, 1023, 0, 255);
  rightOutputValue = map(rightSensorValue, 0, 1023, 0, 255);
  backOutputValue = map(backSensorValue, 0, 1023, 0, 255);

  // Calculate average values using lastValues arrays
  leftAverageValue = findAverage(leftLastValues);
  rightAverageValue = findAverage(rightLastValues);
  backAverageValue = findAverage(backLastValues);

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

  // Wait before the next loop
  delay(50); // Adjust as needed
}

// Function to calculate the average of an array
int findAverage(int arr[25]) {
  int result = 0;
  int sum = 0;
  for (i = 0; i < 25; i++) {
    sum += arr[i];
  }
  result = sum / 25;
  return result;
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
  servoLeft.writeMicroseconds(1600);
  servoRight.writeMicroseconds(1600);
  delay(250);
  Serial.println("left");
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(1000);
}

void turn_right() {
  servoLeft.writeMicroseconds(1400);
  servoRight.writeMicroseconds(1400);
  delay(250);
  Serial.println("right");
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(1000);
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
  delay(5000); // 5 seconds
}
