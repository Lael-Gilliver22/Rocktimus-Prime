#include <stdio.h>
#include <Servo.h>                    // Include servo library
 
Servo servoLeft;                      // Declare left and right servos
Servo servoRight;

#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define sensorOut 2

// Stores frequency read by the photodiodes
int redFrequency = 0;

void lightScan();

void setup() {
  tone(4, 3000, 1000);                // Play tone for 1 second
  delay(1000);                        // Delay to finish tone
  servoLeft.attach(13);               // Attach left signal to pin 13 
  servoRight.attach(12);              // Attach right signal to pin 12

  // Setting the outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
  // Setting the sensorOut as an input
  pinMode(sensorOut, INPUT);
  
  // Setting frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,HIGH);
  
   // Begins serial communication 
  Serial.begin(9600);

  lightScan();
}

void loop() {}

void lightScan()
{
  int max_intensity = 99999;
  int angle_max_intensity = 0;
  int angle = 2;
  
  for(int i = 0; i <= 2; i++)
  {
    delay(1000);
    redFrequency = pulseIn(sensorOut, LOW);
    Serial.print("at angle ");
    Serial.print(angle);
    Serial.print(" R = ");
    Serial.println(redFrequency);
    if (redFrequency < max_intensity){
      max_intensity = redFrequency;
      angle_max_intensity = angle;
      Serial.println("new max detected");
    }
    servoLeft.writeMicroseconds(1550);
    servoRight.writeMicroseconds(1550);
    delay(250);
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1500);
    angle += 1;
    Serial.print("angle is");
    Serial.println(angle);
    Serial.print("angle max is");
    Serial.println(angle_max_intensity);
  }
  
  delay(1000);
  servoLeft.writeMicroseconds(1450);
  servoRight.writeMicroseconds(1450);
  delay(400);
  angle -= 3;
  Serial.print("angle is");
  Serial.println(angle);
  Serial.print("angle max is");
  Serial.println(angle_max_intensity);

  for(int i = 0; i <= 1; i++)
  {
    delay(1000);
    redFrequency = pulseIn(sensorOut, LOW);
    Serial.print("at angle ");
    Serial.print(angle);
    Serial.print(" R = ");
    Serial.println(redFrequency);
    if (redFrequency < max_intensity){
      max_intensity = redFrequency;
      angle_max_intensity = angle;
      Serial.println("new max detected");
    }
    servoLeft.writeMicroseconds(1550);
    servoRight.writeMicroseconds(1550);
    delay(250);
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1500); 
    angle -= 1;
    Serial.print("angle is");
    Serial.println(angle);
    Serial.print("angle max is");
    Serial.println(angle_max_intensity);
  }
  Serial.print("max intensity was ");
  Serial.print(max_intensity);
  Serial.print(" at the angle ");
  Serial.println(angle_max_intensity);

  delay(1000);
  servoLeft.writeMicroseconds(1550);
  servoRight.writeMicroseconds(1550);
  delay(50 * angle_max_intensity);
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  delay(100);
  Serial.print("angle is");
  Serial.println(angle);
  Serial.print("angle max is");
  Serial.println(angle_max_intensity);
  

}
