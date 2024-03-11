#include <Servo.h>                    // Include servo library
 
Servo servoLeft;                      // Declare left and right servos
Servo servoRight;

void setup() {
  tone(4, 3000, 1000);                // Play tone for 1 second
  delay(1000);                        // Delay to finish tone
  servoLeft.attach(13);               // Attach left signal to pin 13 
  servoRight.attach(12);              // Attach right signal to pin 12
  /*
  servoLeft.writeMicroseconds(1700);
  servoRight.writeMicroseconds(1300);
  delay(1000);
  servoLeft.writeMicroseconds(2000);
  servoRight.writeMicroseconds(2000);
  delay(2225);
  servoLeft.writeMicroseconds(1300);
  servoRight.writeMicroseconds(1700);
  delay(1000);
  */
  servoLeft.writeMicroseconds(1600);
  servoRight.writeMicroseconds(1600);
  delay(1000);
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);  
  delay(1000);
  servoLeft.detach();                 // Stop sending servo signals
  servoRight.detach(); 

}

void loop() {
}
