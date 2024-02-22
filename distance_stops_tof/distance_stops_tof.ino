#include "Seeed_vl53l0x.h"
#include <Servo.h>
Servo servoLeft;
Servo servoRight;


Seeed_vl53l0x VL53L0X;
const int stopDist = 40;
const int slowDist = 200;
int slowSpeed = 30;
int fastSpeed = 200;
int servoSpeed = fastSpeed;
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define SERIAL SerialUSB
#else
    #define SERIAL Serial
#endif

void setup() {
    tone(4, 3000, 1000);
    delay(1000);
    servoLeft.attach(13);
    servoRight.attach(12);
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    SERIAL.begin(115200);
    delay(50); // See if you can remove, attempt to stop vl53 measure fail from power instability
    Status = VL53L0X.VL53L0X_common_init();
    if (VL53L0X_ERROR_NONE != Status) {
        SERIAL.println("start vl53l0x mesurement failed!");
        VL53L0X.print_pal_error(Status);
        while (1);
    }
    VL53L0X.VL53L0X_high_accuracy_ranging_init();

    if (VL53L0X_ERROR_NONE != Status) {
        SERIAL.println("start vl53l0x mesurement failed!");
        VL53L0X.print_pal_error(Status);
        while (1);
    }
    SERIAL.println("SETUP COMPLETE");
}


void loop() {
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    memset(&RangingMeasurementData, 0, sizeof(VL53L0X_RangingMeasurementData_t));
    
    // Get measurement Info
    Status = VL53L0X.PerformSingleRangingMeasurement(&RangingMeasurementData);
    
    // What to do with measurement info
    if (VL53L0X_ERROR_NONE == Status) {
        if (RangingMeasurementData.RangeMilliMeter <= stopDist) {
            stop();
        } else {
          if ((RangingMeasurementData.RangeMilliMeter <= slowDist) and  (RangingMeasurementData.RangeMilliMeter > stopDist)){
            servoSpeed = slowSpeed;
            SERIAL.println("SLOW");
          } else {
            servoSpeed = fastSpeed;
            SERIAL.println("RETURN TO FAST");
          }
          SERIAL.println(servoSpeed);
          servoLeft.writeMicroseconds(1500+servoSpeed);
          servoRight.writeMicroseconds(1500-servoSpeed);
          SERIAL.print("MOVING");
            SERIAL.print("Measured distance:");
            SERIAL.print(RangingMeasurementData.RangeMilliMeter);
            SERIAL.println(" mm");
        }
    } else {
        SERIAL.print("mesurement failed !! Status code =");
        SERIAL.println(Status);
    }
    //delay(300);
}


void stop(){
  SERIAL.println("STOP!!!");
  servoSpeed = 0;
  servoLeft.writeMicroseconds(1500+servoSpeed);
  servoRight.writeMicroseconds(1500-servoSpeed);
  // servoLeft.detach();
  // servoRight.detach(); 
}