// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;



// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

unsigned long lastTime = 0;  // 이전 시간 저장 (자이로 적분용)
float roll = 0.0, pitch = 0.0, yaw = 0.0;  // 각도 값


#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    
    accelgyro.setXAccelOffset(-2950);
    accelgyro.setYAccelOffset(549);
    accelgyro.setZAccelOffset(1230);
    accelgyro.setXGyroOffset(-32);
    accelgyro.setYGyroOffset(-45);
    accelgyro.setZGyroOffset(-15);
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */
    /*
    accelgyro.setXAccelOffset(-16752);
    accelgyro.setYAccelOffset(428);
    accelgyro.setZAccelOffset(-584);
    accelgyro.setXGyroOffset(-997);
    accelgyro.setYGyroOffset(-489);
    accelgyro.setZGyroOffset(-311);
*/
    lastTime = millis();  // 초기 시간 설정

}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

   Serial.print("a/g:\t");
//   Serial.print(ax); Serial.print("\t");
//   Serial.print(ay); Serial.print("\t");
//   Serial.print(az); Serial.print("\t");
   Serial.print("x ");Serial.print(gx); Serial.print("\t");
   Serial.print("y ");Serial.print(gy); Serial.print("\t");
   Serial.print("z ");Serial.println(gz);
    // Serial print
//
//   // blink LED to indicate activity
//   float vec = (float)ax * ax + (float)az * az;
//   float vec2 = (float)ax * ax + (float)az * az;
//
//   float accRoll = atan2(ay, sqrt(vec)); //* 180.0 / PI;
//   float accPitch = atan2(-ax, sqrt(vec2)); // * 180.0 / PI;
//    
//   // 3. 자이로 데이터를 통해 적분 (시간 간격 사용)
//   unsigned long currentTime = millis();
//   float dt = (currentTime - lastTime) / 1000.0;  // 시간 차이 (초 단위)
//   lastTime = currentTime;
//
//   float gyroRoll = roll + (gx / 131.0) * dt;  // 131.0 = MPU6050 감도 값
//   float gyroPitch = pitch + (gy / 131.0) * dt;
//   float gyroYaw = yaw + (gz / 131.0) * dt;
//
//   // 4. 가속도와 자이로 데이터를 조합 (보정)
//   roll = 0.96 * gyroRoll + 0.04 * accRoll;
//   pitch = 0.96 * gyroPitch + 0.04 * accPitch;
//   yaw = gyroYaw;  
//   
//   Serial.print("Roll: "); Serial.print(roll);
//   Serial.print("\tPitch: "); Serial.print(pitch);
//   Serial.print("\tYaw: "); Serial.println(yaw);

//    char txt[50], a1[10], a2[10];
//    dtostrf(roll, 4, 3, a1);
//    dtostrf(pitch, 4, 3, a2);
//    sprintf(txt, "X:%s Y:%s", a1, a2);
//    Serial.println(txt);
//    
    delay(100);  // 100ms마다 측정
}
