#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


#include <Adafruit_PWMServoDriver.h> // 서보 모터를 제어하는 PWM 드라이버 라이브러리
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // PWM 드라이버 객체 생성


#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
//#include <geometry_msgs/Quaternion.h>

ros::NodeHandle nh;

//geometry_msgs::Quaternion imu_msg;
std_msgs::Int16MultiArray analog_feedback_msgs;

//ros::Publisher mpu_pub("mpu6050", &imu_msg);
ros::Publisher angle_pub("pca9685", &analog_feedback_msgs);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
