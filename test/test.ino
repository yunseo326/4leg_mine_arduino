

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================


#include <Adafruit_PWMServoDriver.h> 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 

//Mux control pins
int s0 = 8;
int s1 = 9;
int s2 = 10;
int s3 = 11;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================


#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
//#include <geometry_msgs/Quaternion.h>

ros::NodeHandle nh;

//geometry_msgs::Quaternion imu_msg;
std_msgs::Int8MultiArray analog_feedback_msgs;

//ros::Publisher mpu_pub("mpu6050", &imu_msg);
ros::Publisher angle_pub("pca9685", &analog_feedback_msgs);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    
    // rosserial setting
    rosserial_settings();
    
    pwm_setting();

    
    Wire.begin();
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    Serial.begin(38400);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    accelgyro.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(accelgyro.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));




}


void rosserial_settings(){
  nh.initNode();
  //nh.advertise(mpu_pub);
  nh.advertise(angle_pub);
}

void pwm_setting(){
  
    // pca9685 setting
    
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    
    digitalWrite(s0, LOW);
    digitalWrite(s1, LOW);
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);
    
    pwm.begin(); 
    pwm.setOscillatorFrequency(27000000); // 
    pwm.setPWMFreq(60); // 

    delay(10); // 
    // pca9685 setting

}
void loop() {
}


void control_servo(int angle){
    int pul = map(angle, 0, 180,  150, 600); 
    for (int ch = 0; ch<8; ch++){
      pwm.setPWM(ch, 0, pul); 
    }   
}

const byte muxChannel[8][4] PROGMEM={ {0,0,0,0},  
      {1,0,0,0}, //channel 1 
      {0,1,0,0}, //channel 2 
      {1,1,0,0}, //channel 3 
      {0,0,1,0}, //channel 4 
      {1,0,1,0}, //channel 5 
      {0,1,1,0}, //channel 6 
      {1,1,1,0}, //channel 7 
      }; 
      
int readMux(int channel)  { 
      
    //Mux in “SIG” pin
    byte SIG_pin = 0;

    int controlPin[] = {s0, s1, s2, s3}; 
    
    //loop through the 4 sig 
    for(int i = 0; i < 4; i ++){ 
      int address = pgm_read_byte(&muxChannel[channel][i]);
      digitalWrite(controlPin[i], address); 
      } 
    //read the value at the SIG pin 
    int val = analogRead(SIG_pin); //return the value 
    return val; 
  } 
