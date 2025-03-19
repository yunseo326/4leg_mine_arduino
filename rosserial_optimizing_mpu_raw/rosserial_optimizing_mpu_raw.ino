

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
byte s0 = 8;
byte s1 = 9;
byte s2 = 10;
byte s3 = 11;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================


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
    
    // rosserial setting
    rosserial_settings();
    
    pwm_setting();

    
    Wire.begin();
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    Serial.begin(115200);

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


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
byte angle_control = 0;
bool increasing = true;  

void loop() {
    int16_t analog_feedback[8]; 

    
    control_servo(angle_control);
    if (increasing) {
        if (angle_control < 170) {
            angle_control += 1;
        } else {
            increasing = false;  
        }
    } else {
        if (angle_control > 10) {
            angle_control -= 1;
        } else {
            increasing = true; 
        }
    }
    for(int ch = 0; ch <8; ch ++){ 
    analog_feedback[ch] = map(readMux(ch),140,540,0,180);
    }
    
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    analog_feedback_msgs.data = analog_feedback;  
    analog_feedback_msgs.data_length = 8;
    
    angle_pub.publish( &analog_feedback_msgs );

    nh.spinOnce();
    delay(100);
}
//imu_msg.x = q.x;
//imu_msg.y = q.y;
//imu_msg.z = q.z;
//imu_msg.w = q.w;

//mpu_pub.publish( &imu_msg );

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

    byte controlPin[] = {s0, s1, s2, s3}; 
    
    //loop through the 4 sig 
    for(int i = 0; i < 4; i ++){ 
      byte address = pgm_read_byte(&muxChannel[channel][i]);
      digitalWrite(controlPin[i], address); 
      } 
    //read the value at the SIG pin 
    byte val = analogRead(SIG_pin); //return the value 
    return val; 
  } 
    
  
