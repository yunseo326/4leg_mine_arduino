
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[42]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


#include <Adafruit_PWMServoDriver.h> // 서보 모터를 제어하는 PWM 드라이버 라이브러리
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // PWM 드라이버 객체 생성

//Mux control pins
int s0 = 8;
int s1 = 9;
int s2 = 10;
int s3 = 11;

//Mux in “SIG” pin
int SIG_pin = 0;

byte analogValues[2][8] ={ {0,0,0,0,1,0,0,0},  
      {0,0,0,0,1,0,0,0}
      }; 
int num = 8;  
int analog_feedback[8] = {0,0,0,0,1,0,0,0};//

int angle_control = 0;
bool increasing = true;  


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
    // join I2C bus (I2Cdev library doesn't do this automatically)
    
    // rosserial setting
    rosserial_settings();

    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    delay(10); 


    // pca9685 setting
    
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    
    digitalWrite(s0, LOW);
    digitalWrite(s1, LOW);
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);
    
    pwm.begin(); // PWM 드라이버 초기화
    pwm.setOscillatorFrequency(27000000); // 내부 발진 주파수 설정 (보통 이 값은 변경하지 않아도 됨)
    pwm.setPWMFreq(60); // 서보 모터의 주파수 설정 (60Hz, 서보 모터마다 다를 수 있음)

    delay(10); // 초기 설정 후 잠깐 대기
    // pca9685 setting



    // verify connection
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus =1; // = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
//        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


}

void rosserial_settings(){
  nh.initNode();
  //nh.advertise(mpu_pub);
  nh.advertise(angle_pub);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    control_servo(angle_control);
    if (increasing) {
        if (angle_control < 170) {
            angle_control += 1;
        } else {
            increasing = false;  // 170에 도달하면 감소 모드로 변경
        }
    } else {
        if (angle_control > 10) {
            angle_control -= 1;
        } else {
            increasing = true;  // 10에 도달하면 증가 모드로 변경
        }
    }
    save_angle();


    // reset interrupt flag and get INT_STATUS byte
//    mpuInterrupt = false;
//    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    mpu.resetFIFO();
//    /if (mpuIntStatus & 0x02) {
      
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        
        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        Serial.print(q.w);
        Serial.print(q.x);
        Serial.print(q.y);
        Serial.println(q.z);

//    }/

    analog_feedback_msgs.data = analog_feedback;
    angle_pub.publish( &analog_feedback_msgs );

    nh.spinOnce();
    delay(50);
}


void control_servo(int angle){
    int pul = map(angle, 0, 180,  150, 600); // 0~180도 값을 150~600 PWM 값으로 변환
    for (int ch = 0; ch<num; ch++){
      pwm.setPWM(ch, 0, pul); 
    }   
}

void save_angle(){
  for(int ch = 0; ch <num; ch ++){ 
      Serial.print(ch); 
      analog_feedback[ch] = map(readMux(ch),analogValues[0][ch],analogValues[1][ch],0,180);
      Serial.println(analog_feedback[ch]);
      } 
}

const int muxChannel[8][4] PROGMEM={ {0,0,0,0},  
      {1,0,0,0}, //channel 1 
      {0,1,0,0}, //channel 2 
      {1,1,0,0}, //channel 3 
      {0,0,1,0}, //channel 4 
      {1,0,1,0}, //channel 5 
      {0,1,1,0}, //channel 6 
      {1,1,1,0}, //channel 7 
      }; 
      
int readMux(int channel)  { 
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
    
  
