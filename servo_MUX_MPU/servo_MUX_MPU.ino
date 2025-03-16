
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;
#define OUTPUT_READABLE_QUATERNION

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


#include <Adafruit_PWMServoDriver.h> // 서보 모터를 제어하는 PWM 드라이버 라이브러리
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // PWM 드라이버 객체 생성

#define SERVOMIX 150
#define SERVOMAX 600

//Mux control pins
int s0 = 8;
int s1 = 9;
int s2 = 10;
int s3 = 11;

//Mux in “SIG” pin
int SIG_pin = 0;

int analogValues[2][8]; // [0도/180도][채널0~3]
int num = 8;  //4개라면 for문 때문에 1빼줌

int angle_control = 0;
int sequence = 0;
bool increasing = true;  // 증가(true) / 감소(false) 여부를 저장하는 변수



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    
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
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }



    // pca9685 setting
    pca9685_setting();
}

void pca9685_setting(){
    
  // 이거는 각 모터의 아날로그 값 저장하는 것

  // 0도 위치로 이동
  int degree2pul_0 = map(0, 0, 180, SERVOMIX, SERVOMAX); // 0~180도 값을 150~600 PWM 값으로 변환
  for (int ch = 0; ch<num; ch++){
    pwm.setPWM(ch, 0, degree2pul_0); // 채널 , pwm 숫자1, pwm 숫자2  뒤에가 얼마나 많이 해줄꺼지에 대한 내용
    delay(10);
  }
  
  delay(1000); // 서보 모터 안정화 대기
  
  // 0도 위치에서 아날로그 값 읽기
  Serial.println("0도 위치 아날로그 값:");
  for (int ch = 0; ch < num; ch++) {
    analogValues[0][ch] = readMux(ch);
    Serial.print("채널 "); Serial.print(ch); Serial.print(": ");
    Serial.println(analogValues[0][ch]);
  }
  delay(10); // 측정 후 대기
  
  // 180도 위치로 이동
  int degree2pul_180 = map(178, 0, 180, SERVOMIX, SERVOMAX); // 0~180도 값을 150~600 PWM 값으로 변환
  for (int ch = 0; ch<num; ch++){
    pwm.setPWM(ch, 0, degree2pul_180); // 채널 , pwm 숫자1, pwm 숫자2  뒤에가 얼마나 많이 해줄꺼지에 대한 내용
    delay(10);
  }
  delay(1000); // 서보 모터 안정화 대기
  
  
  // 180도 위치에서 아날로그 값 읽기
  Serial.println("180도 위치 아날로그 값:");
  for (int ch = 0; ch < num; ch++) {
    analogValues[1][ch] = readMux(ch);
    Serial.print("채널 "); Serial.print(ch); Serial.print(": ");
    Serial.println(analogValues[1][ch]);
  }
  delay(10); // 측정 후 대기
   
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    
    //while (!mpuInterrupt || sequence == 0) {
    Serial.print("move");
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
    check_angle();
//    sequence = 1;
//    }
//    sequence = 0;


    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    mpu.resetFIFO();
//    // check for overflow (this should never happen unless our code is too inefficient)
//    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
//        // reset so we can continue cleanly
//        mpu.resetFIFO();
//        Serial.println(F("FIFO overflow!"));
//
//    // otherwise, check for DMP data ready interrupt (this should happen frequently)
//    } else 
    if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

    }
}


void control_servo(int angle){
    int pul = map(angle, 0, 180, SERVOMIX, SERVOMAX); // 0~180도 값을 150~600 PWM 값으로 변환
    for (int ch = 0; ch<num; ch++){
      pwm.setPWM(ch, 0, pul); // 채널 , pwm 숫자1, pwm 숫자2  뒤에가 얼마나 많이 해줄꺼지에 대한 내용
      delay(1);
    }   
}

void check_angle(){
  for(int ch = 0; ch <num; ch ++){ 
      Serial.print("Value at channel "); 
      Serial.print(ch); Serial.print(": "); 
      Serial.println(map(readMux(ch),analogValues[0][ch],analogValues[1][ch],0,180));
      delay(5); 
        } 
}

int readMux(int channel)  { 
    int controlPin[] = {s0, s1, s2, s3}; 
    int muxChannel[16][4]={ {0,0,0,0},  
      {1,0,0,0}, //channel 1 
      {0,1,0,0}, //channel 2 
      {1,1,0,0}, //channel 3 
      {0,0,1,0}, //channel 4 
      {1,0,1,0}, //channel 5 
      {0,1,1,0}, //channel 6 
      {1,1,1,0}, //channel 7 
      {0,0,0,1}, //channel 8 
      {1,0,0,1}, //channel 9 
      {0,1,0,1}, //channel 10 
      {1,1,0,1}, //channel 11 
      {0,0,1,1}, //channel 12 
      {1,0,1,1}, //channel 13 
      {0,1,1,1}, //channel 14 
      {1,1,1,1} //channel 15 
      }; 
    //loop through the 4 sig 
    for(int i = 0; i < 4; i ++){ 
      digitalWrite(controlPin[i], muxChannel[channel][i]); 
      } 
    //read the value at the SIG pin 
    int val = analogRead(SIG_pin); //return the value 
    return val; 
  } 
    
  
