#include <Wire.h>
unsigned long lastTime = 0;  // 이전 시간 저장 (자이로 적분용)


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================


#include <Adafruit_PWMServoDriver.h> // 서보 모터를 제어하는 PWM 드라이버 라이브러리
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // PWM 드라이버 객체 생성

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


void setup() {

  rosserial_settings();
  
  // Serial init
  Serial.begin(115200);
  // Wire init
  Wire.begin();
  // Power Management
  Wire.beginTransmission(0x68);
  Wire.write(107);
  Wire.write(0);
  Wire.endTransmission();
  // Register 26
  for(uint8_t i = 2; i <= 7; i++)
  {
    Wire.beginTransmission(0x68);
    Wire.write(26);
    Wire.write(i << 3 | 0x03);
    Wire.endTransmission();
  }
  // Register 27
  Wire.beginTransmission(0x68);
  Wire.write(27);
  Wire.write(3 << 3);
  Wire.endTransmission();
  // Register 28
  Wire.beginTransmission(0x68);
  Wire.write(28);
  Wire.write(0);
  Wire.endTransmission();


  
  pwm.begin(); // PWM 드라이버 초기화
  pwm.setOscillatorFrequency(27000000); // 내부 발진 주파수 설정 (보통 이 값은 변경하지 않아도 됨)
  pwm.setPWMFreq(60); // 서보 모터의 주파수 설정 (60Hz, 서보 모터마다 다를 수 있음)

  delay(10); // 초기 설정 후 잠깐 대기

  lastTime = millis();  // 초기 시간 설정

}


void rosserial_settings(){
  nh.initNode();

  //nh.advertise(mpu_pub);
  nh.advertise(angle_pub);
}



int16_t offset[3] = {-32, 45, -15};




byte angle_control = 0;
bool increasing = true;  

void loop() {
  uint8_t i;
  static int16_t acc_raw[3]={0,}, gyro_raw[3]={0,};
  // Get Accel
  Wire.beginTransmission(0x68);
  Wire.write(59);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  for(i = 0; i < 3; i++) acc_raw[i] = (Wire.read() << 8) | Wire.read();
  // Get Gyro
  Wire.beginTransmission(0x68);
  Wire.write(67);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  for(i = 0; i < 3; i++)
    gyro_raw[i] = gyro_raw[i] * 0.8 + 0.2 * (((Wire.read() << 8) | Wire.read()) - offset[i]);
  // Get DT

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // 시간 차이 (초 단위)
  lastTime = currentTime;
   
  // Gyro Rate
  float gyro_rate[3];
  for(i = 0; i < 3; i++) gyro_rate[i] = gyro_raw[i] / 16.4 * dt;
  // Calculate
  static float angle[3]={0,}, vec;
  vec = sqrt(pow(acc_raw[0], 2) + pow(acc_raw[2], 2));
  angle[0] = (angle[0] + gyro_rate[0]) * 0.98
    + atan2(acc_raw[1], vec) * RAD_TO_DEG * 0.02;
  vec = sqrt(pow(acc_raw[1], 2) + pow(acc_raw[2], 2));
  angle[1] = (angle[1] - gyro_rate[1]) * 0.98
    + atan2(acc_raw[0], vec) * RAD_TO_DEG * 0.02;
  // Serial print
  angle[2] += gyro_rate[2];
  char str[50], a1[10], a2[10], a3[10];
  dtostrf(angle[0], 4, 3, a1);
  dtostrf(angle[1], 4, 3, a2);
  dtostrf(angle[2], 4, 3, a3);
  sprintf(str, "X:%s Y:%s Z:%s", a1, a2, a3);
  Serial.println(str);


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
  
  analog_feedback_msgs.data = analog_feedback;  
  analog_feedback_msgs.data_length = 8;
  
  angle_pub.publish( &analog_feedback_msgs );

  nh.spinOnce();
  delay(10);  
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
    
