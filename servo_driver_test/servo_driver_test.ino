#include <Wire.h> //i2c통신 라이브러리
#include <Adafruit_PWMServoDriver.h> //서보 드라이버 라이브러리

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void setup() {
  Serial.begin(9600); //시리얼모니터를 쓰겠다!

  pwm.begin();
  pwm.setOscillatorFrequency(27000000); //이건 녹칸다도 잘 모르겠음!
  pwm.setPWMFreq(60); //서보모터마다 다를 수 있음!

  delay(10);
}

void loop() {
  //0~180
  for(int i = 0;i<=180;i++){
    int pul = map(i,0,180,150,600); //0~180도값을 150~600으로 조정
    pwm.setPWM(0, 0, pul); 
  }
  //1초쉬고
  delay(1000);
  //180~0
  for(int i = 180;i>=0;i--){
    int pul = map(i,0,180,150,600); //0~180도값을 150~600으로 조정
    pwm.setPWM(0, 0, pul); 
  }
  //1초쉬고
  delay(1000);
}