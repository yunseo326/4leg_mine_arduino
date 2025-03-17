#include <Wire.h> // I2C 통신을 위한 라이브러리
#include <Adafruit_PWMServoDriver.h> // 서보 모터를 제어하는 PWM 드라이버 라이브러리

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // PWM 드라이버 객체 생성

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

void setup() {
  Serial.begin(9600); // 시리얼 모니터를 9600 baud로 시작

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

  
  
void loop() {
  // 서보 모터를 0도에서 180도까지 이동
  for(int i = 0; i <= 180; i++){
    for (int ch = 0; ch<num; ch++){
      int pul = map(i, 0, 180, SERVOMIX, SERVOMAX); // 0~180도 값을 150~600 PWM 값으로 변환
      pwm.setPWM(ch, 0, pul); // 채널 , pwm 숫자1, pwm 숫자2  뒤에가 얼마나 많이 해줄꺼지에 대한 내용
      delay(10);
    }
  }
  
  // 1초 대기
  delay(1000);
  
  for(int ch = 0; ch <num; ch ++){ 
    Serial.print("Value at channel "); 
    Serial.print(ch); Serial.print(": "); 
    Serial.println(map(readMux(ch),analogValues[0][ch],analogValues[1][ch],0,180));
    delay(10); 
    } 
    
  // 서보 모터를 180도에서 0도까지 이동
  for(int i = 180; i >= 0; i--){
    for (int ch = 0; ch<num; ch++){
      int pul = map(i, 0, 180, SERVOMIX, SERVOMAX); // 0~180도 값을 150~600 PWM 값으로 변환
      pwm.setPWM(ch, 0, pul); // 채널 , pwm 숫자1, pwm 숫자2  뒤에가 얼마나 많이 해줄꺼지에 대한 내용
      delay(10);
      Serial.println(map(readMux(ch),analogValues[0][ch],analogValues[1][ch],0,180));
    }
  }
  // 1초 대기
  delay(1000);

  //Loop through and read all 16 values
  //Reports back Value at channel 6 is: 346
  for(int ch = 0; ch < num; ch ++){ 
    Serial.print("Value at channel "); 
    Serial.print(ch); Serial.print(": "); 
    Serial.println(map(readMux(ch),analogValues[0][ch],analogValues[1][ch],0,180));
    delay(10); 
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
  
