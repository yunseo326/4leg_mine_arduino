#include <Wire.h> // I2C 통신을 위한 라이브러리
#include <Adafruit_PWMServoDriver.h> // 서보 모터를 제어하는 PWM 드라이버 라이브러리

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // PWM 드라이버 객체 생성

void setup() {
  Serial.begin(9600); // 시리얼 모니터를 9600 baud로 시작

  pwm.begin(); // PWM 드라이버 초기화
  pwm.setOscillatorFrequency(27000000); // 내부 발진 주파수 설정 (보통 이 값은 변경하지 않아도 됨)
  pwm.setPWMFreq(60); // 서보 모터의 주파수 설정 (60Hz, 서보 모터마다 다를 수 있음)

  delay(10); // 초기 설정 후 잠깐 대기
}

void loop() {
  // 서보 모터를 0도에서 180도까지 이동
  for(int i = 0; i <= 180; i++){
    int pul = map(i, 0, 180, 150, 600); // 0~180도 값을 150~600 PWM 값으로 변환
    pwm.setPWM(0, 0, pul); // 채널 , pwm 숫자1, pwm 숫자2  뒤에가 얼마나 많이 해줄꺼지에 대한 내용
    pwm.setPWM(1, 0, pul); // 첫 번째 서보 모터(채널 0)에 PWM 신호 보내기
    pwm.setPWM(2, 0, pul); // 첫 번째 서보 모터(채널 0)에 PWM 신호 보내기
    pwm.setPWM(3, 0, pul); // 첫 번째 서보 모터(채널 0)에 PWM 신호 보내기

    pwm.setPWM(12, 0, pul); // 채널 , pwm 숫자1, pwm 숫자2  뒤에가 얼마나 많이 해줄꺼지에 대한 내용
    pwm.setPWM(13, 0, pul); // 첫 번째 서보 모터(채널 0)에 PWM 신호 보내기
    pwm.setPWM(14, 0, pul); // 첫 번째 서보 모터(채널 0)에 PWM 신호 보내기
    pwm.setPWM(15, 0, pul); // 첫 번째 서보 모터(채널 0)에 PWM 신호 보내기
    delay(10);
  }
  
  // 1초 대기
  delay(1000);

  // 서보 모터를 180도에서 0도까지 이동
  for(int i = 180; i >= 0; i--){
    int pul = map(i, 0, 180, 150, 600); // 0~180도 값을 150~600 PWM 값으로 변환
    pwm.setPWM(0, 0, pul); // 채널 , pwm 숫자1, pwm 숫자2  뒤에가 얼마나 많이 해줄꺼지에 대한 내용
    pwm.setPWM(1, 0, pul); // 첫 번째 서보 모터(채널 0)에 PWM 신호 보내기
    pwm.setPWM(2, 0, pul); // 첫 번째 서보 모터(채널 0)에 PWM 신호 보내기
    pwm.setPWM(3, 0, pul); // 첫 번째 서보 모터(채널 0)에 PWM 신호 보내기

    pwm.setPWM(12, 0, pul); // 채널 , pwm 숫자1, pwm 숫자2  뒤에가 얼마나 많이 해줄꺼지에 대한 내용
    pwm.setPWM(13, 0, pul); // 첫 번째 서보 모터(채널 0)에 PWM 신호 보내기
    pwm.setPWM(14, 0, pul); // 첫 번째 서보 모터(채널 0)에 PWM 신호 보내기
    pwm.setPWM(15, 0, pul); // 첫 번째 서보 모터(채널 0)에 PWM 신호 보내기  
    delay(10);
  }
  // 1초 대기
  delay(1000);
}
