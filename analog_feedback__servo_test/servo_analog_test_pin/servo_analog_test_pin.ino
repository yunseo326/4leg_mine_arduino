//아두이노의 서보모터 내장 라이브러리
#include <Servo.h>

//서보모터 1개당 인스턴스 1개 필요
Servo myservo;

//아두이노의 디지털2번핀에 서보모터 제어핀이 연결되어있고
//아날로그 피드백선은 아날로그0번핀에 연결되어있다.
#define servo_pin 2
#define feedback A0

//0.1초간격을 유지하기 위한 (시간을 저장할)변수
unsigned long t = 0;

int degree = 0;

void setup() {
  Serial.begin(9600); //9600의 통신속도로 PC와 통신하겠다!
  //서보모터가 연결된핀을 아두이노가 제어하겠다!
  myservo.attach(servo_pin);
}

void loop() {
  //(작업1번) 0.1초 간격으로 아날로그0번핀의 값을 측정해서 시리얼모니터에 출력한다!
  if(millis() - t > 100){
    t = millis();
    int analog = analogRead(feedback);
    Serial.print(degree);
    Serial.print(", ");
    Serial.println(analog);
  }
  
  
  
  //(작업2번)PC에서 0~180의 각도값을 입력받아서 서보모터를 제어하겠다!
  if(Serial.available()){
    //PC에서 뭔가 아두이노쪽으로 전송되었다!(LF)
    String data = Serial.readStringUntil('\n');
    degree = data.toInt(); //문자열을 숫자로 바꿔주세요!
    myservo.write(degree); //0~180
  }
}
