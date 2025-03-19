//아두이노의 서보모터 내장 라이브러리
#include <Servo.h>

//서보모터 1개당 인스턴스 1개 필요
Servo myservo;

//아두이노의 디지털2번핀에 서보모터 제어핀이 연결되어있고
//아날로그 피드백선은 아날로그0번핀에 연결되어있다.
#define servo_pin 2
#define feedback A0
int degree_min = 54; //서보의 각도가 0일때 아날로그값
int degree_max = 587; //서보의 각도가 180일때 아날로그값

//0.1초간격을 유지하기 위한 (시간을 저장할)변수
unsigned long t = 0;

int degree = 0;

//Mux control pins
int s0 = 8;
int s1 = 9;
int s2 = 10;
int s3 = 11;

//Mux in “SIG” pin
int SIG_pin = 0;

void setup(){

Serial.begin(9600); //9600의 통신속도로 PC와 통신하겠다!
myservo.attach(servo_pin);

pinMode(s0, OUTPUT);
pinMode(s1, OUTPUT);
pinMode(s2, OUTPUT);
pinMode(s3, OUTPUT);

digitalWrite(s0, LOW);
digitalWrite(s1, LOW);
digitalWrite(s2, LOW);
digitalWrite(s3, LOW);

myservo.write(0); 
delay(1000);
degree_min = readMux(0);
Serial.println(degree_min);  
//각도를180도로 회전하고 1초쉰다음 가변저항값 측정하기!
myservo.write(180); 
delay(1000);
degree_max = readMux(0);
Serial.println(degree_max);  
//기본위치는 90도
myservo.write(90); 

}

void loop(){
  
if(Serial.available()){
  //PC에서 뭔가 아두이노쪽으로 전송되었다!(LF)
  String data = Serial.readStringUntil('\n');
  Serial.println("turn");
  degree = data.toInt(); //문자열을 숫자로 바꿔주세요!
  myservo.write(degree); //0~180
 
  delay(1000);
  int cal_degree = map(readMux(0),degree_min,degree_max,0,180);
  Serial.print(readMux(0));  
  Serial.print(",");  
  Serial.print(degree);  
  Serial.print(",");  
  Serial.println(cal_degree);  
  
//  for(int i = 0; i < 16; i ++){ 
//  Serial.print("Value at channel "); 
//  Serial.print(i); Serial.print(": "); 
//  Serial.println(readMux(i)); 
//  delay(10); 
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
