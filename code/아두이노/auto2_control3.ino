#include <Servo.h>
#include <SoftwareSerial.h> //소프트웨어시리얼 라이브러리 추가

Servo myServo;

const int ENABLE1 = 6;
const int DIR1 = 9;
const int DIR2 = 10;
const int ENABLE2 = 5;
const int DIR3 = 7;
const int DIR4 = 8;
const int Relay = 4;
const int trigPin = 2;
const int echoPin = 3;

void setup() {
  pinMode(ENABLE1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);

  pinMode(ENABLE2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(DIR4, OUTPUT);

  digitalWrite(ENABLE1, LOW);
  digitalWrite(ENABLE2, LOW);

  Serial.begin(9600);

  myServo.attach(11); 
  pinMode(Relay,OUTPUT);
  digitalWrite(Relay,HIGH); //이건 임시방편

  // 초음파 센서 핀 설정
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  int light = analogRead(A0);
  // Serial.println(light);
  if (light<950){
    digitalWrite(Relay,HIGH);
  }
  else{
    digitalWrite(Relay,LOW);
  }

  if (Serial.available() > 0) {
    char command = Serial.read(); // 시리얼로부터 명령 읽기

    if (command == 'S') {
      int angle = Serial.parseInt(); // 각도 읽기
      myServo.write(angle); // 서보 모터 각도 설정
    }
    else if (command == 'D') {
      int speed = Serial.parseInt(); // 속도 읽기 (0~255)
      int value = map(speed, -100, 100, -255, 255);
      if (value > 0 && value <= 155) {
        analogWrite(ENABLE1, value+100);
        analogWrite(ENABLE2, value+100);
        digitalWrite(DIR1, HIGH);
        digitalWrite(DIR2, LOW);
        digitalWrite(DIR3, HIGH);
        digitalWrite(DIR4, LOW);
      }
      else if (value < 0 && value >= -155){
        analogWrite(ENABLE1, abs(value)+100);
        analogWrite(ENABLE2, abs(value)+100);
        digitalWrite(DIR1, LOW);
        digitalWrite(DIR2, HIGH);
        digitalWrite(DIR3, LOW);
        digitalWrite(DIR4, HIGH);
      }
      else if (value > 155) {
        analogWrite(ENABLE1, 255);
        analogWrite(ENABLE2, 255);
        digitalWrite(DIR1, HIGH);
        digitalWrite(DIR2, LOW);
        digitalWrite(DIR3, HIGH);
        digitalWrite(DIR4, LOW);
      }
      else if (value < -155){
        analogWrite(ENABLE1, 255);
        analogWrite(ENABLE2, 255);
        digitalWrite(DIR1, LOW);
        digitalWrite(DIR2, HIGH);
        digitalWrite(DIR3, LOW);
        digitalWrite(DIR4, HIGH);
      }
      else{
        analogWrite(ENABLE1, 0);
        analogWrite(ENABLE2, 0);
      }
    }
  }
  // digitalWrite(trigPin, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(trigPin, LOW);

  // long duration = pulseIn(echoPin, HIGH);
  // float distance = (duration / 2.0) * 0.0344; // 거리 계산 (cm)

  // // Serial.print("D");
  // Serial.println(distance); // 거리값을 시리얼로 전송
  // delay(10);
  // if(distance < 20){
  //   analogWrite(ENABLE1, 0);
  //   analogWrite(ENABLE2, 0);
  // }

  // if( distance < 20){
  //   analogWrite(ENABLE1, 0);
  //   analogWrite(ENABLE2, 0);
  // }
}