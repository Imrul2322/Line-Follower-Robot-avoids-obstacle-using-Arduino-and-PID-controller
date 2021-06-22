#include "MotorClassL.h"

MotorClassL::MotorClassL(int pinArray[6]){
    pinA=pinArray[0];
    pinB=pinArray[1];
    pinC=pinArray[2];
    pinD=pinArray[3];
    enA=pinArray[4];
    enB=pinArray[5];

    for(int i=0;i<6;i++)pinMode(pinArray[i],OUTPUT);
}

void MotorClassL::motor1(int INa,int INb,int speed){
	digitalWrite(pinA,INa);
	digitalWrite(pinB,INb);
	analogWrite(enA,speed);
}
void MotorClassL::motor2(int INc,int INd,int speed){
	digitalWrite(pinC,INc);
	digitalWrite(pinD,INd);
	analogWrite(enB,speed);
}
void MotorClassL::motorForward(int a,int b ){
    if(a<0) motor1(0,1,-a);
    else motor1(1,0,a);
    if(b<0) motor2(0,1,-b);
    else motor2(1,0,b);
}
void MotorClassL::motor1Forward(int a){
  if(a<0) motor1(0,1,-a);
  else motor1(1,0,a);
}
void MotorClassL::motor2Forward(int b){
  if(b<0) motor2(0,1,-b);
  else motor2(1,0,b);
}
void MotorClassL::tiltLft(int b){
    motor1(0,1,b);
    motor2(0,0,0);
}
void MotorClassL::tiltRyt(int a){
    motor1(0,0,0);
    motor2(0,1,a);
}
void MotorClassL::brake(){
    motor1(0,0,255);
    motor2(0,0,255);
}
void MotorClassL::brake(int p){
    motor1(0,0,p);
    motor2(0,0,p);
}
void MotorClassL::motor1Brake(){
    motor1(0,0,255);
}
void MotorClassL::motor2Brake(){
    motor2(0,0,255);
}
