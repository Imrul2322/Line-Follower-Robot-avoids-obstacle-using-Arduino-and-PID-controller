#ifndef MotorClassL_h
#define MotorClassL_h
#include<Arduino.h>

class MotorClassL{
    
public :
    void motor1(int INa,int INb,int speed);
    void motor2(int INa,int INb,int speed);
    int pinA,pinB,pinC,pinD,enA,enB;
    MotorClassL(int pinArray[6]);
    void motorForward(int a,int b );
    void motor1Forward(int a);
    void motor2Forward(int b);
    void tiltRyt(int a);
    void tiltLft(int b);
    void brake();
    void brake(int p);
    void motor1Brake();
    void motor2Brake();
};

#endif
