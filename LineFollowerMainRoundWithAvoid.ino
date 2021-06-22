//go
#define trigPin 33
#define echoPin 31
/////////////////////////////////////////////////////////////////
#include "MotorClassL.h"
int pinArray[]={30,28,26,24,7,8};
MotorClassL mc(pinArray);

int ML=150,MR=150;
int MLx=200,MRx=200;
int mlSpeed,mrSpeed;
//////////////////////////////////////////////////////////////////
#include <QTRSensors.h>
#define NUM_SENSOR    8
#define TIMEOUT       2500
#define EMITTER_PIN   2
//#define SENSOR_DEBUG
#define SONAR_DEBUG
//#define CALIBON
//int sensorPin[]={34,36,38,40,42,44,46,48};
int sensorPin[]={48,46,44,42,40,38,36,34};
QTRSensorsRC qtrrc((unsigned char[]) {48,46,44,42,40,38,36,34},
  NUM_SENSOR, TIMEOUT, EMITTER_PIN); 
  
//int chk[NUM_SENSOR];
unsigned int sensorD[NUM_SENSOR];
unsigned int sensorA[NUM_SENSOR];
int sensorMin[NUM_SENSOR];
int sensorMax[NUM_SENSOR];
int sensorExPin[]={A8,A9,A10,A11};
int chke[4];
int sensorExD[4];
//////////////////////////////////////////////////////////////////
int lastError=0;
float KP,KD;
boolean isMotorEnabled;
//////////////////////////////////////////////////////////////////

void setup(){
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  sensorInit();
  isMotorEnabled=true;
  mlSpeed = mrSpeed = 0;
  ML=MR=160;
  KP=.93+.03;//.89
  KD=1.75+.04; //1.68
}

void loop(){
  
  //motorTest();
  //digitalWrite(22,HIGH);
  //analogWrite(9,100);
  //mc.motor2(0,1,100);
  //sensorTest();
  /*int x=1;
  x=analogRead(A8);
  Serial.print(x);
  delay(100);*/
  
  
  //mc.motorForward(ML,MR);
  fire();
  pid();
  patternCheck();
  pidCalib();
  Serial.println();/**/
  
  //avoidRight();
}
int patternCheck(){
  //if(chk[0]&&chk[1]&&chk[2]&&chk[3]&&chk[4]&&chk[5]) ;
  if(sensorD[0]&&sensorD[1]&&sensorD[2]&&sensorD[3]&&!sensorD[7]&&sensorD[4])    //111000
  {
    mc.motorForward(ML,MR);
    delay(100+20);
    if(sensorReadEx){
      turnLeftMod();
      //turnLeft();
    }
    else turnLeft();
  }
  else if(sensorD[7]&&sensorD[6]&&sensorD[5]&&sensorD[4]&&!sensorD[0]&&sensorD[3])   //000111
  {
    mc.motorForward(ML,MR);
    delay(100+20);
    if(sensorReadEx){
      turnRightMod();
      //turnRight();
    }
    else turnRight();
  }
  else if(!sensorD[0]&&!sensorD[1]&&!sensorD[2]&&!sensorD[3]&&!sensorD[4]&&!sensorD[5]&&!sensorD[6]&&!sensorD[7]) {
    mc.motorForward(ML,MR);
    delay(100+20);
  }
  /*else if(chk[0]&&chk[1])left();   //110000
  else if(chk[5]&&chk[4])right();    //000011*/
  
  //else if(!chk[0]&&!chk[1]&&!chk[2]&&!chk[3]&&!chk[4]&&!chk[5]) reverse(); 
}

void motorTest(){
 // mc.motorForward(200,200);
// turnRight();
 turnLeft();
  //delay(1500);
  mc.brake();
  delay(1500);
}
void sensorTest(){
  sensorRead();
}
void pidCalib(){
  if(Serial.available()){
    char x=Serial.read();
    if(x=='+'){
      if(KP<3)KP+=.01;
    }
    else if(x=='-'){
      if(KP>0)KP-=.01;
    }
    else if(x=='9'){
      if(KD<3)KD+=.01;
    }
    else if(x=='0'){
      if(KD>0)KD-=.01;
    }
    else if(x=='w'||x=='W'){
      if(ML<250){
        ML+=5;
        MR+=5;
      }
    }
    else if(x=='s'||x=='S'){
      if(ML>0){
        ML-=5;
        MR-=5;
      }
    }
  }
}
void pid(){
  int pos=sensorRead(); //range: 800 to 6400
  int error = ((double)pos/8-450)*250/350; //range: -250 to 250
  #ifdef SENSOR_DEBUG
  Serial.print('\t');
  Serial.print(error);
  Serial.print('\t');
  Serial.print(KP);
  Serial.print('\t');
  Serial.print(KD);
  #endif
  //int memory = (lastError - error);
  int memory = (error - lastError);
  int motorSpeed = KP * error + KD * memory;
  lastError = error;
  
  mlSpeed=ML;
  mrSpeed=MR;
  int maxSpeed=230;
  if((ML+motorSpeed) > maxSpeed){
    mlSpeed=maxSpeed;
    mrSpeed=MR-motorSpeed-(ML+motorSpeed-maxSpeed);
  }
  else if((MR-motorSpeed) > maxSpeed){
    mrSpeed=maxSpeed;
    mlSpeed=ML+motorSpeed-(MR-motorSpeed-maxSpeed);
  }
  else{
    mlSpeed=ML+motorSpeed;
    mrSpeed=MR-motorSpeed;
  }
  
  /*if(motorSpeed<0) mlSpeed = ML + motorSpeed;
  else if(motorSpeed>0) mrSpeed = MR - motorSpeed;
  else {
    mlSpeed=MLx;
    mrSpeed=MRx;
  }
  */
  if (mlSpeed < 0)
    mlSpeed = 0;
  if (mrSpeed < 0)
    mrSpeed = 0;
    
   Serial.print('\t');
   Serial.print(mlSpeed);
   Serial.print('\t');
   Serial.print(mrSpeed);
   Serial.print('\t');
   Serial.print(ML);
   
   if(isMotorEnabled) mc.motorForward(mrSpeed,mlSpeed);
}

void turnLeft()
{
  
    //mc.motorForward(m1Speed,m2Speed);
   // mc.motorForward(M1,M2);
    mc.brake();
    delay(50);
    while(1)
    {
      sensorRead();
      left();
      if(sensorD[1] || sensorD[2] || sensorD[3] )break;
    }
}
void turnRight()
{
  
    //mc.motorForward(M1,M2);
    //delay(100-20);
    mc.brake();
    delay(50);
    while(1)
    {
      sensorRead();
      right();
      if(sensorD[4] || sensorD[5] || sensorD[6]) break;
    }
}
void turnLeftMod()
{
  
    //mc.motorForward(m1Speed,m2Speed);
   // mc.motorForward(M1,M2);
    mc.brake();
    delay(50);
    while(1){
      sensorRead();
      left();
      if(!sensorD[1] && !sensorD[2] && !sensorD[3] )break;
    }
    while(1)
    {
      sensorRead();
      left();
      if(sensorD[3] )break;
    }
    mc.brake();
    delay(500);
}
void turnRightMod()
{
  
    //mc.motorForward(M1,M2);
    //delay(100-20);
    mc.brake();
    delay(50);
    while(1){
      sensorRead();
      right();
      if(!sensorD[4] && !sensorD[5] && !sensorD[6])break;
    }
    while(1)
    {
      sensorRead();
      right();
      if(sensorD[4] ) break;
    }
    mc.brake();
    delay(500);
}
void left()
{
  if(isMotorEnabled)mc.motorForward(-160+40,160-40);
}
void right()
{
  if(isMotorEnabled)mc.motorForward(160-40,-160+40);
}
long fire()
{
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 30000);
  distance = (duration/2) / 29.1;
  
  if(distance>0 && distance<10){
    mc.brake();
    isMotorEnabled=false;
    avoidRight();
  }
  else isMotorEnabled=true;
  
  if (distance >= 200 || distance <= 0){
    //isMotorEnabled=false;
    //nothing
  }
  
  #ifdef SONAR_DEBUG
  Serial.print(distance);
  Serial.println(" cm");
  #endif
  return distance;
}

void avoidRight1(){
  int unitDelay=10;
  int mfl=100,mfr=117,mtl=100,mtr=105;
  int turnLoop=80,ForwardShortLoop=100,ForwardLongLoop=25;
  mc.brake();
  delay(1000);
  for(int i=0;i<turnLoop;i++){ ///////right
    mc.motorForward(mtl,-mtr);
    delay(unitDelay);
  }
  mc.brake();
  delay(300);
  for(int i=0;i<ForwardShortLoop;i++){ ///////forward small
    mc.motorForward(mfl,mfr);
    delay(unitDelay);
  }
  mc.brake();
  delay(300);
  for(int i=0;i<turnLoop;i++){///////left
    mc.motorForward(-mtl,mtr);
    delay(unitDelay);
  }
  mc.brake();
  delay(300);
  for(int i=0;i<ForwardLongLoop;i++){///////forward large
    mc.motorForward(mfl,mfr);
    delay(unitDelay*10);
  }
  mc.brake();
  delay(300);
  for(int i=0;i<turnLoop;i++){///////left
    mc.motorForward(-mtl,mtr);
    delay(unitDelay);
  }
  mc.brake();
  delay(300);
  for(int i=0;i<ForwardShortLoop;i++){///////forward small
    mc.motorForward(mfl,mfr);
    //if(sensorD[7])break;
    delay(unitDelay);
  }
  mc.brake();
  delay(100);
  while(1){
    mc.motorForward(mfl,mfr);
    if(sensorD[7])break;
  }
  mc.brake();
  delay(1000);
  for(int i=0;i<turnLoop;i++){ ///////right
    mc.motorForward(mtl,-mtr);
    delay(unitDelay);
  }
  mc.brake();
  delay(1000);
}

void avoidRight(){
  int unitDelay=10;
  int mfl=100,mfr=117,mtl=100,mtr=105;
  int turnLoop=80,ForwardShortLoop=100,ForwardLongLoop=25;
  mc.brake();
  delay(1000);
  for(int i=0;i<turnLoop;i++){ ///////right
    mc.motorForward(mtl,-mtr);
    delay(unitDelay);
  }
  mc.brake();
  delay(300);
  for(int i=0;i<ForwardShortLoop;i++){ ///////forward small
    mc.motorForward(mfl,mfr);
    delay(unitDelay);
  }
  mc.brake();
  delay(300);
  for(int i=0;i<turnLoop;i++){///////left
    mc.motorForward(-mtl,mtr);
    delay(unitDelay);
  }
  mc.brake();
  delay(300);
  for(int i=0;i<ForwardLongLoop;i++){///////forward large
    mc.motorForward(mfl,mfr);
    delay(unitDelay*10);
  }
  mc.brake();
  delay(300);
  for(int i=0;i<turnLoop;i++){///////left
    mc.motorForward(-mtl,mtr);
    delay(unitDelay);
  }
  mc.brake();
  delay(300);
  for(int i=0;i<ForwardShortLoop+10;i++){///////forward small
    mc.motorForward(mfl,mfr);
    //if(sensorD[7])break;
    delay(unitDelay);
  }
  mc.brake();
  delay(100);
  while(1){
    mc.motorForward(mfl,mfr);
    if(sensorD[7]||sensorD[6]||sensorD[5]||sensorD[4])break;
  }
  mc.brake();
  delay(1000);
  for(int i=0;i<turnLoop;i++){ ///////right
    mc.motorForward(mtl,-mtr);
    delay(unitDelay);
  }
  mc.brake();
  delay(500);
}
