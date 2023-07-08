#include <stdlib.h>
#include <stdio.h>
#define Lencoder0PinA 2
#define Lencoder0PinB 3
#define LPWM 5
#define IN1 7
#define IN2 6
#define Rencoder0PinA 21
#define Rencoder0PinB 20
#define RPWM 8
#define IN3 10
#define IN4 9
#include <MsTimer2.h>

float Lencoder0Pos=0;
float Rencoder0Pos=0;
float prevT=0;
float Leprev=0;
float Reprev=0;
float Leprev2=0;
float Reprev2=0;
float Lrpm,Lspeed,Leintegral,Leintegral2;
float Rrpm,Rspeed,Reintegral,Reintegral2;
float Kp=7;
float Kd=3;
float Ki=6;
float timer=100;
float L_T=10;
float L_mr=10;
float v=10;
int theta=0;
String theta_string = "";
bool stringComplete = false;



void setup() {
  pinMode(Lencoder0PinA,INPUT);
  pinMode(Lencoder0PinB,INPUT);
  pinMode(Rencoder0PinA,INPUT); 
  pinMode(Rencoder0PinB,INPUT);
  pinMode(LPWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(RPWM,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  attachInterrupt(0,LdoEncoderA,CHANGE);
  attachInterrupt(1,LdoEncoderB,CHANGE);
  attachInterrupt(2,RdoEncoderA,CHANGE);
  attachInterrupt(3,RdoEncoderB,CHANGE);

  MsTimer2::set(timer,timerISR);
  MsTimer2::start();
  Serial.begin (9600);
  theta_string.reserve(200);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir==1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir==-1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
}
void LdoEncoderA(){
  if (digitalRead(Lencoder0PinA)==HIGH) { 
    if (digitalRead(Lencoder0PinB)==LOW) {  
      Lencoder0Pos=Lencoder0Pos+1;
    } 
    else {
      Lencoder0Pos=Lencoder0Pos-1;
    }
  }
  else             
  { 
    if (digitalRead(Lencoder0PinB)==HIGH) {   
      Lencoder0Pos=Lencoder0Pos+1;
    } 
    else {
      Lencoder0Pos=Lencoder0Pos-1;
    }
  }
}

void LdoEncoderB(){
  if (digitalRead(Lencoder0PinB)==HIGH) {   
    if (digitalRead(Lencoder0PinA)==HIGH) {  
      Lencoder0Pos=Lencoder0Pos+1;
    } 
    else {
      Lencoder0Pos=Lencoder0Pos-1;
    }
  }
  else { 
    if (digitalRead(Lencoder0PinA)==LOW) {   
      Lencoder0Pos=Lencoder0Pos+1;
    } 
    else {
      Lencoder0Pos=Lencoder0Pos-1;
    }
  }
}

void RdoEncoderA(){
  if (digitalRead(Rencoder0PinA)==HIGH) { 
    if (digitalRead(Rencoder0PinB)==LOW) {  
      Rencoder0Pos=Rencoder0Pos+1;
    } 
    else {
      Rencoder0Pos=Rencoder0Pos-1;
    }
  }
  else             
  { 
    if (digitalRead(Rencoder0PinB)==HIGH) {   
      Rencoder0Pos=Rencoder0Pos+1;
    } 
    else {
      Rencoder0Pos=Rencoder0Pos-1;
    }
  }
}

void RdoEncoderB(){
  if (digitalRead(Rencoder0PinB)==HIGH) {   
    if (digitalRead(Rencoder0PinA)==HIGH) {  
      Rencoder0Pos=Rencoder0Pos+1;
    } 
    else {
      Rencoder0Pos=Rencoder0Pos-1;
    }
  }
  else { 
    if (digitalRead(Rencoder0PinA)==LOW) {   
      Rencoder0Pos=Rencoder0Pos+1;
    } 
    else {
      Rencoder0Pos=Rencoder0Pos-1;
    }
  }
}

void CheckEncoder(){
  Lrpm=Lencoder0Pos*(60000/timer)/2200;
  Lspeed=Lrpm*6.8*PI/60;
  Lencoder0Pos=0;

  Rrpm=Rencoder0Pos*(60000/timer)/2200;
  Rspeed=Rrpm*6.8*PI/60;
  Rencoder0Pos=0;
}


void timerISR(){
  CheckEncoder();
}


void loop() {

  if (stringComplete){
    theta = theta_string.toInt();
    theta_string = "";
    stringComplete=false;
  }
  
  if(theta>=0){
    float V_l=(1+L_mr*(abs(theta)*PI/180)/(2*L_T))*v;
    float V_r=-(1-L_mr*(abs(theta)*PI/180)/(2*L_T))*v;
    long currT = micros();
    float deltaT=((float)(currT-prevT))/(1.0e6);
    prevT=currT;

    float Le=Lspeed-V_l;
    float Ldedt=(Le-Leprev)/(deltaT);
    Leintegral=Leintegral+Le*deltaT;
    float Lu=Kp*Le+Kd*Ldedt+Ki*Leintegral;

    float Re=Rspeed-V_r;
    float Rdedt=(Re-Reprev)/(deltaT);
    Reintegral=Reintegral+Re*deltaT;
    float Ru=Kp*Re+Kd*Rdedt+Ki*Reintegral;

    int Ldir=1;
    if (Lu<0){
      Ldir=-1;
    }

    float Lpwr=fabs(Lu);
    if(Lpwr>255){
     Lpwr=255;
    }

    int Rdir=1;
    if (Ru<0){
     Rdir=-1;
    }

    float Rpwr=fabs(Ru);
    if(Rpwr>255){
     Rpwr=255;
    }
    setMotor(Ldir,Lpwr,LPWM,IN1,IN2);
    setMotor(Rdir,Rpwr,RPWM,IN3,IN4);
    Leprev=Le;
    Reprev=Re;
  }
  
  else if(theta<0){
    float V_r=-(1+L_mr*(abs(theta)*PI/180)/(2*L_T))*v;
    float V_l=(1-L_mr*(abs(theta)*PI/180)/(2*L_T))*v;
    long currT = micros();
    float deltaT=((float)(currT-prevT))/(1.0e6);
    prevT=currT;

    float Le2=Lspeed-V_l;
    float Ldedt2=(Le2-Leprev2)/(deltaT);
    Leintegral2=Leintegral2+Le2*deltaT;
    float Lu2=Kp*Le2+Kd*Ldedt2+Ki*Leintegral2;

    float Re2=Rspeed-V_r;
    float Rdedt2=(Re2-Reprev2)/(deltaT);
    Reintegral2=Reintegral2+Re2*deltaT;
    float Ru2=Kp*Re2+Kd*Rdedt2+Ki*Reintegral2;

    int Ldir=1;
    if (Lu2<0){
      Ldir=-1;
    }

    float Lpwr=fabs(Lu2);
    if(Lpwr>255){
     Lpwr=255;
    }

    int Rdir=1;
    if (Ru2<0){
     Rdir=-1;
    }

    float Rpwr=fabs(Ru2);
    if(Rpwr>255){
     Rpwr=255;
    }
    setMotor(Ldir,Lpwr,LPWM,IN1,IN2);
    setMotor(Rdir,Rpwr,RPWM,IN3,IN4);
    Leprev2=Le2;
    Reprev2=Re2;
  }
}

  

void serialEvent(){
  while(Serial.available()){
    char theta1 = (char)Serial.read();
    theta_string += theta1;
    if(theta1 == '\n'){
      stringComplete = true;
    }
  }
}