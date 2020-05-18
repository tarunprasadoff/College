#include<Servo.h>
Servo servo;
const int trigPin_1=6;
const int echoPin_1=7;

//const int trigPin_2=8;
const int echoPin_2=9;//

float t_1;
float d_1;
float t_2;
float d_2;
float x,x0,e,ed=0,kp,kd,ki,s,theta,ei=0,e0=0;



void setup() {
  // put your setup code here, to run once:
servo.attach(5);
pinMode(trigPin_1,OUTPUT);
//pinMode(trigPin_2,OUTPUT);
pinMode(echoPin_1,INPUT);
//pinMode(echoPin_2,INPUT);
Serial.begin(9600);
digitalWrite(trigPin_1,LOW);
//digitalWrite(trigPin_2,LOW);
}

void loop() {
  // put your main code here, to run repeatedly://
s=21.5;
kp=-.5;
kd=-.325;
ki=-.25*0;
digitalWrite(trigPin_1,HIGH);
delayMicroseconds(10);
digitalWrite(trigPin_1,LOW);
t_1=pulseIn(echoPin_1,HIGH);
d_1=t_1*0.034/2;

x=d_1+1;
if (e0==0){
  x0=x;
}


if (abs(x-x0)>60){
  x=x0;
}
x0=x;

e=-s+x;

if (e>46) {
  e=4.25-21.5;
}
if (x<2) {
  e=18;
}
ei=ei+(e*.005);
if (e0!=0){
  ed=((e-e0)/0.005);
}
theta=-(kp*e+kd*ed+ki*ei);
if (abs(theta)>15){
  theta=(15*theta)/abs(theta);
}

Serial.print(e,2);
Serial.print(" ERROR  ");
Serial.print(-kp*e,2);
Serial.print(" THETA-P ");
Serial.print(-kd*ed,2);
Serial.print(" THETA-D ");
Serial.print(-ki*ei,2);
Serial.print(" THETA-I ");
Serial.print(-(kp*e+kd*ed+ki*ei),2);
Serial.print(" SUM ");
Serial.print(theta,2);
Serial.print(" THETA ");
Serial.println();
e0=e;
servo.write(98+theta);
delay(5);
    
}
