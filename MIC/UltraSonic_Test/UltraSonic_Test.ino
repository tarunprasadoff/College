const int trigPin_1=6;
const int echoPin_1=7;
float t_1;
float d_1;
void setup() {
  // put your setup code here, to run once:
pinMode(trigPin_1,OUTPUT);
pinMode(echoPin_1,INPUT);
Serial.begin(9600);
digitalWrite(trigPin_1,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(trigPin_1,HIGH);
delayMicroseconds(10);
digitalWrite(trigPin_1,LOW);
t_1=pulseIn(echoPin_1,HIGH);
d_1=t_1*0.034/2;
Serial.println(d_1);
}
