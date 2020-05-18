
// note: for my beam to be horizontal, Servo Motor angle should be 102 degrees.

int last_reading;
#include<Servo.h>
#include<PID_v1.h>


const int servoPin = 5;                                               //Servo Pin

const int TrigPin = 6;

const int EchoPin = 7;

 
 
float Kp = 0.58;                                                      //Initial Proportional Gain
float Ki = 0.15;                                                      //Initial Integral Gain
float Kd = 0.35;                                                    //Intitial Derivative Gain
double Setpoint, Input, Output, ServoOutput;      
int i,j,s=21.5;                                 



PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);           //Initialize PID object, which is in the class PID.
                                                                      
                                                                     
                                                                     
                                                                     
Servo myServo;                                                       //Initialize Servo.


void setup() {
 
  Serial.begin(9600);                                                //Begin Serial 
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  
  myServo.attach(servoPin);                                          //Attach Servo

  Input = readPosition();                                            //Calls function readPosition() and sets the balls
                                                                     //  position as the input to the PID algorithm

  
 
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(-15,15);                                     //Set Output limits to -80 and 80 degrees. 
}

void loop()
{
   if(Serial.available()>0)
   {     
      String data= Serial.readString(); // reading the data received from the bluetooth module
      i=(data[0]-'0')*10; 
      j=data[1]-'0';
      s=i+j;
   } 
  Setpoint =s;
  Input = readPosition();                                            


 if(Input<45)
 {
  myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees 
  ServoOutput=85+Output;                                            // 102 degrees is my horizontal 
  myServo.write(ServoOutput);                                        //Writes value of Output to servo
 }
  
}
      
      
      

float readPosition() {
  //delay(20);                                                            //Don't set too low or echos will run into eachother.      
  
double duration, cm;
double now = millis();
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  duration = pulseIn(EchoPin, HIGH);
  cm = duration/(29*2);
 /* if (cm <7 ) {cm=6; Serial.println("Mazen");}*/
  Serial.println(cm);
  
  return cm;                                          //Returns distance value.
}
