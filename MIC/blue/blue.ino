#include <Servo.h>

byte servoPin = 9;
Servo servo;

void setup() {
  servo.attach(servoPin);

  servo.writeMicroseconds(1500); // send "stop" signal to ESC.
  delay(7000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
  int signal = 1899; // Set signal value, which should be between 1100 and 1900

  servo.writeMicroseconds(signal); // Send signal to ESC.
}
