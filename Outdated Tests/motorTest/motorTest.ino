#include <Servo.h>

Servo motorOne;
Servo motorTwo;
Servo motorThree;
Servo motorFour;

int minValue = 0;
int maxValue = 180;
int motorSpeed = minValue;

void setup() {
  Serial.begin(9600);
  
  motorOne.attach(3,minValue,maxValue);
  motorTwo.attach(5,minValue,maxValue);
  motorThree.attach(6,minValue,maxValue);
  motorFour.attach(9,minValue,maxValue);
}

void loop() {

while(Serial.available() > 0){
}

motorSpeed = analogRead(A5);
motorSpeed = map(motorSpeed,0,1023,0,180);
Serial.println(motorSpeed);

motorOne.write(motorSpeed);
motorTwo.write(motorSpeed);
motorThree.write(motorSpeed);
motorFour.write(motorSpeed);

}
