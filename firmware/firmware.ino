// Based on the original "Hungry Robot"
// https://youtu.be/KfP_LfUiwdc

// Build video:
// 

// Ported to ESP8266

#include <Servo.h>
#include "Sounds.h"

uint8_t ledPin = 2;
uint8_t sensorPin = A0;
uint8_t servoPin = D6;

Servo armServo;

int sensorValue = 0;
int prevSensorValue = 0;
const int THRESHOLD = 360;

void setup() {
  Serial.begin(115200);
  pinMode(D5, OUTPUT);

  // we use pin number D5 for control the robot.
  armServo.attach(servoPin);
  // move the motor to default angle
  armServo.write(90);

  // setup the distance sensor pin
  pinMode(sensorPin, INPUT);

  // setup the servo, speaker, and led pins
  pinMode(servoPin, OUTPUT);
  pinMode(speakerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // release torque from the servo
  delay(1000);
  armServo.detach();
  
  // play activation sound
  sing(sound_connect);  
}
uint32_t counter = 0;
void loop() {
  // read analog value and asign the value into sensorValue
  sensorValue = analog
  Read(sensorPin);
  Serial.println(sensorValue);  

  // these two lines means that some object has come
  // from outside of Threshold to inside of it.
  if(prevSensorValue <= THRESHOLD) {
    if(sensorValue > THRESHOLD) {
      // It's time to action
      eat();
      sing(sound_happy);              
    }
  }

  // regardless the action, save current sensor value prevSensorValue
  // so that we can check the direction
  prevSensorValue = sensorValue;

  // this delay controls how often this loop is running
  // without this delay, this loop runs too fast
  // then, the differences between previous and current sensor value
  // can not be meaningful.
  delay(10);
}

void eat() {
  // call this function for turning on three LEDs all together
  digitalWrite(ledPin, LOW);

  // eating sequence
  // wait 1000ms (1 second)
  delay(1000);

  // let's move the motor to degree '10' (move the arm up)
  armServo.attach(servoPin);  
  armServo.write(5);

  // wait 300 ms
  delay(300);

  // let's move the motor to degree '70' (move the arm down)
  armServo.write(70);
  delay(500);

  // after eating
  delay(100);
  armServo.write(50);
  delay(250);
  armServo.write(70);
  delay(250);
  armServo.write(50);
  delay(250);
  armServo.write(70);
  delay(250);
  armServo.write(50);
  delay(250);
  armServo.write(70);
  delay(250);
  armServo.write(50);
  delay(250);
  armServo.write(90);
  delay(250);

  // release arm's torque
  armServo.detach();

  // turn off the LED
  digitalWrite(ledPin, HIGH);
}
