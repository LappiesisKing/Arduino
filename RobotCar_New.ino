/*
  Data Array positions received from remote control:
  
  [0] Right Joystick (X) - U2 Sterring 
  [1] Left Joystick (Y) - U1 Motor drive  
  [2] A button - LED color
  [3] B button - Manual mode
  [4] C button - Auto mode
  [5] D button - Buzzer
  [6] Pot R1 - U1 Joystick fine adjustment
  [7] Pot R6 - U2 Joystick fine adjustment
  [8] Right Joystick (Y) - U2 Ultrasonic sensor servo position
  [9] Left Joystick (X) - U1 (to be implemented)
*/

#include <NewPing.h>
#include <SPI.h>
#include "RF24.h"
#include <Servo.h>
 
#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define MAX_DISTANCE 200

#define FORWARD HIGH
#define BACKWARD LOW

const int dirAPin = 7;                                        // define pin used to control rotational direction of motor A
const int pwmAPin = 6;                                        // define pin for PWM used to control rotational speed of motor A
const int dirBPin = 4;                                        // define pin used to control rotational direction of motor B
const int pwmBPin = 5;                                        // define pin for PWM used to control rotational speed of motor B

RF24 radio(9, 10);                                            // defines the object to control NRF24L01
byte addresses[5] = "00007";                                  // defines communication address which should correspond to remote control
int data[10]={512, 512, 0, 0, 1, 1, 512, 512, 512, 512};      // defines the array used to save the communication data with Right Xaxis added

Servo dirServo;                                               // defines servo to control turning of smart car
int dirServoPin = 2;                                          // defines pin for signal line of the last servo
float dirServoOffset = 6;                                     // defines a variable for deviation (degree) of the servo

// NewPing setup of pins and maximum distance
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 
 
void setup() {
  pinMode(dirAPin, OUTPUT);                                   // sets dirAPin to output mode
  pinMode(pwmAPin, OUTPUT);                                   // sets pwmAPin to output mode
  pinMode(dirBPin, OUTPUT);                                   // sets dirBPin to output mode
  pinMode(pwmBPin, OUTPUT);                                   // sets pwmBPin to output mode

  
  dirServo.attach(dirServoPin);                               // attaches the servo on servoDirPin to the servo object
  dirServo.write(90);                                         // moves dirServo to 90 deg position (center)
  
  Serial.begin(9600);
}

void drive(byte dirServoDegree, bool motorDir, byte motorSpd) {
  dirServo.write(dirServoDegree);            // sets sterring servo position
  digitalWrite(dirAPin, motorDir);                            // defines direction of drive motor A rotation (forward / reverse)
  digitalWrite(dirBPin, motorDir);                            // defines direction of drive motor B rotation (forward / reverse)
  analogWrite(pwmAPin, motorSpd);                             // defines speed of drive motor B rotation (0 - 255) based on defined speed (spd = 128)
  analogWrite(pwmBPin, motorSpd);                             // defines speed of drive motor B rotation (0 - 255) based on defined speed (spd = 128)
}
 
void loop() {
   delay(50);
   unsigned int distance = sonar.ping_cm();
   Serial.print(distance);
   Serial.println("cm"); 

   if (distance > 0 and distance < 30) 
    {
     // drive(90,  BACKWARD, 0);
      drive(135-(data[7] - 512) / 12, BACKWARD, 128);

        for(int i=0;i<10;i++){                                // runs the car in reverse and steered for 1s
          delay(100);                                         // waits 100 ms
        }
    }
    else
    {
      drive(95, FORWARD, 128);
    }
 
 }
