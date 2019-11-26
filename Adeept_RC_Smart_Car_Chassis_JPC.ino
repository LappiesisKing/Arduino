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
  [8] Left Joystick (X) - U1 (to be implemented)
*/

#include <SPI.h>
#include "RF24.h"
#include <Servo.h>

RF24 radio(9, 10);                                            // defines the object to control NRF24L01
byte addresses[5] = "00007";                                  // defines communication address which should correspond to remote control
// int data[9]={512, 512, 0, 0, 1, 1, 512, 512, 512};         // defines the array used to save the communication data
int data[10]={512, 512, 0, 0, 1, 1, 512, 512, 512, 512};      // defines the array used to save the communication data with Right Xaxis added
int mode[1];

Servo dirServo;                                               // defines servo to control turning of smart car
int dirServoPin = 2;                                          // defines pin for signal line of the last servo
float dirServoOffset = 6;                                     // defines a variable for deviation (degree) of the servo

Servo ultrasonicServo;                                        // define servo to control turning of ultrasonic sensor
int ultrasonicPin = 3;                                        // define pin for signal line of the last servo
int trigPin = 0;                                              // define Trig pin for ultrasonic ranging module
int echoPin = 1;                                              // define Echo pin for ultrasonic ranging module

float maxDistance = 200;                                      // define the operation range (cm) for the ultrasonic ranging module, maximum sensor distance is rated at 400cm

#define FORWARD HIGH
#define BACKWARD LOW

const int dirAPin = 7;                                        // define pin used to control rotational direction of motor A
const int pwmAPin = 6;                                        // define pin for PWM used to control rotational speed of motor A
const int dirBPin = 4;                                        // define pin used to control rotational direction of motor B
const int pwmBPin = 5;                                        // define pin for PWM used to control rotational speed of motor B
const int buzzerPin = 8;                                      // define pin for buzzer
const int RPin = A3; 
const int GPin = A4; 
const int BPin = A5; 
int RGBVal = 0;
int automatic = 0;

// car will run this instruction once when turned on 
void setup() {
  radio.begin();                                              // initialize RF24
  radio.setRetries(0, 15);                                    // set retries times
  radio.setPALevel(RF24_PA_LOW);                              // set power
  radio.openReadingPipe(1, addresses);                        // open delivery channel
  radio.startListening();                                     // sets module as a receiver

  dirServo.attach(dirServoPin);                               // attaches the servo on servoDirPin to the servo object
  dirServo.write(90);                                         // moves dirServo to 90 deg position (center)
  ultrasonicServo.attach(ultrasonicPin);                      // attaches the servo on ultrasonicPin to the servo object
  ultrasonicServo.write(100);                                 // moves ultrasonicServo to 90 deg position (center)
  
  pinMode(dirAPin, OUTPUT);                                   // sets dirAPin to output mode
  pinMode(pwmAPin, OUTPUT);                                   // sets pwmAPin to output mode
  pinMode(dirBPin, OUTPUT);                                   // sets dirBPin to output mode
  pinMode(pwmBPin, OUTPUT);                                   // sets pwmBPin to output mode
  pinMode(buzzerPin, OUTPUT);                                 // sets buzzerPin to output mode
  pinMode(RPin, OUTPUT);                                      // sets RPin to output mode
  pinMode(GPin, OUTPUT);                                      // sets GPin to output mode
  pinMode(BPin, OUTPUT);                                      // sets BPin to output mode
  pinMode(trigPin, OUTPUT);                                   // sets trigPin to output mode
  pinMode(echoPin, INPUT);                                    // sets echoPin to input mode

  Serial.begin(9600);
}

// gets data sent by the remote control
void receiveData(){
   if (radio.available()) {                                   // checks if data / new data has been sent by the remote control
    while (radio.available()) {                               // loops through all data positions
      radio.read( data, sizeof(data) );                       // reads each position from received data
    }
    if(!data[2]){                                             // changes LED color
    RGBVal++ ;
    if(RGBVal>7){
      RGBVal=0;}
    }
    if(!data[3]){                                             // selects remote control mode
      automatic = 0;
    }
    if(!data[4]){                                             // selects autonomos driving mode
      automatic = 1;
    }  
    if (!data[5])                                             // honks the buzzer
      tone(buzzerPin, 2000);
    else
      noTone(buzzerPin);
   }
}

// defines remote control mode parameters
void ctrlCar0(byte dirServoDegree,byte ultrasonicServoDegree, bool motorDir, byte motorSpd) {
  dirServo.write(dirServoDegree + dirServoOffset);            // sets sterring servo position
  ultrasonicServo.write(ultrasonicServoDegree);               // sets ultrasonic servo position
  digitalWrite(dirAPin, motorDir);                            // defines direction of drive motor A rotation (forward / reverse)
  digitalWrite(dirBPin, motorDir);                            // defines direction of drive motor B rotation (forward / reverse)
  analogWrite(pwmAPin, motorSpd);                             // defines speed of drive motor A rotation (0 - 255) based on remote control joystick U1 position
  analogWrite(pwmBPin, motorSpd);                             // defines speed of drive motor B rotation (0 - 255) based on remote control joystick U1 position
}

// defines autonomous driving mode parameters
void ctrlCar1(byte dirServoDegree, bool motorDir, byte motorSpd) {
  dirServo.write(dirServoDegree + dirServoOffset);            // sets sterring servo position
  digitalWrite(dirAPin, motorDir);                            // defines direction of drive motor A rotation (forward / reverse)
  digitalWrite(dirBPin, motorDir);                            // defines direction of drive motor B rotation (forward / reverse)
  analogWrite(pwmAPin, motorSpd);                             // defines speed of drive motor B rotation (0 - 255) based on defined speed (spd = 128)
  analogWrite(pwmBPin, motorSpd);                             // defines speed of drive motor B rotation (0 - 255) based on defined speed (spd = 128)
}

// defines the distance from an object
int getDistance() {
  
  digitalWrite(trigPin, LOW);   
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);  
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);   
  
  float pingTime = pulseIn(echoPin, HIGH);  
  //int sensorDistance = pingTime/58;                       // Divide by 58 for cm or 148 for inches       
  //return sensorDistance;
  
  if (pingTime != 0) {                                      // checks if pulse bounces back to echoPin
    int sensorDistance = pingTime/58;                       // calculates the distance divided by 58 for (cm) or 148 for (inches)       
    return sensorDistance;                                  // returns measured distance(cm)
  }
  else                                 
    return maxDistance;                                     // returns max dstance (200 cm) if pulse does not bounce back to echoPin (no object found)
}


/* 
  NOTE: HC_SR04 ultrasonic sensor only detects objects perpendicular to the sensor. 
  If the object is not perpendicular, pulse will not bounce back to the sensor 
  (angled incidence) and the object will not been seen by the sensor. 
*/


// car will loop through this instruction continuously after turned on 
void loop() {

  // gets data sent by the remote control
  receiveData();

  // run this instruction if remote control mode is active
  if (automatic == 0) {
    mode[0]=0;                                              // mode[0]=0 : remote control mode
 
    // calculates the steering angle of the servos according to the direction of the joystick U2 on the remote control
    int dirServoDegree = map(data[0], 0, 1023, 45, 135) - (data[7] - 512) / 12;
    int ultrasonicServoDegree = map(data[8], 0, 1023, 50, 150);

    // calculates the drive motors speed according to the direction of the joystick U1 on the remote control
    int motorSpd = data[1] - 512 + (data[6] - 512) / 10;
//    Serial.print(motorSpd);
//    Serial.print("*****");
    bool motorDir = motorSpd > 0 ? BACKWARD : FORWARD;
    motorSpd = abs(constrain(motorSpd, -512, 512)); 
 //   Serial.print(motorSpd);
 //   Serial.print("*****");
    motorSpd = map(motorSpd, 0, 512, 0, 255);
 //   Serial.print(motorSpd);
 //   Serial.print("*****");

 if (motorSpd > 100)
{
  motorSpd = 255;
}
else
{
  motorSpd = 0; 
}
      
    // controls the steering and speed of the smart car sent by the remote control (remote control mode)
    ctrlCar0(dirServoDegree, ultrasonicServoDegree, motorDir, motorSpd);
    
    // loops though 7 LED colors and OFF  
    switch (RGBVal) {
      case 0: digitalWrite(RPin, LOW);digitalWrite(GPin, LOW);digitalWrite(BPin, LOW); break;     //White
      case 1: digitalWrite(RPin, LOW);digitalWrite(GPin, LOW);digitalWrite(BPin, HIGH); break;    //Yellow
      case 2: digitalWrite(RPin, LOW);digitalWrite(GPin, HIGH);digitalWrite(BPin, LOW); break;    //Magenta
      case 3: digitalWrite(RPin, LOW);digitalWrite(GPin, HIGH);digitalWrite(BPin, HIGH); break;   //Red
      case 4: digitalWrite(RPin, HIGH);digitalWrite(GPin, LOW);digitalWrite(BPin, LOW); break;    //Cyan
      case 5: digitalWrite(RPin, HIGH);digitalWrite(GPin, LOW);digitalWrite(BPin, HIGH); break;   //Green
      case 6: digitalWrite(RPin, HIGH);digitalWrite(GPin, HIGH);digitalWrite(BPin, LOW); break;   //Blue
      case 7: digitalWrite(RPin, HIGH);digitalWrite(GPin, HIGH);digitalWrite(BPin, HIGH); break;  //Off
      default: break;
      }
  }

  // run this instruction if autonomous drving mode is active
  if (automatic == 1) {
  
      byte barDistance = maxDistance;                         // defines initial distance as max distance (200 cm)
      byte barDegree;                                         // allocates angle of uletrasonic sensor
      byte distance;                                          // allocates measured shortest distance
      mode[0]=1;                                              // mode[0]=1 : autonomous driving mode
      
      ultrasonicServo.write(50);                              // define the initial scanning position servo of pan tilt
      delay(200);                                             // waits 200 ms
      
      // Starts sweeping the area from 40 deg to 160 deg to identify the shortest distance (barDistance) and its direction (barDegree)
      for (byte ultrasonicServoDegree = 40; ultrasonicServoDegree <= 160; ultrasonicServoDegree += 10) {
        ultrasonicServo.write(ultrasonicServoDegree);         // sets ultrasonic servo position 
        delay(50);                                            // waits 50 ms                                         
        receiveData();                                        // checks for new data from the remote control (such as driving mode change or buzzer)  
        distance = getDistance();                             // defines first measured distance
        
        Serial.print(distance);
        Serial.print("*****");
        
        if (distance < barDistance) {                         // checks if distance is shorter than the initial or previous one
          barDegree = ultrasonicServoDegree;                  // defines the direction of the actual measured distance
          barDistance = distance;                             // defines new max distance
        }
      }
    
      delay(200);                                             // waits 200 ms
      ultrasonicServo.write(100);                             // Sets ultrasonicServo to center position before moving the vehicle.
    
      int spd = 128;                                          // defines motor speed for autonomous driving
      
      if (barDistance < 20) {                                 // checks if the distance from object is less than 20cm
        if (barDegree < 100)                                  // checks which side of the car the shortest distance is (< 100 : left | = 100 : in front of the car | > 100 : right)
          ctrlCar1(135-(data[7] - 512) / 12,  BACKWARD, spd); // steers the wheels completely to the left and reverses the car 
        else
          ctrlCar1(45-(data[7] - 512) / 12,  BACKWARD, spd);  // steers the wheels completely to the right and reverses the car
        
        for(int i=0;i<15;i++){                                // runs the car in reverse and steered for 1.5s
          delay(100);                                         // waits 100 ms
          receiveData();                                      // checks for new data from the remote control (such as driving mode change or buzzer)
        }
      }
      
      else if (barDistance >= 20 and barDistance < 50) {        // checks if the distance from object is between 20cm and 50cm
        if (barDegree < 100)                                  // checks which side of the car the shortest distance is (< 100 : left | = 100 : in front of the car | > 100 : right)
          ctrlCar1(135-(data[7] - 512) / 12, BACKWARD, spd);  // steers the wheels completely to the left and reverses the car 
        else
          ctrlCar1(45-(data[7] - 512) / 12, BACKWARD, spd);   // steers the wheels completely to the right and reverses the car 
        
        for(int i=0;i<10;i++){                                // runs the car in reverse and steered for 1.0s
          delay(100);                                         // waits 100 ms
          receiveData();                                      // checks for new data from the remote control (such as driving mode change or buzzer)
        }
      }
      
      else if (barDistance >= 50 and barDistance < 80) {        // checks if the distance from object is between 50cm and 80cm
        if (barDegree < 100)                                  // checks which side of the car the shortest distance is (< 100 : left | = 100 : in front of the car | > 100 : right)
          ctrlCar1(100-(data[7] - 512) / 12, FORWARD, spd);   // steers the wheels by 10 degrees to the left and keeps moving forward 
        else
          ctrlCar1(80-(data[7] - 512) / 12, FORWARD, spd);    // steers the wheels by 10 degrees to the right and keeps moving forward
        
        for(int i=0;i<10;i++){                                // runs the car forward and steered for 1.0s
          delay(100);                                         // waits 100 ms
          receiveData();                                      // checks for new data from the remote control (such as driving mode change or buzzer)
        }
      }
      
      ctrlCar1(90-(data[7] - 512) / 12, FORWARD, spd);        // runs the car straight forward. no objects found within max distance.
   }
  
}
