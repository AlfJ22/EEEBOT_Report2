#include <Wire.h>        // include Wire library
#include <Encoder.h>    //include Encoder library
#include <Servo.h>      //include Servo library

Encoder rearRight(3,12);
Encoder rearLeft(11,2);
#define enA 5 
#define enB 6 
#define INa A0  
#define INb A1  
#define INc A2  
#define INd A3 
byte speedSetting = 0;
#define servoPin 4
Servo myservo;        // create servo object to control a servo
float servoLeft = 8, servoRight = 110, servoCentre = 59;
char avgDisStr[8];



String message;
String command;

void setup() {
  Wire.begin(0x08);             // join i2c bus with address 8
  Wire.onReceive(receiveEvent); // create a receive event
  Wire.onRequest(requestEvent); // creates a request event
  Serial.begin(115200);           // start serial to visualise data 
                               // received via serial monitor in the IDE
  //configure the motor control pins as outputs
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT); 

  motors(120, 153);   //discrepencies in the motors mean one has to be set to a lower speed setting
  //motors have also been set 2/3 of the maximum speed setting to help lessen the effect any potential lag in the system 
  myservo.attach(servoPin);
}

void loop() {
  long encoderLeft = 0, encoderRight = 0; 
  float distanceLeft = 0, distanceRight = 0, avgDistance;
  encoderLeft = rearLeft.read();
  encoderRight = rearRight.read();
  distanceLeft = encoderLeft*0.0078;
  distanceRight = encoderRight*0.0078;
  avgDistance = (distanceRight+distanceLeft/2);
  dtostrf(avgDistance, 3,2, avgDisStr);
}

void requestEvent(){
  Wire.write(avgDisStr);
  Serial.println(avgDisStr);
}

void receiveEvent() {
  message = "";
  command = "";
  int i, angle;
  char angleChar[3];
  while (Wire.available()) {  // loop whilst bus is busy
    char c = Wire.read();     // receive data byte by byte
    message += c;             // form complete string
  }

  for (i=0; i<3; i++){
    angleChar[i] = message[i];
  }

  for (i=3; i<6; i++){
    command += message[i];
  }

  angle = atoi(angleChar); 
  if (command == "for"){  //if the message recieved is "for"
    goForwards();             //call the "goForwards" Function
  }
  else if (command == "stp"){  //if the message recieved is "stp"
    stopMoving();               //call the "stopMoving" function
  }   

  else if (message == "bak"){  //if the message recieved is "bak"
    goBackwards();        //the "goBackwards function is called"    
  }
  
  myservo.write(angle);
} 


void motors(int leftSpeed, int rightSpeed) {
  //set individual motor speed
  //direction is set separately

  analogWrite(enA, leftSpeed);
  analogWrite(enB, rightSpeed);
}

void goForwards() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}

void goBackwards() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void stopMoving() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}