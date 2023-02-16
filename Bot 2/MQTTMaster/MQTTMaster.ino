#include <MPU6050_tockn.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

MPU6050 mpu6050(Wire);

const int trigPin = 5;    // defines pins for HC-SR04
const int echoPin = 18;
#define SOUND_SPEED 0.034

const char* ssid = "b07car";      //Defines ssid, password and ip for broker
const char* password = "eeebot07";
const char* mqtt_server = "192.168.2.1";

float error;
char buzzword[8];   //buzzword is a global variable as it needs to be changed on recieve message and then trasmitted iun the loop function

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
  Wire.begin();             // join i2c bus
  Serial.begin(115200);     //begins serial communication   
  pinMode(trigPin, OUTPUT); // sets trigger (5) as output from esp to sensor
  pinMode(echoPin, INPUT); // sets echo pin (18) as input to esp from the sensor
  pinMode (27, INPUT);
  pinMode (26, INPUT);
  pinMode (25, INPUT);
  pinMode (33, INPUT);
  pinMode (32, INPUT);
  pinMode (35, INPUT);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageAngle;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageAngle += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/motors") {
    Serial.print("Changing motors to ");
    if(messageAngle == "on"){
      strcpy(buzzword, "for"); 
    }
    else if(messageAngle == "off"){
      strcpy(buzzword, "stp");
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/motors");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {

  if (!client.connected()) {        //attempt reconnection if client connection lost
    reconnect();
  }
  client.loop();

  float sensorArray[2][6];
  float denominator = 0, numerator = 0, avg = 0, prevError, sumError, u; 
  const int setpoint = 18;
  int i, servoAngle;
  char stringAngle[3], transmission[20], encoderDisStr[8];
  int encoderDistance;

  sensorArray[0][0] = analogRead(35);   //fills 2d array witg Sensor values and positions
  sensorArray[0][1] = analogRead(32);
  sensorArray[0][2] = analogRead(33);
  sensorArray[0][3] = analogRead(25);
  sensorArray[0][4] = analogRead(26);
  sensorArray[0][5] = analogRead(27);
  sensorArray[1][0] = 4;
  sensorArray[1][1] = 10;
  sensorArray[1][2] = 16;
  sensorArray[1][3] = 22;
  sensorArray[1][4] = 28;
  sensorArray[1][5] = 34;  
  
  for (i=0; i<6; i++)       //creating the variables used in weighted average calculation
  {
    numerator += sensorArray[0][i]*sensorArray[1][i];
    denominator += sensorArray[0][i];
  }
  
  avg = numerator/denominator;    //created weighted average
  
  prevError = error;      //PID calculations
  error = setpoint - avg;
  sumError += error;

  u = (error) + (sumError) + (error - prevError);

  servoAngle = 55 - u;


  gcvt(servoAngle, 10, stringAngle);    
  if (servoAngle < 100)
    strcat(stringAngle, "-");   //adds a - at the end of the angle string so that if the angle is only 2 digits, it still takes 3 characters
  else if (servoAngle < 10)
    strcat(stringAngle, "--");  //adds a -- at the end of the angle string so that if the angle is only 1 digit, it still takes 3 characters 
  strcpy(transmission, stringAngle);
  strcat(transmission, buzzword);  //concatinates the servo angle and buzzword
  Wire.beginTransmission(0x08);
  Wire.write(transmission);    
  Wire.endTransmission();
  

  Wire.requestFrom(0x08,1);  //requests 1 piece of data from the arduino nano
  if (Wire.available()) {	      // reads response from arduino slave
		encoderDistance = Wire.read();
    encoderDistance -= 48;
	}

  dtostrf(encoderDistance, 3,0, encoderDisStr);
  client.publish("esp32/distanceCovered", encoderDisStr);   //transmits distance traveled as a message with specified topic


  long duration;
  float distance;
  char disSensedString[8];
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distance = duration * SOUND_SPEED/2;
  dtostrf(distance, 4,2, disSensedString);
  client.publish("esp32/distanceSensed", disSensedString);
  
  delay(50);
}
