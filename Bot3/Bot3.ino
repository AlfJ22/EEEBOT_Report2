/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
//#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

int i;
float e;  //PID variables are declared here
float eSum = 0;
float ePrevious = 0;
float kp = 0.5;
float ki = 0.5;
float kd = 0.5;
float distance[6] = {44.78, 32.56, 23.77, -23.77, -32.56, -44.78};
//array distance holds values for each sensor's distance from the reference point
int IRs[6] = {13, 12, 14, 27, 35, 34};  //holds data pins used
//an array is used to hold data pins, so that pins can be cycled through in a for loop
float IRb[6];
float IRw[6]; //IRb and IRw are array that will hold sensor values under black/white light

// Replace the next variables with your SSID/Password combination
const char* ssid = "b07car";
const char* password = "eeebot07";                

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.2.1";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

//uncomment the following lines if you're using SPI
/*#include <SPI.h>
#define BME_SCK 18
#define BME_MISO 19
#define BME_MOSI 23
#define BME_CS 5*/

//Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
//float temperature = 0;
//float humidity = 0;

// LED Pin
const int ledPin = 4;

void setup() {
  Serial.begin(9600);
  pinMode(IRs[0], INPUT);
  pinMode(IRs[1], INPUT);
  pinMode(IRs[2], INPUT);
  pinMode(IRs[3], INPUT);
  pinMode(IRs[4], INPUT);
  pinMode(IRs[5], INPUT);
  delay(3000);
  Serial.println("Place over white"); //default black/white values are recorded
  delay(3000);
  IRw[0] = analogRead(IRs[0]);  
  IRw[1] = analogRead(IRs[1]);
  IRw[2] = analogRead(IRs[2]);
  IRw[3] = analogRead(IRs[3]);
  IRw[4] = analogRead(IRs[4]);
  IRw[5] = analogRead(IRs[5]);
  Serial.println("Place over black");
  delay(3000);
  IRb[0] = analogRead(IRs[0]);
  IRb[1] = analogRead(IRs[1]);
  IRb[2] = analogRead(IRs[2]);
  IRb[3] = analogRead(IRs[3]);
  IRb[4] = analogRead(IRs[4]);
  IRb[5] = analogRead(IRs[5]);
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  //status = bme.begin();
  /*
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  */
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(ledPin, OUTPUT);
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
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    //Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      //Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
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
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    
    // Temperature in Celsius
    //temperature = bme.readTemperature();   
    // Uncomment the next line to set temperature in Fahrenheit 
    // (and comment the previous temperature line)
    //temperature = 1.8 * bme.readTemperature() + 32; // Temperature in Fahrenheit
    
    // Convert the value to a char array
    /*
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    client.publish("esp32/temperature", tempString);

    //humidity = bme.readHumidity();
    
    // Convert the value to a char array
    char humString[8];
    dtostrf(humidity, 1, 2, humString);
    Serial.print("Humidity: ");
    Serial.println(humString);
    client.publish("esp32/humidity", humString);
    */

    for (i = 0; i < 6; i++)
    {
      int data;
      int sensor = calibrate(IRs[i], IRb[i], IRw[i]);
      Serial.println(analogRead(IRs[i]));
      Serial.print("Sensor value:\t");
      Serial.println(sensor);
      if (sensor >= 750)
      {
        data = 1;
      }
      else
      {
        data = 0;
      }
      Serial.print("data:\t");
      Serial.println(data);
      char topic[16];
      int sensorNum = i+1;
      sprintf(topic, "Sensor_%d", sensorNum);
      char sensorMessage[16];
      sprintf(sensorMessage, "%d", data);
      Serial.print("Message:\t");
      Serial.println(sensorMessage);
      client.publish(topic, sensorMessage);
    }
    
    //PID:
    e = weightedAverage();  
    //weightedAverage function is called and return value is stored
    Serial.print("Weighted Average:\t");
    Serial.println(e);  
    eSum = eSum + e;
    float up = kp * e;
    float ui = ki * eSum;
    float ud = kd * (e - ePrevious);
    float u =   up + ui + ud;
    char PIDmessage[16];
    dtostrf(u, 1, 2, PIDmessage);
    Serial.print("Proportional Integral Derivative:\t");
    Serial.println(PIDmessage);
    client.publish("esp32/PID", PIDmessage);
    delay(10);
  }
}

/*
calibrate function is declared, with parameters relating to the sensor pin,
 default black value and default white value
*/
int calibrate(int IRsensor, float black, float white)
{
  float IRvalue = analogRead(IRsensor); //current sensor value is recorded
  int sensorValue = constrain(IRvalue, black, white);
  //value is constrained to be within default black/white values
  sensorValue = map(sensorValue, black, white, 0, 1000);  //value is mapped on a range of 0 to 1000
  sensorValue = map(sensorValue, 0, 1000, 1000, 0);
  //map is reversed so a high output is given over black rather than white
  return sensorValue;
}

float weightedAverage()
{
  int i;
  float weightedAv;
  float numerator = 0;  //numerator and denominator variables are declared
  float denominator = 0;
  for (i = 0; i < 6; i++) //for loop is used to take values from each sensor
  {
    float temp = distance[i] * calibrate(IRs[i], IRb[i], IRw[i]);
    //calibrated sensor values are multiplied by their distance from reference point and stored
    numerator = numerator + temp;
    denominator = denominator + calibrate(IRs[i], IRb[i], IRw[i]);
    //calibrated sensor value is added to denominator variable
  }
  weightedAv = numerator/denominator; //weighted average is calculated and returned
  return weightedAv;
}
