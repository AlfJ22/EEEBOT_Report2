/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

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

MPU6050 mpu6050(Wire);

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
float Angx = 0;
float Angy = 0;
float Angz = 0;
float AccX = 0;
float AccY = 0;
float AccZ = 0;
long timer = 0;
// LED Pin
const int ledPin = 4;

void setup() {

  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  //status = bme.begin();  

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
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
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
  mpu6050.update(); 
  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
    

    
    // Convert the value to a char array
    char AngxString[8];
    dtostrf(Angx, 1, 2, AngxString);
    Angx = mpu6050.getAngleX();
    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
    client.publish("esp32/mpuAngX", AngxString);

    // Convert the value to a char array
    char AngyString[8];
    dtostrf(Angy, 1, 2, AngyString);
    Angy = mpu6050.getAngleY();
    Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
    client.publish("esp32/mpuAngY", AngyString);

        // Convert the value to a char array
    char AngzString[8];
    dtostrf(Angz, 1, 2, AngzString);
     Angz = mpu6050.getAngleZ();
     Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
    client.publish("esp32/mpuAngZ",AngzString);

        char AccXString[8];
    dtostrf(AccX, 1, 2, AccXString);
    AccX = mpu6050.getAccX();
    Serial.print("accX : ");Serial.print(mpu6050.getAccX());
    client.publish("esp32/mpuAccX", AccXString);

        char AccYString[8];
    dtostrf(AccY, 1, 2, AccYString);
    AccY = mpu6050.getAccY();
    Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
    client.publish("esp32/mpuAccY", AccYString);

        char AccZString[8];
    dtostrf(AccZ, 1, 2, AccZString);
    AccZ = mpu6050.getAccZ();
     Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
    client.publish("esp32/mpuAccZ", AccZString);
    

    timer = millis();
  }
}
