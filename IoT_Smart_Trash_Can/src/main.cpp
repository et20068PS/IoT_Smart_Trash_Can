#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h> 

//WiFi connection
const char* ssid = "LenovoPS";
const char* password =  "#Lu89832";

//MQTT client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient); 
const char* mqttServer = "broker.hivemq.com";
int mqttPort = 1883;
const char* mqttUser = "et20068@lehre.dhbw-stuttgart.de";
const char* mqttPassword = "IoT_MQTT!_2023";
const char* inTopic = "/swa/commands";
const char* outTopic = "/swa/testdaten";

long last_time;
long now;
char data [50];

int count = 0;
char messages [50];
int Testpin = 4;

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message from topic: ");
  Serial.print(topic);
  Serial.print("; message: ");
  
  for (int i = 0; i < length; i++) {
    Serial.print((char) payload[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
 
  //WiFi Setup
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

  //MQTT client
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);

  pinMode(Testpin, INPUT);
}

void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
      Serial.println("Reconnecting to MQTT Broker..");
      String clientId = "ESP32Client-";
      clientId += String(random(0xffff), HEX);
      
      if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
        Serial.println("Connected.");
        // subscribe to topic
        mqttClient.subscribe(inTopic);
      }
      
  }
}

void loop() {


  if (!mqttClient.connected())
  reconnect();
  mqttClient.loop();

  now = millis();
  if (now - last_time > 5000)
  {
      boolean Testsignal = digitalRead(Testpin);

      sprintf(data, "%i", Testsignal);
      Serial.println(Testsignal);
      mqttClient.publish(outTopic, data);

      count++;
      snprintf(messages, 75, "%ld", count);
      Serial.print("Sending messages: ");
      Serial.println(messages);
      mqttClient.publish(outTopic, messages);

      last_time = now;
  }
  
}