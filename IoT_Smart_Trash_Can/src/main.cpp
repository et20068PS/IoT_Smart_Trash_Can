#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h> 

//WiFi connection variables
const char* ssid = "LenovoPS";
const char* password =  "#Lu89832";

//MQTT client variables
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient); 
const char* mqttServer = "broker.hivemq.com";
int mqttPort = 1883;
const char* mqttUser = "et20068@lehre.dhbw-stuttgart.de";
const char* mqttPassword = "IoT_MQTT!_2023";
const char* inTopic1 = "/swa/commands";
const char* inTopic2 = "/swa/info";
const char* outTemperature = "/sensor/temperature";
char temperatureData [50];
const char* outHumidity = "/sensor/humidity";
char humidityData [50];
const char* outFilling = "/sensor/fillingLevel";
char fillingData [50];

long last_time;
long now;

float sensorTemperature;
float sensorHumidity;
int sensorFillingLevel;

//WiFi connect function
void connectWiFi(){
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
}

//MQTT connect function
void connectMQTT() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
      Serial.println("Reconnecting to MQTT Broker..");
      String clientId = "ESP32Client-";
      clientId += String(random(0xffff), HEX);
      
      if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
        Serial.println("Connected.");
        mqttClient.subscribe(inTopic1);
        mqttClient.subscribe(inTopic2);
      }  
  }
}

//Callback function for receiving MQTT data
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message from topic: ");
  Serial.print(topic);
  Serial.print("; message: ");
  
  for (int i = 0; i < length; i++) {
    Serial.print((char) payload[i]);
  }
  Serial.println();
}

//Publish temperature sensor data to MQTT
void publishTemperature(float temperature){
    snprintf(temperatureData, sizeof(temperatureData), "%f °C", temperature);
    Serial.print("Sending temperature: ");
    Serial.println(temperatureData);
    mqttClient.publish(outTemperature, temperatureData);
}

//Publish humidity sensor data to MQTT
void publishHumidity(float humidity){
    snprintf(humidityData, sizeof(humidityData), "%f %", humidity);
    Serial.print("Sending humidity: ");
    Serial.println(humidityData);
    mqttClient.publish(outHumidity, humidityData);
}

//Publish filling level data to MQTT
void publishFillingLevel(int fillingLevel){
    snprintf(fillingData, sizeof(fillingData), "%d %", fillingLevel);
    Serial.print("Sending filling level: ");
    Serial.println(fillingData);
    mqttClient.publish(outFilling, fillingData);
}

void setup() {
  Serial.begin(115200);
 
  //WiFi Setup
  WiFi.begin(ssid, password);

  //MQTT client
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);
}

void loop() {

  if (!WiFi.isConnected()){
    connectWiFi();
  }

  if (!mqttClient.connected()){
    connectMQTT();
  }

  mqttClient.loop();

  now = millis();
  if (now - last_time > 5000)
  {
      sensorTemperature = random(30.0); //max. Wert random Funktion 30
      sensorHumidity = random(70.0); //max. Wert random Funktion 70
      sensorFillingLevel = random(100.0); 
      publishTemperature(sensorTemperature);
      publishHumidity(sensorHumidity);
      publishFillingLevel(sensorFillingLevel);
      last_time = now;
  }
  
}