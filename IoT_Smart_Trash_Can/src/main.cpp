#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h> 

#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#define AIO_USERNAME  "SmartTrashCan"
#define AIO_KEY       "aio_UJVn26FOOOAQZAqOZlDbZWO54NXE"
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883

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
const char* outVOC = "/sensor/VOC";
char vocData [50];
const char* outFilling = "/sensor/fillingLevel";
char fillingData [50];
const char* outCriticalWarning = "/limits/warning";
char criticalWarningData [1];

long last_time;
long now;

Adafruit_MQTT_Client mqttAdafruit(&wifiClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish fillingLevelAdafruit = Adafruit_MQTT_Publish(&mqttAdafruit, AIO_USERNAME "/feeds/sensor/fillingLevel");

//Sensor measurement variables
float sensorTemperature;
float sensorHumidity;
float sensorVOC;
int sensorFillingLevel;

//Limits
float limitHumidity = 50;
float limitVOC = 3000;
boolean criticalWarning = false;

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

//Publish VOC data to MQTT
void publishVOC(float VOC){
    snprintf(vocData, sizeof(vocData), "%f", VOC);
    Serial.print("Sending VOC: ");
    Serial.println(vocData);
    mqttClient.publish(outVOC, vocData);
}

//Publish critical warning to MQTT
void publishCriticalWarning(boolean warning){
    sprintf(criticalWarningData, "%i", warning);
    Serial.print("Sending critical warning: ");
    Serial.println(criticalWarningData);
    mqttClient.publish(outCriticalWarning, criticalWarningData);
}


/*
//Adafruit MQTT connect function
void connectMQTTAdafruit(){
    mqttAdafruit.connect();
  if (mqttAdafruit.connected()) {
    Serial.println("Connected to Adafruit IO-MQTT broker!");
  } else {
    Serial.println("Failed to connect to Adafruit IO-MQTT broker!");
  }
}
*/

void setup() {
  Serial.begin(115200);
 
  //WiFi Setup
  WiFi.begin(ssid, password);

  Serial.println("Deine Mudda");
  mqttAdafruit.connect();
  Serial.println("Dein Vadder");

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
  //For Adafruit IO connection
  //if (!mqttAdafruit.connected()){
    //connectMQTTAdafruit();
  //}

  mqttClient.loop();

  now = millis();
  if (now - last_time > 5000)
  {
      sensorTemperature = 1.353456836 * random(30.0); //max. Wert random Funktion 30 factor 1.353456836 for float generation, random only creates integer
      sensorHumidity = 1.353456836 * random(70.0); //max. Wert random Funktion 70
      sensorVOC = 1.68364967 * random(2000.0);
      sensorFillingLevel = random(100.0); 
      publishTemperature(sensorTemperature);
      publishHumidity(sensorHumidity);
      publishVOC(sensorVOC);
      publishFillingLevel(sensorFillingLevel);

      if(sensorHumidity > limitHumidity || sensorVOC > limitVOC){
        criticalWarning = true;
        publishCriticalWarning(criticalWarning);
      }
      else{
        criticalWarning = false;
        publishCriticalWarning(criticalWarning);
      }


      fillingLevelAdafruit.publish(sensorFillingLevel);

      last_time = now;
  }
  
}