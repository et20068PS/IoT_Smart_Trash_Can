#include <Arduino.h>
#include <WiFi.h>

#include <AdafruitIO_WiFi.h>
#include <AdafruitIO_Feed.h>
#define AIO_USERNAME  "SmartTrashCan"
#define AIO_KEY       "aio_UJVn26FOOOAQZAqOZlDbZWO54NXE"
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883

//WiFi connection variables
const char* ssid = "LenovoPS";
const char* password =  "#Lu89832";

//MQTT client variables
WiFiClient wifiClient;

long last_time;
long now;

AdafruitIO_WiFi io(AIO_USERNAME, AIO_KEY, ssid, password);
AdafruitIO_Feed *dataFillingLevel = io.feed("fillingLevel");
AdafruitIO_Feed *dataHumidity = io.feed("humidity");
AdafruitIO_Feed *dataTemperature = io.feed("temperature");
AdafruitIO_Feed *dataVOC = io.feed("VOC");

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


void handleFeedData(AdafruitIO_Data *data) {
  Serial.println("Received feed data:");
  Serial.println(data->value());
}

void setup() {
  Serial.begin(115200);
 
  //WiFi Setup
  WiFi.begin(ssid, password);
  connectWiFi();

  Serial.print("Connecting to Adafruit IO");
  io.connect();

 // wait for Adafruit IO connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  //conncetion status
  Serial.println(io.statusText());

  //toggleSwitch->onMessage(handleFeedData);
  //toggleSwitch->get();
}

void loop() {

  if (!WiFi.isConnected()){
    connectWiFi();
  }

  io.run();

  now = millis();
  if (now - last_time > 5000)
  {
      sensorTemperature = 1.353456836 * random(30.0); //max. Wert random Funktion 30 factor 1.353456836 for float generation, random only creates integer
      sensorHumidity = 1.353456836 * random(70.0); //max. Wert random Funktion 70
      sensorVOC = 1.68364967 * random(2000.0);
      sensorFillingLevel = random(100.0); 
     

      dataFillingLevel->save(sensorFillingLevel);
      dataHumidity->save(sensorHumidity);
      dataTemperature->save(sensorTemperature);
      dataVOC->save(sensorVOC);

      last_time = now;
  }
  
}