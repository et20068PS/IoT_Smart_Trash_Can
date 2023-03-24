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

//Timer variables
long last_time;
long now;

//Adafruit IO connection variables
AdafruitIO_WiFi io(AIO_USERNAME, AIO_KEY, ssid, password);

//Adafruit Topics for sensors
AdafruitIO_Feed *dataFillingLevel = io.feed("fill");
AdafruitIO_Feed *dataHumidity = io.feed("humidity");
AdafruitIO_Feed *dataTemperature = io.feed("temperature");
AdafruitIO_Feed *dataVOC = io.feed("air");

//Adafruit Topics for remote control
AdafruitIO_Feed *dataBucketHeight = io.feed("bucketheight");
AdafruitIO_Feed *dataSaveEnergy = io.feed("saveenergy");

//Sensor measurement variables
float sensorTemperature;
float sensorHumidity;
float sensorVOC;
int sensorFillingLevel;

//Limit variables
float limitHumidity = 50;
float limitVOC = 3000;
int bucketHeight = 50;
boolean criticalWarning = false;

boolean saveEnergy;

//WiFi connect function
void connectWiFi(){
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to the WiFi network");
}

//WiFi disconnect function
void disconnectWiFi(){
  Serial.print("Disonnecting WiFi");
  while (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Disconnected from WiFi network");
}

//Read actual sensor data
void readSensors(){
    //generating random sensor data for testing
    sensorTemperature = 1.353456836 * random(30.0); //max. Wert random Funktion 30 factor 1.353456836 for float generation, random only creates integer
    sensorHumidity = 1.353456836 * random(70.0); //max. Wert random Funktion 70
    sensorVOC = 1.68364967 * random(2000.0);
    sensorFillingLevel = random(100.0); 
}

//Publish data to Adafruit IO
void publishData(){
    dataFillingLevel->save(sensorFillingLevel);
    dataHumidity->save(sensorHumidity);
    dataTemperature->save(sensorTemperature);
    dataVOC->save(sensorVOC);
}

//Function to process received data from Adafruit IO Dashboard
void handleFeedData(AdafruitIO_Data *data) {
  Serial.print("Received data from feed: ");
  Serial.println(data->feedName());
  Serial.println(data->value());

  if(String(data->feedName()) == "saveenergy"){
    saveEnergy = data->toBool();
  }
  else if(String(data->feedName()) == "bucketheight"){
    bucketHeight = data->toInt();
  }
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
  //Adafruit IO conncetion status
  Serial.println();
  Serial.println(io.statusText());

  io.run();

  //Receive data from Adafruit IO Dashboard
  dataBucketHeight->onMessage(handleFeedData);
  dataBucketHeight->get();
  dataSaveEnergy->onMessage(handleFeedData);
  dataSaveEnergy->get();
}

void loop() {

  //Must be called frequently to keep Adafruit IO connection alive
  io.run();

  now = millis();
  if (!saveEnergy && now - last_time > 10000)
  {
      if (!WiFi.isConnected()){
        connectWiFi();
      }

      readSensors();
      //publishing sensor data to Adafruit IO
      publishData();

      last_time = now;
  }
  else if(saveEnergy){

    readSensors();
    //publishing sensor data to Adafruit IO
    publishData();

    //Sleep mode implementation
    //Stays in sleep mode until timer wakes up the ESP32
    //When woke up, ESP32 starts from the beginning of the code and reinitializes
    Serial.println("Ich geh schlafen");
    disconnectWiFi();
    esp_sleep_enable_timer_wakeup(1 * 60 * 1000000); //1 * 60 * 1000000 = 1 minute
    esp_deep_sleep_start();

  }
  
}