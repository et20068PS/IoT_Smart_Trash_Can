#include <Arduino.h>

#include <WiFi.h>
#include <AdafruitIO_WiFi.h>
#include <AdafruitIO_Feed.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_SGP40.h>
#include <SPI.h>
#include <Wire.h>

#define echo  19 // Echo Pin
#define trigger  18 //Trigger-Pin

Adafruit_SHT4x sht4 = Adafruit_SHT4x();
Adafruit_SGP40 sgp;

#define AIO_USERNAME  "SmartTrashCan"
#define AIO_KEY       "Replace with your specific Adafruit IO Key"
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883

//WiFi connection variables
const char* ssid = "WLAN Lutz";
const char* password =  "GottIstSoGut!";
// Haaaaaloo
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
AdafruitIO_Feed *dataEmptyTrash = io.feed("emptytrash");


//Adafruit Topics for remote control
AdafruitIO_Feed *dataBucketHeight = io.feed("bucketheight");
AdafruitIO_Feed *dataSaveEnergy = io.feed("saveenergy");

//Sensor measurement variables
float sensorTemperature;
float sensorHumidity;
float sensorVOC;
int sensorFillingLevel;
int outputEmptyTrash;

//Limit variables
float limitHumidity = 50;
float limitVOC = 3000;
float bucketHeight = 50.00;
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


// read VOC sensor
int VOC (float t, float rh){
  return (sgp.measureVocIndex(t, rh));
}

// read temperature and humidity
void temp_hum(float *t, float *rh){
  sensors_event_t humidity, temp; 
  sht4.getEvent(&humidity, &temp);// update data 
  *t = temp.temperature;
  *rh = humidity.relative_humidity;
}

// initialize VOC sensor with real temperature and humidity data
void init (float t, float rh){
  int initVOC = sgp.measureVocIndex(t, rh);
}

// read ultarsonic sensor and calculate filling level
int ultrasonic (){
  long duration = 0;
  long distance = 0;
  digitalWrite(trigger, LOW); 
  delay(5);
  digitalWrite(trigger, HIGH); //emit an ultrasonic wave
  delay(10); 
  digitalWrite(trigger, LOW);
  duration = pulseIn(echo, HIGH); // record pulses
  distance = ((duration/2) * 0.03432)+1; // calculate distance with offset 1 cm
    if (distance > bucketHeight)
      {
        return (-1); // distance value invalid
      }
    else 
      {
      float f_pusage= 100-((distance/bucketHeight)*100); // calculate occupied space
      f_pusage = round(f_pusage); // round value for return int
      return (f_pusage);
      }
}

//Read actual sensor data
void readSensors(){
    //generating random sensor data for testing
    //sensorTemperature = 1.353456836 * random(30.0); //max. Wert random Funktion 30 factor 1.353456836 for float generation, random only creates integer
    //sensorHumidity = 1.353456836 * random(70.0); //max. Wert random Funktion 70
    //sensorVOC = 1.68364967 * random(2000.0);
    //sensorFillingLevel = random(100.0); 
  sensorFillingLevel = ultrasonic();
  Serial.print(sensorFillingLevel); 
  Serial.println("%");  

  temp_hum(&sensorTemperature, &sensorHumidity);
  Serial.print("Temperature: "); Serial.print(sensorTemperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(sensorHumidity); Serial.println("% rH");

  sensorVOC = VOC (sensorTemperature,sensorHumidity);
  Serial.print("Voc Index: ");
  Serial.println(sensorVOC);
}

//calculate whether garbage should be emptied
void calculateEmptyTrash(){
  if(sensorFillingLevel > 74){
    outputEmptyTrash = 1;
  }
  else if (sensorHumidity > 85){
    outputEmptyTrash = 1;
  }
  else if(sensorVOC > 300){
    outputEmptyTrash = 1;
  }
  else if((sensorTemperature > 27) && (sensorHumidity > 80)){
    outputEmptyTrash = 1;
  }
  else {
    outputEmptyTrash = 0;
  }
}

//Publish data to Adafruit IO
void publishData(){
    dataFillingLevel->save(sensorFillingLevel);
    dataHumidity->save(sensorHumidity);
    dataTemperature->save(sensorTemperature);
    dataVOC->save(sensorVOC);
    dataEmptyTrash->save(outputEmptyTrash);
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
    bucketHeight = data->toFloat();
  }
}

void setup()
{
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
  Serial.println();
  Serial.println(io.statusText());
  io.run();

  //Receive data from Adafruit IO Dashboard
  dataBucketHeight->onMessage(handleFeedData);
  dataBucketHeight->get();
  dataSaveEnergy->onMessage(handleFeedData);
  dataSaveEnergy->get();
  
if (! sht4.begin()) {
      Serial.println("Couldn't find SHT40 - Program stops working");
      while (1) delay(1);
}
Serial.println("Found SHT40 sensor");

if (! sgp.begin()){
  Serial.println("Couldn't find SGP40 - Program stops working");
  while (1);
}
Serial.println("Found SGP40 sensor");

pinMode(echo, INPUT); // Echo-Pin is Input
pinMode(trigger, OUTPUT); // Trigger_Pin is Output
sht4.setPrecision(SHT4X_HIGH_PRECISION); // get best precision
  switch (sht4.getPrecision()) {
     case SHT4X_HIGH_PRECISION: 
       Serial.println("High precision");
       break;
     case SHT4X_MED_PRECISION: 
       Serial.println("Med precision");
       break;
     case SHT4X_LOW_PRECISION: 
       Serial.println("Low precision");
       break;
  }

  sht4.setHeater(SHT4X_NO_HEATER); // set no heater for temp and hum sensor
  switch (sht4.getHeater()) {
     case SHT4X_NO_HEATER: 
       Serial.println("No heater");
       break;
     case SHT4X_HIGH_HEATER_1S: 
       Serial.println("High heat for 1 second");
       break;
     case SHT4X_HIGH_HEATER_100MS: 
       Serial.println("High heat for 0.1 second");
       break;
     case SHT4X_MED_HEATER_1S: 
       Serial.println("Medium heat for 1 second");
       break;
     case SHT4X_MED_HEATER_100MS: 
       Serial.println("Medium heat for 0.1 second");
       break;
     case SHT4X_LOW_HEATER_1S: 
       Serial.println("Low heat for 1 second");
       break;
     case SHT4X_LOW_HEATER_100MS: 
       Serial.println("Low heat for 0.1 second");
       break;
  }
  
  Serial.println("Starte VOC Sensor Initialisierungszeit");
  for (int i = 0; i<= 200 ;i++)
  {
    temp_hum(&sensorTemperature, &sensorHumidity);
    init (sensorTemperature,sensorHumidity);
    delay(1000);
  }
  Serial.println("VOC vollständig initialisiert - beginne mit Programmablauf");
}

void loop()
{
 //Must be called frequently to keep Adafruit IO connection alive
  io.run();

  now = millis();
  if (!saveEnergy && now - last_time > 10000)
  {
      if (!WiFi.isConnected()){
        connectWiFi();
      }

      readSensors();
      calculateEmptyTrash();
      //publishing sensor data to Adafruit IO
      publishData();

      last_time = now;
  }
  else if(saveEnergy){

    readSensors();
    calculateEmptyTrash();
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
