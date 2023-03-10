#include <Arduino.h>
#include "Adafruit_SHT4x.h"
#include "Adafruit_SGP40.h"
#include <SPI.h>
#include <Wire.h>

#define echo  19 // Echo Pin
#define trigger  18 //Trigger-Pin
float bucket_high = 50.00; // Bucket High

Adafruit_SHT4x sht4 = Adafruit_SHT4x();
Adafruit_SGP40 sgp;

void setup()
{
Serial.begin (9600);
while (!Serial)
    delay(10);     

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
}

  
int VOC (float t, float rh)
{
return (sgp.measureVocIndex(t, rh));
}

void temp_hum(float *t, float *rh)
{
sensors_event_t humidity, temp; 
sht4.getEvent(&humidity, &temp);// update data 
*t = temp.temperature;
*rh = humidity.relative_humidity;
}

int ultrasonic ()
{
long duration = 0;
long distance = 0;
digitalWrite(trigger, LOW); 
delay(5);
digitalWrite(trigger, HIGH); //emit an ultrasonic wave
delay(10); 
digitalWrite(trigger, LOW);
duration = pulseIn(echo, HIGH); // record pulses
distance = ((duration/2) * 0.03432)+1; // calculate distance with offset 1 cm
if (distance > bucket_high)
    {
      return (-1); // distance value invalid
    }
    else 
    {
     float f_pusage= 100-((distance/bucket_high)*100); // calculate occupied space
     f_pusage = round(f_pusage); // round value for return int
     return (f_pusage);
    }
}
void loop()
{
int32_t sensorFillingLevel = 0;
sensorFillingLevel = ultrasonic();
Serial.print(sensorFillingLevel); 
Serial.println("%");  

float sensorTemperature = 0;
float sensorHumidity = 0;
temp_hum(&sensorTemperature, &sensorHumidity);
Serial.print("Temperature: "); Serial.print(sensorTemperature); Serial.println(" degrees C");
Serial.print("Humidity: "); Serial.print(sensorHumidity); Serial.println("% rH");

int32_t sensorVOC = 0;
sensorVOC = VOC (sensorTemperature,sensorHumidity);
Serial.print("Voc Index: ");
Serial.println(sensorVOC);

delay(1000);

}