//////////////////
//SAROS_Util
//Version: 1.1
//Date: 7/7/2023
//Supported Main: 1.X
//Author: Tristan McGinnis
//Use: Contains supplementary functions and values used for SAROS payloads
///////////////////
#include <Wire.h> 

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_BNO055.h> 
#include <utility/imumaths.h> 
#include "Adafruit_BMP3XX.h" 
#include "Adafruit_SHT4x.h" 
#include <SD.h>
#include <SPI.h>

sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;

//Functions in this library
String printEvent(sensors_event_t* event);
String readBno(Adafruit_BNO055 bno, int type);
void ledPulse(int led_pin,int blinkSpeed, unsigned int curTime , unsigned int *lastToggle);
void setWire0(int pin_SCL, int pin_SDA);
void setWire1(int pin_SCL, int pin_SDA);
boolean threadFunc(int freq, unsigned int curTime , unsigned int *lastRun);




void setWire0(int pin_SCL, int pin_SDA)
{
  Wire.setSDA(pin_SDA);
  Wire.setSCL(pin_SCL);
}

void setWire1(int pin_SCL, int pin_SDA)
{
  Wire1.setSDA(pin_SDA);
  Wire1.setSCL(pin_SCL); 
}

void ledPulse(int led_pin,int blinkSpeed, unsigned int curTime , unsigned int *lastToggle)
{
  if(int(curTime - *lastToggle) >= blinkSpeed) //If it's time or past time to toggle
  {
    digitalWrite(led_pin, !digitalRead(led_pin));
    *lastToggle = curTime;
  }
}

boolean threadFunc(int freq, unsigned int curTime , unsigned int *lastRun)
{
  if(int(curTime - *lastRun) >= freq) //If it's time or past time to run
  {
    *lastRun = curTime;
    return true;    
  }
  return false;
}

String readBno(Adafruit_BNO055 bno, int type)
{
  String returnValue = "bno_CannotRead";
  if(type == 1)//orientation data
  {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    return printEvent(&orientationData);
  }else if(type == 2)//gyro data
  {
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    return printEvent(&angVelocityData);
  }else if(type == 3)//mag data
  {
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    return printEvent(&magnetometerData);
  }else if(type == 4)//accel data
  {
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    return printEvent(&accelerometerData);
  }else
  {
    return "bno_UnknownType";
  }
}

String printEvent(sensors_event_t* event) {
  String returnValues = "bno_NoValues";
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    //Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    //Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    //Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    //Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    //Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    //Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    //Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    //Serial.print("Unk:");
    return "bno_UnknownEvent";
  }
  returnValues = String(x) + "," + String(y) + "," + String(z);
  return returnValues;
}



//NOTES




/*BNO READ AND PRINT EXAMPLES
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);
 */
