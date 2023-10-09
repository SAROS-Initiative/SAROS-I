//////////////////
//SAROS_Util
//Version: 1.5
//Date: 09/15/2023
//Supported Main: 2.X
//Author: Tristan McGinnis
//Use: Contains supplementary functions and values used for SAROS payloads
///////////////////
#include <Wire.h> 


#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
//#include <SparkFun_u-blox_GNSS_v3.h>

#include <Adafruit_Sensor.h> 
#include <Adafruit_BNO055.h> 
#include <utility/imumaths.h>
#include "Zanshin_BME680.h"
//#include "libraries/BME680/src/Zanshin_BME680.h"
//#include "Adafruit_BME680.h"
//#include "libraries/cus_bme680/BME680.h"
#include "ADS1X15.h"
//#include <Adafruit_ADS1X15.h>
#include "Adafruit_SHT4x.h" 
#include <SD.h>
#include <SPI.h>

sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;

//Functions in this library
String printEvent(sensors_event_t* event);
String readBno(Adafruit_BNO055 bno, int type);
void ledToggle(int led_pin);
void ledPulse(int led_pin,int blinkSpeed, unsigned long curTime , unsigned long *lastToggle);
void setWire0(int pin_SCL, int pin_SDA);
void setWire1(int pin_SCL, int pin_SDA);
void ledCode(int led1, int led2, int led3, int code);
boolean threadFunc(int freq, unsigned long curTime , unsigned long *lastRun);

String getBoardID();


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



void ledCode(int led1, int led2, int led3, int code) //Flash a specific LED code
{
  //o-off s-static f-flashing
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);

  int blink_count = 10;
  int delay_len = 150;

  switch(code){
    case -1: //Blue-f Green-f Red-f
      for(int i = 0; i < blink_count; i++)
      {
        ledToggle(led1);
        ledToggle(led2);
        ledToggle(led3);
        delay(delay_len);
        ledToggle(led1);
        ledToggle(led2);
        ledToggle(led3);
        delay(delay_len);
      }
      break;
    case 0: //Blue-f Green-o Red-o
      for(int i = 0; i < blink_count; i++)
      {
        ledToggle(led1);
        delay(delay_len);
        ledToggle(led1);
        delay(delay_len);
      }
      break;
    case 1: //Blue-o Green-f Red-o
      for(int i = 0; i < blink_count; i++)
      {
        ledToggle(led2);
        delay(delay_len);
        ledToggle(led2);
        delay(delay_len);
      }
      break;
    case 2: //Blue-o Green-o Red-f
      for(int i = 0; i < blink_count; i++)
      {
        ledToggle(led3);
        delay(delay_len);
        ledToggle(led3);
        delay(delay_len);
      }
      break;
    case 3: //Blue-s Green-f Red-o
      ledToggle(led1);
      for(int i = 0; i < blink_count; i++)
      {
        ledToggle(led2);
        delay(delay_len);
        ledToggle(led2);
        delay(delay_len);
      }
      break;
    case 4: //Blue-s Green-o Red-f
      ledToggle(led1);
      for(int i = 0; i < blink_count; i++)
      {
        ledToggle(led3);
        delay(delay_len);
        ledToggle(led3);
        delay(delay_len);
      }
      break;
    case 5: //Blue-f Green-s Red-o
      ledToggle(led2);
      for(int i = 0; i < blink_count; i++)
      {
        ledToggle(led1);
        delay(delay_len);
        ledToggle(led1);
        delay(delay_len);
      }
      break;
    case 6: //Blue-o Green-s Red-f
      ledToggle(led2);
      for(int i = 0; i < blink_count; i++)
      {
        ledToggle(led3);
        delay(delay_len);
        ledToggle(led3);
        delay(delay_len);
      }
      break;
    case 7: //Blue-f Green-o Red-s
      ledToggle(led3);
      for(int i = 0; i < blink_count; i++)
      {
        ledToggle(led1);
        delay(delay_len);
        ledToggle(led1);
        delay(delay_len);
      }
      break;
    case 8: //Blue-o Green-f Red-s
      ledToggle(led3);
      for(int i = 0; i < blink_count; i++)
      {
        ledToggle(led2);
        delay(delay_len);
        ledToggle(led2);
        delay(delay_len);
      }
      break;
    case 9: //Blue-f Green-f Red-o
      for(int i = 0; i < blink_count; i++)
      {
        ledToggle(led1);
        ledToggle(led2);
        delay(delay_len);
        ledToggle(led1);
        ledToggle(led2);
        delay(delay_len);
      }
      break;
    case 10: //Blue-o Green-f Red-f
      for(int i = 0; i < blink_count; i++)
      {
        ledToggle(led2);
        ledToggle(led3);
        delay(delay_len);
        ledToggle(led2);
        ledToggle(led3);
        delay(delay_len);
      }
      break;
    case 11: //Blue-f Green-o Red-f
      for(int i = 0; i < blink_count; i++)
      {
        ledToggle(led1);
        ledToggle(led3);
        delay(delay_len);
        ledToggle(led1);
        ledToggle(led3);
        delay(delay_len);
      }
      break;
    case 20: //Rolling LEDS
      for(int i = 0; i < blink_count; i++)
      {
        ledToggle(led1);
        delay(50);
        ledToggle(led1);
        ledToggle(led2);
        delay(50);
        ledToggle(led2);
        ledToggle(led3);
        delay(50);
        ledToggle(led3);
        delay(50);
        ledToggle(led3);
        delay(delay_len);
      }
    default:
      break;
  }

  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
}
    

void ledToggle(int led_pin) //Toggles LED from it's current status
{
  digitalWrite(led_pin, !digitalRead(led_pin));
}


void ledPulse(int led_pin,int blinkSpeed, unsigned long curTime , unsigned long *lastToggle)//Pulse LED at desired rate w/o hold
{
  if(int(curTime - *lastToggle) >= blinkSpeed) //If it's time or past time to toggle
  {
    digitalWrite(led_pin, !digitalRead(led_pin));
    *lastToggle = curTime;
  }
}

void ledBlink(int led_pin, int speed, int count) //Blink led at desired speed, desired amount of times, w/ system hold (delays)
{
  digitalWrite(led_pin, LOW);
  for(int i = 0; i < count; i++)
  {
    ledToggle(led_pin);
    delay(speed);
    ledToggle(led_pin);
    delay(speed);
  }
}


boolean threadFunc(int freq, unsigned long curTime , unsigned long *lastRun)
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


String getBoardID()
{
  //Pull the board ID# from the id.txt file on the SD card
  if (SD.exists("id.txt"))
  {
    String inputID = "";
    File idFile = SD.open("id.txt", FILE_READ);
    while (idFile.available())
    {
      char buf = char(idFile.read());
      
      inputID += buf;
    }
    String String_ID = String(inputID);
    idFile.close();
    return String_ID;
  }else
  {
    return "ERR";
  }

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
