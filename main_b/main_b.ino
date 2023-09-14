///////////////////
//SAROS_I_B
//Version: 2.0
//Date: 09/14/2023
//Author: Tristan McGinnis & Sam Quartuccio
//Use: Main source code for SAROS I flight, B-Class boards.
///////////////////

// Imports:
#include "SAROS_Util.h"

//Board Details
String ID = "STX";

//Packet Values
String packet;
int utc_hr, utc_min, utc_sec;
int mis_hr, mis_min;
double mis_time;
long int packetCt = 0;
int gp_sats;
double rel_altitude, t_temp, b_pres, b_temp, humidity;
double pd1, pd2;
long gp_lat, gp_lon, gp_alt;


//BNO Setup
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // passed Wire into BNO055
double sea_level = 1013.25;
//


Adafruit_BMP3XX bmp; //BMP Setup
Adafruit_SHT4x sht4 = Adafruit_SHT4x(); //Humidity Sensor Setup


//GPS Setup
SFE_UBLOX_GNSS gps; 
//static const uint32_t GPSBaud = 38400;


//



//General Variables
unsigned int lastBlink = 0; //Last time LED Blinked
unsigned int lastPoll = 0; //Last time polling major sensors
unsigned int lastShort = 0; //last time polling PDs only
unsigned int lastGPS = 0; //last GPS poll

//Used for SPI SD Logger
const int _MISO = 8;
const int _MOSI = 11;
const int _CS = 9;
const int _SCK = 10;
//File dataFile = SD.open("example.txt", FILE_WRITE);
int fileCt = 0;



void setup() {
  //dataFile.close();
  boolean setDynamicModel(dynModel newDynamicModel = DYN_MODEL_AIRBORNE4g, uint16_t maxWait = 1100);
  //uint8_t dynamicModelTest = getDynamicModel(uint16_t maxWait = 1100); // Get the dynamic model - returns 255 if the sendCommand fails
  
  Serial.begin(115200);//USB Interface
  pinMode(25, OUTPUT);//onboard pico LED
  
  setWire0(1, 0); //set I2C0 for Wire -- SCL 1 SDA 0
  Wire.begin();
  setWire1(3, 2);//set I2C1 for Wire1 -- SCL 3 SDA 2
  Wire1.begin();
  
  //Setup for SPI SD Logger
  SPI1.setRX(_MISO);
  SPI1.setTX(_MOSI);
  SPI1.setSCK(_SCK);
  SPI1.setCS(_CS);

  if (!SD.begin(_CS, SPI1)) {
    Serial.println("initialization failed!");
    //return;
  }else{
    Serial.println("initialization done.");
  }

  //Create up to 20 unique output files
  String stringPrint = "";
  for(int i = 0; i < 20; i++)
  {
    if (SD.exists(String("data_out_"+i))) {
      stringPrint = "data_out_"+ String(i) + "already exists";
      Serial.println(stringPrint);
    } else {
      stringPrint = "data_out_" + String(i) + " does not exist. Creating: data_out_" + String(i);
      Serial.println(stringPrint);
      fileCt = i;
      break;
    }
    if(i == 20)
    {
      Serial.println("File limit reached!");
    }
  }
  

  
  File dataFile = SD.open("example.txt", FILE_WRITE);
  dataFile.println("TEST PRINT");
  dataFile.close();
  

  //Serial1.setRX(13);
  //Serial1.setTX(12);  Serial1.begin(250000);//Adafruit OpenLog Interface (TEMP)

  
  Serial2.setRX(5);
  Serial2.setTX(4);
  //set Pins for YIC, UART line begins during test
  //Serial2.begin(GPSBaud);//YIC interface

  pinMode(28, INPUT);//set INPUT pin mode for thermistor
  analogReadResolution(12);//up analog read resolution to 12 bit



  
  // Start Up Sequence
  digitalWrite(25, HIGH);
  delay(2000);
  digitalWrite(25, LOW);
  delay(500);

  Serial.println("-----------------------"); 
  Serial.println("Start Up Sequence"); 
  Serial.println("-----------------------"); 
  
  dataFile.println("-----------------------"); 
  dataFile.println("Start Up Sequence"); 
  dataFile.println("-----------------------"); 
  


  //  BNO055 Check
  if (!bno.begin()){
    Serial.println("BNO055\t[ ]");
  }else
    Serial.println("BNO055\t[X]");

  //  BMP388 Check
  if (!bmp.begin_I2C(0x77, &Wire1)){
    Serial.println("BMP388\t[ ]");
  }else
  {
    Serial.println("BMP388\t[X]");
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }
  

  //  SHT4 Check 
  if (!sht4.begin(&Wire1)){
    Serial.println("SHT4X\t[ ]");
  }else
    Serial.println("SHT4X\t[X]");
    sht4.setPrecision(SHT4X_HIGH_PRECISION);

  //  YIC Check
  Serial2.begin(9600);
  if (gps.begin(Serial2))
  {
    Serial.println("YIC on 9600, switching to 38400");//38400 example suggestion
    gps.setSerialRate(38400);
    gps.setNavigationFrequency(5);
  }
  delay(2000);
  Serial2.begin(38400);
  if (gps.begin(Serial2))
  {
    Serial.println("YIC on 38400");
    gps.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
    gps.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    gps.saveConfiguration(); //Save the current settings to flash and BBR
  }

  //Cycle BMP
  for(int i = 0; i < 10; i++)
  {
    bmp.performReading();
  }

  //Get Starting Pressure
  bmp.performReading();
  sea_level = bmp.readPressure() / 100.0;
  double temp_alt = bmp.readAltitude(sea_level);
  Serial.println("Start Pressure/Alt: "+ String(sea_level) +","+String(bmp.readAltitude(sea_level)));
  dataFile.println("Start Pressure/Alt: "+ String(sea_level)+","+String(bmp.readAltitude(sea_level)));
  
  Serial.println("-----------------------"); 
  Serial.println("Start Up Sequence Complete"); 
  Serial.println("-----------------------"); 
  
  dataFile.println("-----------------------"); 
  dataFile.println("Start Up Sequence Complete"); 
  dataFile.println("-----------------------"); 
  

  
}

//henry.sun@yic.com

void loop() {
  //Serial.println(millis());
  //File dataFile = SD.open("example.txt", FILE_WRITE);
  String fName = "data_out_" + String(fileCt)+".txt";
  File dataFile = SD.open(fName, FILE_WRITE);

  while(mis_time <= 21600)//run for 6 hours
  {
    mis_time = millis()/1000.0;

    pd1 = analogRead(26);
    pd2 = analogRead(27);
    t_temp = (1.0/(log((200000.0/((4095.0/analogRead(28)) - 1))/200000)/3892.0 + 1.0/(25 + 273.15))) - 273.15; //Calculation for Thermistor

    if(threadFunc(240, millis() , &lastPoll))
    {
      packetCt++;
      bmp.performReading();

      sensors_event_t humidity, temp;
      sht4.getEvent(&humidity, &temp);

      //This check function freezes the process.
      /*
      if(gps.checkUblox()) //get new GPS data if available
      {
        //Serial.println("GPS Avail.");
        gp_lat = gps.getLatitude();
        gp_lon = gps.getLongitude();
        gp_sats = gps.getSIV();
        utc_hr = gps.getHour();
        utc_min = gps.getMinute();
        utc_sec = gps.getSecond();
      }
      */

      
      //mis_time = millis()/1000.00;
      //mis_time = millis()/1000.0;


      //Large-Format 4hz packet
      packet = ID + "," + String(packetCt) + "," + String(mis_time) + "," + String(pd1)+","+String(pd2) + ",";
      packet += String(t_temp) + "," + String(utc_hr) + ":" + String(utc_min) + ":" + String(utc_sec) + "," + String(bmp.readAltitude(sea_level))+",";
      packet += String(gp_lat)+","+String(gp_lon)+","+String(gp_sats)+","+String(gp_alt)+","+String(bmp.pressure/100.0)+",";
      packet += String(bmp.temperature)+","+String(humidity.relative_humidity)+",";
      packet += readBno(bno, 1) +","+readBno(bno, 2)+","+readBno(bno, 3)+","+readBno(bno, 4);
      Serial.println(packet);
      dataFile.println(packet);
      dataFile.flush();
    }else
    {
      //Small-Format no limit packet
      if(mis_time <= 14400.00)//Only run for 4 hours
      {
        packetCt++;
        packet = String(ID)+","+String(packetCt)+","+ String(mis_time) +","+String(pd1)+","+String(pd2) + "," + String(t_temp) + ",,,,,,,,,,,,,,,,";
        //Serial.println(packet);
        dataFile.println(packet);
        //dataFile.flush();
      }
    }
  }









}
