///////////////////
//SAROS_I_Class_B
//Version: 2.6
//Date: 10/08/2023
//Author: Tristan McGinnis
//Use: Main source code for SAROS test board
///////////////////

// Imports:
#include "SAROS_Util.h"

// Debug settings
#define debug 0 //Running in DEBUG mode? Main LEDS will indicate during loop
                //GPS will periodically test for a lock every 30 seconds during main loop
#define skipGPSLock 0 //Skip waiting for GPS lock?

//Constants
#define LED1 15
#define LED2 14
#define LED3 13
#define LED0 25

//Board Details
String ID = "SX";

//Packet Values
String packet;
int utc_hr, utc_min, utc_sec;
int mis_hr, mis_min;
double mis_time;
long int packetCt = 0;
int gp_sats;
double humidity;
int16_t pd1, pd2, pd3, pd4;
long gp_lat, gp_lon, gp_alt;
//static int32_t b_temp, b_humidity, b_press, b_gas;
int32_t b_temp = 99;
int32_t b_humidity = 99;
int32_t b_press = 99;
int32_t b_gas = 99;



//BNO Setup
//uint16_t BNO055_SAMPLERATE_DELAY_MS = 5;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1); // passed Wire into BNO055
double sea_level = 1013.25;
//

//BME680 Setup
//Adafruit_BME680 bme = Adafruit_BME680(&Wire1);
BME680_Class BME680;
//Bme68x bme;//bme object for BOSH Bme6 library

//Humidity Sensor
Adafruit_SHT4x sht4 = Adafruit_SHT4x(); //Humidity Sensor Setup

//GPS Setup
SFE_UBLOX_GNSS gps; 
int gpsLock = 0;
int gpsLockAttempts = 0;
int gpsFound = 0;
//static const uint32_t GPSBaud = 38400;

//ADC to I2C ADS1015
ADS1015 ADS(0x49, &Wire1);
//Adafruit_ADS1015 ADS;

//Thermistor Things
double R_T; //Calculation for Thermistor temperature value
double t_temp; 


//Timing Variables
unsigned long lastPoll = 0; //Last time polling major sensors
unsigned long lastShort = 0; //last time polling PDs only
unsigned long lastGPSLockCheck = 0;


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
  

  //LEDs
  pinMode(LED1, OUTPUT); // Blue
  pinMode(LED2, OUTPUT); // Green
  pinMode(LED3, OUTPUT); // Red
  pinMode(LED0, OUTPUT);//onboard pico LED


  //setWire1(3,2);//(scl, sda)
  Wire1.setSCL(3);
  Wire1.setSDA(2);
  Wire1.begin();
  Wire1.setClock(500000);
  
  //Setup for SPI SD Logger
  SPI1.setRX(_MISO);
  SPI1.setTX(_MOSI);
  SPI1.setSCK(_SCK);
  SPI1.setCS(_CS);

  digitalWrite(25, HIGH);
  delay(2000);
  digitalWrite(25, LOW);

  pinMode(26, INPUT);//set INPUT pin mode for thermistor
  //analogReadResolution(10);//up analog read resolution to 12 bit- TO BE REPLACED BY I2C ADC Converter
  

  if (!SD.begin(_CS, SPI1)) {
    Serial.println("SD initialization failed!");
    ledCode(LED1, LED2, LED3, 2);//Code BGR Off/Off/Flashing
  }else{
    Serial.println("SD initialized.");
    ledBlink(LED1, 100, 3);//Blink Blue for Success
  }

  ID = "S" + String(getBoardID()); //Get assigned board ID from id.txt on SD card
  if(ID == "SERR")
  {
    ledCode(LED1, LED2, LED3, 11);//Code BGR Flashing/Off/Flashing
  }

  //Increment file name id to generate different output files upon reset
  for(int i = 0; i < 25; i++)
  {
    String stringPrint = "";
    String fileName = String(ID)+"_data_out_"+String(i)+".txt";

    if(i == 24)
    {
      Serial.println("File limit reached!"); 

      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      ledCode(LED1, LED2, LED3, 3);//Code BGR Stable/Flashing/Off

      fileCt = 99;
    }

    if (SD.exists(fileName)) {
      stringPrint = fileName + " already exists";
      Serial.println(stringPrint);

    } else {
      stringPrint = fileName + " does not exist. Creating: data_out_" + String(i);
      Serial.println(stringPrint);
      fileCt = i;

      ledBlink(LED1, 100, 3);//Blink Blue for Success
      break;
    }

  }
  

  //GPS Check
  //Serial2.begin(9600);
  for(int i = 0; i < 11; i++)//Try to start GPS for 10 seconds
  {
    if(gps.begin(Wire1, 0x42) == true)
    {
      gps.setI2COutput(COM_TYPE_UBX);
      gps.setNavigationFrequency(5);
      gps.setI2CpollingWait(250);

      Serial.println("NEO-M9N\t[X]");

      gpsFound = 1;
      ledBlink(LED1, 100, 3);//Blink Blue for Success
      break;
    }
    Serial.println("NEO-M9N\t[]");
    if(i == 10)//If not started after 10 seconds
    {
      ledCode(LED1, LED2, LED3, 6);//Code BGR Off/Stable/Flashing
      break;
    }
    delay(1000);
  }


  //  BNO055 Check
  if (!bno.begin()){
    Serial.println("BNO055\t[ ]");
    ledCode(LED1, LED2, LED3, 10);//Code BGR Off/Flashing/Flashing
  }else
  {
    Serial.println("BNO055\t[X]");
    ledBlink(LED1, 100, 3);//Blink Blue for Success
  }
  

  //BME680 Check
  if(!BME680.begin(500000, 0))
  {
    Serial.println("BME680\t[ ]");
    ledCode(LED1, LED2, LED3, 4);//Code BGR Stable/Off/Flashing
  }else
  {
    Serial.println("BME680\t[X]");
    BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
    BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
    BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
    BME680.setIIRFilter(IIR4);
    BME680.setGas(0,0);
    
    //Cycle BME to remove initial garbage data
    for(int i = 0; i < 5; i++)
    {
      BME680.getSensorData(b_temp, b_humidity, b_press, b_gas);
      Serial.println("CLEARING...");
      delay(100);
    }
    ledBlink(LED1, 100, 3);//Blink Blue for Success
  }
  
  //Delay to seperate setup flashes
  delay(500);


  //  ADS1015 Check
  if (!ADS.begin()){//for adafruit, set ADS.begin(0x49, &Wire1)
    Serial.println("ADS1015\t[ ]");
    ledCode(LED1, LED2, LED3, 7);//Code BGR Flashing/Off/Stable
  }else
  {
    Serial.println("ADS1015\t[X]");
    
    //Serial.println("ADS_DR: "+String(ADS.getDataRate()));
    ADS.setMode(0);
    ADS.setWireClock(500000);
    ADS.requestADC(0);
    ADS.setDataRate(7);//4000 for adafruit library, 7 for other
    Serial.println("DATA RATE:");
    Serial.println(String(ADS.getDataRate()));
    Serial.println("MODE TYPE:");
    Serial.println(String(ADS.getMode()));
    
    ledBlink(LED1, 100, 3);//Blink Blue for Success
  }
  

  //Delay to seperate setup flashes
  delay(500);

  //  SHT4 Check 
  if (!sht4.begin(&Wire1)){
    Serial.println("SHT4X\t[ ]");
    ledCode(LED1, LED2, LED3, 8);//Code BGR Off/Flashing/Stable
  }else
  {
    Serial.println("SHT4X\t[X]");
    sht4.setPrecision(SHT4X_HIGH_PRECISION);

    ledBlink(LED1, 100, 3);//Blink Blue for Success
  }


  if(gpsFound && !skipGPSLock)
  {
    for(int j = 0; j < 80; j++)//Try for ~4 minutes (total of 3 second delay per attempt)
    {
      if(gps.getSIV() >= 3)
      {
        Serial.println("GPS SIV:");
        Serial.println(String(gps.getSIV()));
        gpsLock = 1;
        ledCode(LED1, LED2, LED3, -1);//Code BGR Flashing/Flashing/Flashing
        break;
      }
      ledBlink(LED3, 100, 5);//Flash RED for 1 second
      delay(2000);
    }
    delay(1000);
  }

  Serial.println("-----------------------"); 
  Serial.println("Start Up Sequence Complete"); 
  Serial.println("-----------------------"); 
  ledCode(LED1, LED2, LED3, 20);

}


//henry.sun@yic.com

/////////////
//Main Loop Start
/////////////
void loop() {

  String fName = String(ID)+"_data_out_" + String(fileCt)+".txt"; //File name chosen based on last created file
  File dataFile = SD.open(fName, FILE_WRITE); //Open data output file

  do//runs for 6 hours
  {

    pd1 = ADS.readADC(0); //read photodiode 1
    pd2 = ADS.readADC(1); //read photodiode 2
    pd3 = ADS.readADC(2); //read photodiode 3
    pd4 = ADS.readADC(3); //read photodiode 4


    R_T = log((((10*1000) * pow(2,10)) / analogRead(26)) - (10*1000)); //Calculation for Thermistor temperature value
    t_temp = 0.0085*pow(R_T, 4) - 0.4359*pow(R_T, 3) + 9.233*pow(R_T, 2) - 108.53*R_T + 520.59;

    if(threadFunc(1000, millis() , &lastPoll))//Run large-format packet every 1000ms
    {
      ledToggle(25);

      packetCt++;
      //bme.performReading();
      BME680.getSensorData(b_temp, b_humidity, b_press, b_gas, false);
      String bme_data = String(b_temp/100.0)+","+String(b_press/100.0) + "," +String(b_humidity);

      if(gpsFound)
      {
        if(debug)
        {
          if(threadFunc(30000, millis(), &lastGPSLockCheck)) //Check if GPS is locked every 30 seconds
          {
            if(gps.getSIV() >=3)
            {
              gpsLock = 1;
              digitalWrite(LED3, HIGH); //RED LED ON
            }else
            {
              gpsLock = 0;
              digitalWrite(LED3, LOW); //RED LED OFF
            }
          }
        }

        gp_lat = gps.getLatitude();
        gp_lon = gps.getLongitude();
        //gp_sats = gps.getSIV();
        gp_alt = gps.getAltitude();
        utc_hr = gps.getHour();
        utc_min = gps.getMinute();
        utc_sec = gps.getSecond();
      }
      
      sensors_event_t humidity, temp;
      sht4.getEvent(&humidity, &temp);
      double rel_humidity = humidity.relative_humidity;


      //Large-Format 4hz packet
      packet = ID + "," + String(packetCt) + "," + String(mis_time) + "," + String(pd1)+","+String(pd2) + "," + String(pd3) +","+ String(pd4) +",";
      packet += String(t_temp) + "," + String(utc_hr) + ":" + String(utc_min) + ":" + String(utc_sec) + ",";
      packet += String(gp_lat)+","+String(gp_lon)+","+String(gp_sats)+","+String(gp_alt)+",";
      packet += String(bme_data)+","+String(rel_humidity)+",";
      packet += readBno(bno, 1) +","+readBno(bno, 2)+","+readBno(bno, 3)+","+readBno(bno, 4);
      Serial.println(packet);
      dataFile.println(packet);
      dataFile.flush();

      mis_time = millis()/1000.0; //get mission time (system clock time)
    }else if(threadFunc(1, millis(), &lastShort)) //run high-freq no more than 1khz
    {

      packetCt++;
      packet = String(ID)+","+String(packetCt)+","+ String(mis_time) +","+String(pd1)+","+String(pd2) + "," + String(pd3)+ ","+String(pd4)+"," + String(t_temp) + ",,,,,,,,,,,,,,,,,,,,,";
      //Serial.println(packet);
      dataFile.println(packet);
      //dataFile.flush();
      ///Serial.println(String(packetCt));
      lastShort = millis();

    }
    //test_fin = millis() - test_start;
    //Serial.println(test_fin);
  }while(mis_time <= 21600);//Run for 6 hours maximum

}// Main loop end

