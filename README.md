
# SAROS-I

Software for SAROS-I launch(es). Class B payloads are the only class of payloads in use for SAROS-I. 

## Usage
The following are pre-requisites for making this source code functional on the flight boards. 

 1. Use Arduino IDE 2.X. Issues have been found when uploading to boards with IDE 1.X versions.
 2. Modify your libraries. This code uses modified libraries for several sensors. Copy the folders inside the `/main_b/libraries/` into your main `/Arduino/libraries/` directory to add/overwrite the existing libraries. Backing up your original libraries is recommended but you can always reinstall the libraries through the IDE at any time. 

  

## LED Code Guide:
The following is a guide for reading the setup status indicator LEDs. During setup (immediately after power on) there will be a 3-pulse blue flash after each sensor's startup is successful. Following the setup of all sensors, the board will flash red every ~3 seconds to indicate waiting for a GPS lock. Upon GPS lock or ~4.5 minutes time out, whichever occurs first, the program will continue into data logging mode. In logging mode, the on-board LED will pulse at 1hz to indicate logging and proper operation. <br>

### Key:
O-off <br>

S-static <br>

F-flashing <br>
###### -------------------
LED Order: BLUE/GREEN/RED <br>
Some "ground modules" may read BLUE/RED/RED,  but indicators should always be read from left to right with BLUE as the leftmost LED and RED as the rightmost.
##### ----------------------

-1: F/F/F -GPS Locked <br>

0:  F/O/O -Sensor setup success<br>

1:  O/F/O <br>

2:  O/O/F -SD Init Failure (SD Card not found) <br>

3:  S/F/O -SD File Limit Reached (25 data_out files) <br>

4:  S/O/F -BME Startup Failure <br>

5:  F/S/O <br>

6:  O/S/F -GPS Startup Failure <br>

7:  F/O/S -ADS Startup Failure <br>

8:  O/F/S -SHTX Startup Failure <br>

9:  F/F/O <br>

10:O/F/F -BNO Startup Failure <br>

11:F/O/F  -SD No ID found <br>

20: Rolling colors across all LEDs - Setup Process Complete <br>

Constant Blinking Red- Will occur after all sensors are started, indicating attempts to lock GPS <br>
