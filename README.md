# SAROS-Demo-Flight
Software for payload demonstration flights.

## LED Code Guide:
O-off <br>
S-static <br>
F-flashing <br> 
 <br>
Order: BLUE/GREEN/RED <br> 
 <br>
-1: F/F/F -GPS Locked <br> 
0:  F/O/O <br> 
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
20: Rolling colors across all LEDs - Setup Complete (After attempting GPS Lock for ~4.5 minutes) <br> 
 <br>
Constant Blinking Red- Trying to lock GPS (This will go for ~4.5 minutes) <br> 

