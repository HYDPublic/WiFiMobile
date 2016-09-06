# WiFiMobile
A Mobile Loconet Command station, allowing Loconet over UDP to control a single Locomotive. 
This code cts as a mobile Loconet UDP node and recieves LOCONET commnds from Rocrail. 
It accepts locomotive speed and direction commands and controls a servo on D8 that runs the motor in the engine.
An RFID reader is mounted on the loco and also run by this code, and reads tags on the track. 
When  a tag is seen, it sends a unique (7bit) code in the rage 100-217 to Rocrail so it can act as block indicators.
The address is deliberately made 7 bit to keep address short. A consequence is that in a batch of many tags, you may find "duplicates". 

The code includes "left overs" from the switch project that will be removed later when the code is tidied up, but it is now workng..

Key features:
The code acts as a single slot "Command Station". It responds with a Slot number equating to the Locomotive address, which is set in CV[1]
CV's can be set and read using the RocRail programmer. 
CV's 5 and 2 set the upper and lower limts of the Servo for forwards speed.
CV's 9 and 6 set the upper and lower limits of the Servo for backwards speed
CV[29} bit 0 controls direction normal or reversed.
Acceleration is controlled by CV[3]
Lights on NodeMCU ports D0 and D3 are "forward and "backwards" lights, switched on by f0. (connect LED to 3.3V!!. )
D8 is a servo output that controls the locomotive motor. 
I use the servo motor BECC interface as a 5V source powered by the LIPO used to power the motors. 
MFRC522 reader uses D1,D2 D5,D6 D7

TODO
The code retains the SV settings to set I/O directions and servos, as well as the Loconet LocoIO programming interface. 
This code needs eventually to be removed, and settings for the servo and two lights hard coded. 


