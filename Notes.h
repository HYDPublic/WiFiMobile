/*  REFERENCE data.. Do not INCLUDE in the main file....

This software sets up up a single loco command station.
CV[1] is the loco address, but also the "slot" that the CS uses to command its "locomotive".
The Locomotive uses a single servo connected to D8 for speed and direction. The speed at hwich the servo responds is set by CV[3]. 
CV[29] bit 0 controls the "direction" response.
Functions F3 and F4 control Leds on Ports.



The module also has RFID Sensor interface that sends a B2 Message to rocrail. (for sensors).
The RFID sensor sends an unique address to Rocrail for each Tag seen.

/**
  * Programming SV’s
  *
  *     The SV’s in LocoIO can be programmed using Loconet OPC_PEER_XFER messages.
  In this "mobile version" the SV settings do not do much, but in the Static Version of this code, they control the 8 ports and determine if they are Servos, inputs or outputs..
  *
  Programming CV's.. The code accepts and responds to CV[1], as well as CV29 (direction bit), and CV[3} to set the speed at which the servo responds to new settings.
  
  
  BIG BUT.... Currently this code misses some UDP commands, making it unreliable. It is hoped that we can speed up the RFID reader so that codes will not be missed.
