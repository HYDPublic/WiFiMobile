
#include <SPI.h>
#include <MFRC522.h> 
 
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUDP.h>
#include <Servo.h> 
#include <EEPROM.h>

#include "SV.h";
#include "Defaults.h";
#include "OPC_Codes.h";
#include "Secrets.h"; 

String wifiSSID = SSID_RR;
String wifiPassword = PASS_RR;  
int wifiaddr;

WiFiUDP UDP;  
WiFiUDP UDP2;
IPAddress ipBroad; 
const int port = 1235;

#include "Subroutines.h"; 

///HTML server
WiFiServer server(80);


//otherstuff

byte udpTestPacket0[] = {0xB2, 0x02, 0x50, 0x1F};
byte udpTestPacket1[] = {0xB2, 0x02, 0x40, 0x0F};
byte UDPSendPacket[16]  ;
boolean Phase = 0 ;

SPISettings settingsRFID (40000000, LSBFIRST,SPI_MODE0); 

#include "Motor.h"; 
#include "RFID_Subs.h";
void setup() {
  POWERON=true;
  Ten_Sec= 10000;
 LOCO = 1;
  Serial.begin(115200);
  Serial.print(F("Initialising. Trying to connect to:"));
  Serial.println(SSID_RR);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  while (WiFi.status() != WL_CONNECTED) {delay(500);Serial.print(".");}
  Serial.println();
  Serial.println(WiFi.localIP());
    ipBroad=WiFi.localIP();
    Serial.print(" last byte:");
    wifiaddr=ipBroad[3];
  Serial.println(wifiaddr);
  ipBroad[3]=255; //Set broadcast to local broadcast ip e.g. 192.168.0.255

  // Start the server
    server.begin();
    Serial.println("Server started");
    // start UDP server
    UDP.begin(port);
  

//  Setup addresses


        Data_Updated= false; 
        EEPROM.begin(512);

    if((EEPROM.read(255) == 0xFF) && (EEPROM.read(256) == 0xFF)){ //eeprom empty, first run. Can also set via CV[8]=8
                      Serial.println(" ******* EPROM EMPTY....Setting Default EEPROM values *****");
                      SetDefaultSVs();
                      WriteSV2EPROM(); 
                      Data_Updated= true; 
                      EEprom_Counter=millis()+Ten_Sec;
                      //EEPROM.commit();
                      //delay(100);
                                } //if
     

     EPROM2SV();     //Write to the SV and CV registers
      SV[2]=wifiaddr;  //overwrite this one though!!
      ShowSVS();  
      MyLocoLAddr=CV[18]+((CV[17]&0x3F)*256);
      MyLocoAddr= CV[1];/// 
      FullBoardAddress=AddrFull(SV[2],SV[1]);

//   * Setup rfidstuff *************************

  SPI.begin();        // Init SPI bus// 
  SPI.setFrequency(1000000);
  //SPI.beginTransaction(settingsRFID);
    mfrc522.PCD_Init(); // Init MFRC522 card
    byte readReg = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
    if((readReg == 0x00) || (readReg == 0xFF)){ //missing reader
      Serial.println("Reader absent");
    } else {
      Serial.println("Reader present");
      bReaderActive = true;  
      //If you set Antenna Gain to Max it will increase reading distance
      mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
           }

  //++++++++++++++++++++Print Debug and for information stuff    +++++++++++++++++++++++++++++
  Serial.print("LocoIO Board address :");
  Serial.print(SV[1]);
  Serial.print("-");
  Serial.println(SV[2]);
  Serial.print(" Servos ");
   for ( int i=1;i<=8;i++) {
    if ((((SV[3*i]& 0x88) ==0x88))){   //OUTPUT & pulse TYPE == SERVO
    Serial.print(" D"); 
    Serial.print(i);
    Serial.print(":=");
    Serial.print(SensorAddress(i));
    }
   }
  Serial.println("   ");
  Serial.print(" Switches ");
   for ( int i=1;i<=8;i++) {
      if ((SV[(3*i)]& 0x88) == 0x00)    {   // only do this if this port is a "Switch Report" Serial.print(" :"); 
    Serial.print(" D"); 
    Serial.print(i);
    Serial.print(":=");
     Serial.print(SensorAddress(i));
      }
   }
  Serial.println("   ");
  Serial.print(" Outputs ");          
   for ( int i=1;i<=8;i++) {
      if ((SV[(3*i)]& 0x88) == 0x80)    {   // "OUTUT and NOT PULSE" 
    Serial.print(" D"); 
    Serial.print(i);
    Serial.print(":=");
     Serial.print(SensorAddress(i));
      }
   }
   
  Serial.println("   ");
  Serial.print(" Using Long 'Locomotive Address' is");
  Serial.print (CV[18]+((CV[17]&0x3F)*256));
  Serial.print(" CV17=");
  Serial.print(CV[17]); 
  Serial.print(" CV18=");
  Serial.print(CV[18]); 
  Serial.print(" Short 'Locomotive Address' is");
  Serial.print (CV[1]);
  Serial.println();
  SetMyPorts();
// if(LOCO==0){
// for (int i=1 ; i<=8; i++) {
//    SDelay[i]=1000;
//    SetServo(i,1);
//      }
// }
  if (!myservo8.attached()) {myservo8.attach(D8); }// re attach loco motor servos in case they have been switched off
  digitalWrite (BlueLed, LOW) ;  /// turn On  


 ReadInputPorts();
 for (int i=0 ; i<=8; i++) {
   lastButtonState[i]= buttonState[i]+25;  // is not boolean, so simpy make it different
 }
  
if ( LOCO == 1 ) {
  Serial.println("......... Setting Loco defaults");
   Motor_Speed=0;
   Loco_motor_servo_demand=90;
   myservo8.attach(D8);
   myservo8.write(Loco_motor_servo_demand);
   //ServoOffDelay[8]=1;
   //SDelay[8]= 1000;
   servoDelay[8]=millis();
   LocoUpdate;
}


SensorOutput_Inactive = true;


LoopTimer=millis();

}  /// end of setup ///////




void loop() {
 LoopTimer=millis();   
 delay(2); 
   // +++++++++  Set up things that may have been changed...+++++++++
    digitalWrite (BlueLed ,HIGH) ;  ///   turn OFF
    FullBoardAddress=AddrFull(SV[2],SV[1]);
    MyLocoLAddr=CV[18]+((CV[17]&0x3F)*256);
    MyLocoAddr=CV[1];/// 

    if (POWERON) {
      myservo8.attach(D8);
      }
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

   
  // commit the writes to the  Eprom?
    if ((millis()>= EEprom_Counter) & (Data_Updated)){                // commit EEPROM only when needed..
                     Data_Updated=false; 
                     #if _SERIAL_DEBUG
                     Serial.println("Commiting EEPROM");
                     #endif
                     EEPROM.commit();     
                     delay(250);
                     //ESP.reset() ;
                     }    // soft reset? ?? why does doing this eprom commit stop the servo output???
                      
  //re-connect wifi if not connected
    while (WiFi.status() != WL_CONNECTED)                        {
              delay(10);
              #if _SERIAL_DEBUG
              Serial.print("~");
                #endif
              WiFi.mode(WIFI_STA);
              WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str()); }
              
    
 //periodic updates and checks
    if ((millis()>=LocoUpdateTime) & (LocoUpdated)){LocoUpdate(1);}  
    if (millis()>= CVSVUpdateTime){doLocoServo();}   // avoid doing anything when doing cv or sv read writes...
    if (millis()>= CVSVUpdateTime){checkRFID();}  // if we are doing CVSv anything, wait one second before doing any CheckRFID 
 
  UDPFetch(recMessage); //have we recieved data??    
      
         switch (Message_Length){
          case 0: {  // no rx data to work with //if not receiving, do other things...
                     //   ReadInputPorts();

                    }    // end case 0
          break;             
 
         case 2:
                if ((recMessage[0]==0x82) & (recMessage[1]==0x7D)){   //  power off
                   POWERON=false;
                   Motor_Speed=0;
                   myservo8.write(90);
                   delay(150);
                   myservo8.detach(); }
                    if ((recMessage[0]==0x85) & (recMessage[1]==0x7A)){   //  emergency stops
                   POWERON=false;
                   Motor_Speed=0;
                   myservo8.write(90);
                   delay(150);
                   myservo8.detach(); }
                if ((recMessage[0]==0x83) & (recMessage[1]==0x7C)){ POWERON=true; }
         
         break;
           case 4:
                Len4commands(recMessage); 
                break;
         case 14:
                Len14Commands(recMessage);
                break;
         case 16:
                Len16Commands(recMessage);
                break;
         default:
            Show_MSG();
                break;
  } // end switch cases

//Serial.print(" c ");
//Serial.print(millis()-LoopTimer);


 
  
} //void loop


void ReadInputPorts(){
//buttonState[1]= digitalRead(D1);
//buttonState[2]= digitalRead(D2);
buttonState[3]= digitalRead(D3);
buttonState[4]= digitalRead(D4);
//buttonState[5]= digitalRead(D5);
//buttonState[6]= digitalRead(D6);
//buttonState[7]= digitalRead(D7);
//buttonState[8]= digitalRead(D8);
// Check for port changes +++++++++++++++++++++++++++++++++++++
            for (int i=1 ; i<8; i++) {  // range 1 to 8 (1=port D1)  so later we can sort an automatic port i/o setup section routine
            if ((SV[(3*i)]& 0x38) == 0x00)    {   // only do this if this port is a "Switch Report" SV3 .. bits 4 5 are 0 =Switch and Request (bit 3) is also 0 
               if (Debounce(i) == i) {
                 lastButtonState[i] = buttonState[i];
                 DebounceTime[i] =0 ;
            #if _SERIAL_DEBUG
          Serial.print ("Change on IO port : ");
          Serial.print(i); 
          Serial.print ("  >Sensor Address:");
          Serial.print(SensorAddress(i));  
          Serial.print(" State");
          Serial.print(buttonState[i]);
          Serial.print (" Config (SV[");
          Serial.print(((3*i))); 
          Serial.print ("]) = ");
          Serial.println(SV[((3*i))]);
         
          #endif  
          setOPCMsg(UDPSendPacket, SensorAddress(i)-1, buttonState[i]);   //setOPCMsg (uint8_t *SendMsg, int i, uint16_t addr, int InputState )
         // Serial.print(")  ");//  dump_byte_array(UDPSendPacket,4);// Serial.println(); 
          UDPSEND(UDPSendPacket,4,2); }}}

    
}
void SetMyPorts(){
  int i;
  //  pinMode(D0, INPUT);   is also red led on some nodemcu boards
 
i=3;
if ((SV[(3*i)]& 0x88) == 0x80)  { pinMode(D3, OUTPUT);}  // "OUTUT and NOT PULSE" 
i=4;
if ((SV[(3*i)]& 0x88) == 0x80)  { pinMode(D0, OUTPUT);}  // CHEAT    FIX later..using d0 for d4 for loco leds OUTUT and NOT PULSE"   
pinMode(BlueLed, OUTPUT);  //is also D4...

 // pinMode(D10, INPUT);  // stops serial if set...
    
  Serial.println("Ports set");
}
//------


