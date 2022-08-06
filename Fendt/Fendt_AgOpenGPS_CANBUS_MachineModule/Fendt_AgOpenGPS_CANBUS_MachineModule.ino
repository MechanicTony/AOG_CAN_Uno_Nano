/*
06.08.2022
Fendt K_Bus code for use with AgOpenGPS
like all Arduino code, copyed and pasted from everywhere

Any questions ask on the AgOpenGPS forum, CANBUS section

Connected to AgOpenGPS as machine module via USB, two relay contacts are connected to standard AutoSteer PCB:
- Small GO will turn SteerSW relay ON
- Small END will turn SteerSW relay OFF
- Big GO will turn WorkSW relay ON
- Big END will turn WorkSW relay OFF 

Automatic operation of Big GO/END via AgOpenGPS:
- If hyd lift is enabled, auto sections is ON, and hyd lift is ON, AgOpen will press the big GO/END via hitch lift
- If hyd lift is enabled, and "Invert Relays" is ON, AgOpen will press the big GO/END via secton control (Section 1)

Note: Machine Config User1 is used to set what buttons you would like AgOpen to press via CAN.
User1 = 1 is the Left (Green) hydralic +/- on the drive handle
User1 = 2 is the Right(Red) hydralic +/- on the drive handle
User1 = Anything else is the Big Go/End

*/
#include <mcp_can.h>                                         
#include <SPI.h>
#include <EEPROM.h> 
#define EEP_Ident 0x5401 
  
//******** User Settings **********************************************

#define CAN0_INT 2        // Interrupt pin (Check CAN Board)    Standard = 2
MCP_CAN CAN0(10);         // Chip Select pin (Check CAN Board)  Standard = 10 or 9

#define ModuleSpeed MCP_16MHZ   // Big UNO shaped board
//#define ModuleSpeed MCP_8MHZ    // Small blue CAN board

#define WorkSW_PIN 7      // Pin D7 Work Switch                                         
#define SteerSW_PIN 8     // Pin D8 Steer Switch                                       

#define Model 0           // Model 0 = Com2&3 Fendt 100kbs K-Bus
                          // Model 1 = SCR/S4 Fendt 250kbs K-Bus

#define relayON 1         // 1 = High is ON, 0 = Low is ON   

#define onDelay 0         // WorkSW on delay
#define offDelay 0        // WorkSW off delay                   

bool deBug = false;                         
//bool deBug = true;        //Prints the CAN messages recived after the filters (Only button data should be recived)

//*********************************************************************

#define bootText1 "Debug mode, waiting for CAN data (Note: Filters should remove all messages except button data.\r\n"
#define bootText2 "Waiting for AgOpenGPS and/or CAN Data.\r\n"

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];

bool workSwitchCAN;
bool workRelayControl;
unsigned long workTriggerTime;

boolean goDown = false, endDown = false , bitState = false, bitStateOld = false;  //CAN Hitch Control

uint16_t modelID;
byte goPress[8]   = {0x15, 0x00, 0x00, 0xCA, 0x80, 0x01, 0x00, 0x00} ;    //  press big go
byte goLift[8]    = {0x15, 0x00, 0x00, 0xCA, 0x00, 0x02, 0x00, 0x00} ;    //  lift big go
byte endPress[8]  = {0x15, 0x00, 0x00, 0xCA, 0x80, 0x03, 0x00, 0x00} ;    //  press big end
byte endLift[8]   = {0x15, 0x00, 0x00, 0xCA, 0x00, 0x04, 0x00, 0x00} ;    //  lift big end

//Variables for config - 0 is false  
  struct Config {
      uint8_t raiseTime = 2;
      uint8_t lowerTime = 4;
      uint8_t enableToolLift = 0;
      uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

      uint8_t user1 = 0; //user defined values set in machine tab
      uint8_t user2 = 0;
      uint8_t user3 = 0;
      uint8_t user4 = 0;

  };  Config aogConfig;   //4 bytes
  
  const unsigned int LOOP_TIME = 40;      
  unsigned long lastCurrentTime = LOOP_TIME;
  unsigned long currentTime = LOOP_TIME;
  uint16_t count = 0;
  uint16_t countHyd = 0;

  //Comm checks
  uint8_t watchdogTimer = 0; //make sure we are talking to AOG
  uint8_t serialResetTimer = 0; //if serial buffer is getting full, empty it

  bool isRaise = false;
  bool isLower = false;
  
   //Communication with AgOpenGPS
  int16_t temp, EEread = 0;

   //Parsing PGN
  bool isPGNFound = false, isHeaderFound = false;
  uint8_t pgn = 0, dataLength = 0, idx = 0;
  int16_t tempHeader = 0;
  
  uint8_t AOG[] = {0x80,0x81, 0x7f, 0xED, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };

  //The variables used for storage
  uint8_t relayHi=0, relayLo = 0, tramline = 0, uTurn = 0, hydLift = 0, geoStop = 0;
  float gpsSpeed;
  
  uint8_t raiseTimer = 0, lowerTimer = 0, lastTrigger = 0;
  
//reset function
void(* resetFunc) (void) = 0;

void setup()
{
  Serial.begin(38400);
  while (!Serial) { ; } // wait for serial port to connect. Needed for native USB

      EEPROM.get(0, EEread);     // read identifier

      if (EEread != EEP_Ident)   // check on first start and write EEPROM
      {
          EEPROM.put(0, EEP_Ident);
          EEPROM.put(6, aogConfig);
      }
      else
      {
          EEPROM.get(6, aogConfig);
      }
      
// Fendt Com2&3 is 100kbs K-BUS & CAN ID:61F        
  if(Model == 0)  
  {
    if(CAN0.begin(MCP_STDEXT, CAN_100KBPS, ModuleSpeed) == CAN_OK)   
      Serial.println("MCP2515 Initialized Successfully!");
    else
      Serial.println("Error Initializing MCP2515...");  

  CAN0.init_Mask(0,0,0x07FFFF00);       // Init first mask...     - Makes filter look at the ID & first data byte    
  CAN0.init_Filt(0,0,0x061F1500);       // Init first filter...   - Make sure the CAN ID = 61F & first data byte = 0x15  
  CAN0.init_Filt(1,0,0x061F1500);       // Init second filter...  - As above
  
  CAN0.init_Mask(1,0,0x07FFFF00);       // Init second mask...    - As above
  CAN0.init_Filt(2,0,0x061F1500);       // Init third filter...   - As above
  CAN0.init_Filt(3,0,0x061F1500);       // Init fouth filter...   - As above
  CAN0.init_Filt(4,0,0x061F1500);       // Init fifth filter...   - As above
  CAN0.init_Filt(5,0,0x061F1500);       // Init sixth filter...   - As above  

  modelID = 0x61F;

  if(aogConfig.user1 == 1){
   goPress[1]  = 0x4A; //Com2&3 Drive Handle Green
   goLift[1]   = 0x4A; //Com2&3 Drive Handle Green
   endPress[1] = 0x4A; //Com2&3 Drive Handle Green
   endLift[1]  = 0x4A; //Com2&3 Drive Handle Green

   goPress[4]  = 0x00; 
   goLift[4]   = 0x00; 
   endPress[4] = 0x00; 
   endLift[4]  = 0x00;  

   goPress[5]  = 0x8B; //Com2&3 Drive Handle Green - Press
   goLift[5]   = 0x57; //Com2&3 Drive Handle Green Release
   endPress[5] = 0x0F; //Com2&3 Drive Handle Green + Press
   endLift[5]  = 0x57; //Com2&3 Drive Handle Green Release 

   goPress[6]  = 0x39; //Com2&3 Drive Handle Green - Press
   goLift[6]   = 0x15; //Com2&3 Drive Handle Green Release
   endPress[6] = 0x1D; //Com2&3 Drive Handle Green + Press
   endLift[6]  = 0x15; //Com2&3 Drive Handle Green Release   
  }
  
  else if(aogConfig.user1 == 2){
   goPress[1]  = 0x49; //Com2&3 Drive Handle Red
   goLift[1]   = 0x49; //Com2&3 Drive Handle Red
   endPress[1] = 0x49; //Com2&3 Drive Handle Red
   endLift[1]  = 0x49; //Com2&3 Drive Handle Red  

   goPress[4]  = 0x00; 
   goLift[4]   = 0x00; 
   endPress[4] = 0x00; 
   endLift[4]  = 0x00;  

   goPress[5]  = 0x8B; //Com2&3 Drive Handle Red - Press
   goLift[5]   = 0x57; //Com2&3 Drive Handle Red Release
   endPress[5] = 0x0F; //Com2&3 Drive Handle Red + Press
   endLift[5]  = 0x57; //Com2&3 Drive Handle Red Release 

   goPress[6]  = 0x39; //Com2&3 Drive Handle Red - Press
   goLift[6]   = 0x15; //Com2&3 Drive Handle Red Release
   endPress[6] = 0x1D; //Com2&3 Drive Handle Red + Press
   endLift[6]  = 0x15; //Com2&3 Drive Handle Red Release
  }
  
  else{
   goPress[1]  = 0x33; //Com2&3 Big GO
   goLift[1]   = 0x33; //Com2&3 Big GO
   endPress[1] = 0x34; //Com2&3 Big END
   endLift[1]  = 0x34; //Com2&3 Big END
  }    

  goPress[2]  = 0x1E; //Com2&3 Commands
  goLift[2]   = 0x1E; 
  endPress[2] = 0x1E; 
  endLift[2]  = 0x1E;  
           
  }

// Fendt SCR/S4 is 250kbs K-BUS & CAN ID:613    
  else if(Model == 1)  
  {
    if(CAN0.begin(MCP_STDEXT, CAN_250KBPS, ModuleSpeed) == CAN_OK)
      Serial.println("MCP2515 Initialized Successfully!");
    else
      Serial.println("Error Initializing MCP2515...");

  CAN0.init_Mask(0,0,0x07FFFF00);       // Init first mask...     - Makes filter look at the ID & first data byte    
  CAN0.init_Filt(0,0,0x06131500);       // Init first filter...   - Make sure the CAN ID = 613 & first data byte = 0x15  
  CAN0.init_Filt(1,0,0x06131500);       // Init second filter...  - As above
  
  CAN0.init_Mask(1,0,0x07FFFF00);       // Init second mask...    - As above
  CAN0.init_Filt(2,0,0x06131500);       // Init third filter...   - As above
  CAN0.init_Filt(3,0,0x06131500);       // Init fouth filter...   - As above
  CAN0.init_Filt(4,0,0x06131500);       // Init fifth filter...   - As above
  CAN0.init_Filt(5,0,0x06131500);       // Init sixth filter...   - As above  

  modelID = 0x613;

  if(aogConfig.user1 == 1){
   goPress[1]  = 0x26; //SCR/S4 Drive Handle Green -
   goLift[1]   = 0x26; //SCR/S4 Drive Handle Green -
   endPress[1] = 0x25; //SCR/S4 Drive Handle Green +
   endLift[1]  = 0x25; //SCR/S4 Drive Handle Green + 
  }
  else if(aogConfig.user1 == 2){
   goPress[1]  = 0x29; //SCR/S4 Drive Handle Red -
   goLift[1]   = 0x29; //SCR/S4 Drive Handle Red -
   endPress[1] = 0x28; //SCR/S4 Drive Handle Red +
   endLift[1]  = 0x28; //SCR/S4 Drive Handle Red +
  }
  else{
   goPress[1]  = 0x20; //SCR/S4 Big GO
   goLift[1]   = 0x20; //SCR/S4 Big GO
   endPress[1] = 0x21; //SCR/S4 Big END
   endLift[1]  = 0x21; //SCR/S4 Big END
  }    

  goPress[2]  = 0x06; //SCR/S4 Commands
  goLift[2]   = 0x06; 
  endPress[2] = 0x06; 
  endLift[2]  = 0x06;  
            
  }
   
  CAN0.setMode(MCP_NORMAL);                                   
  pinMode(CAN0_INT, INPUT);   
                                    
  if(Model == 0) Serial.println("\r\nFendt Com2&3 100kbs K-Bus Machine Module.\r\n");
  else if(Model == 1) Serial.println("\r\nFendt SCR/S4 250kbs K-Bus Machine Module.\r\n");
  else Serial.println("\r\nNo Model, set in program and reload.\r\n");
                              
  pinMode(SteerSW_PIN, OUTPUT);                                           
  pinMode(WorkSW_PIN, OUTPUT); 
  digitalWrite(SteerSW_PIN, !relayON);
  digitalWrite(WorkSW_PIN, !relayON); 

  currentTime = millis();
  workTriggerTime = currentTime;
  workSwitchCAN = false;

  if(deBug){
    Serial.println(bootText1);
    aogConfig.enableToolLift = 0;
  }
  else{
    Serial.println(bootText2);
  }

}

void loop()
{
  
  currentTime = millis();
  
//Timed loop triggers every 40 msec ( Looptime setting )
  if (currentTime - lastCurrentTime >= LOOP_TIME)
  {
   lastCurrentTime = currentTime;
    
    //AOG timed loop
    if(count++ > 4) //5hz
    { 
          count = 0;
          //If connection lost to AgOpenGPS, the watchdog will count up 
          if (watchdogTimer++ > 250) watchdogTimer = 12;

          //clean out serial buffer to prevent buffer overflow
          if (serialResetTimer++ > 20)
          {
              while (Serial.available() > 0) Serial.read();
              serialResetTimer = 0;
          }

          if (watchdogTimer > 12)
          {
                relayLo = 0;
                relayHi = 0;               
          }

          //add the checksum
          int16_t CK_A = 0;
          for (uint8_t i = 2; i < sizeof(AOG) - 1; i++)
          {
              CK_A = (CK_A + AOG[i]);
          }
          AOG[sizeof(AOG) - 1] = CK_A;

          if(!deBug){
            Serial.write(AOG, sizeof(AOG));
            Serial.flush();   // flush out buffer
          }
          
    }//AOG timed loop

   //Hitch Control
   if((aogConfig.user1 == 1 || aogConfig.user1 == 2) && Model == 0)
   {
    if(countHyd++ > 12) //500ms
    {
      countHyd = 0;
      if (goDown)   liftGo();   //Lift Go button if pressed
      if (endDown)   liftEnd(); //Lift End button if pressed
    }    
   }
   
   else{
    if (goDown)   liftGo();   //Lift Go button if pressed
    if (endDown)   liftEnd(); //Lift End button if pressed
   }

    //If Invert Relays is selected in hitch settings, Section 1 is used as trigger.
    if (aogConfig.isRelayActiveHigh == 1)
    {
      bitState = (bitRead(relayLo, 0));
    }
    //If not selected hitch command is used on headland used as Trigger  
    else
    {
      if (hydLift == 1) bitState = 1;
      if (hydLift == 2) bitState = 0;    
    }
    //Only if tool lift is enabled AgOpen will press GO/END via CAN
    if (aogConfig.enableToolLift == 1)
    {
      if (bitState  && !bitStateOld) pressGo(); //Press Go button
      if (!bitState && bitStateOld) pressEnd(); //Press End button
    }

    bitStateOld = bitState;
         
   //Check if work relay needs changing
     if (workSwitchCAN != workRelayControl)
     {   
      if(workSwitchCAN == 1 && (currentTime - workTriggerTime >= onDelay))
      {
        workRelayControl = workSwitchCAN;
        digitalWrite(WorkSW_PIN, relayON);
      }
      else if(workSwitchCAN == 0 && (currentTime - workTriggerTime >= offDelay))
      {
        workRelayControl = workSwitchCAN;
        digitalWrite(WorkSW_PIN, !relayON); 
      }      
     }//End work relay needs change
     
  }//End Timed loop
  
//Read CAN Module  
  if(!digitalRead(CAN0_INT))                                    
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);  
                              
    if(deBug)
    {
      Serial.print("ID: ");
      Serial.print(rxId, HEX);
      Serial.print(" Data: ");
      for(int i = 0; i<len; i++)      // Print each byte of the data
      {
        Serial.print(rxBuf[i], HEX);
        Serial.print(", ");
      }     
    }
    
//SCR/S4 Armrest Buttons     
    if(rxId == 0x613 && rxBuf[0] == 0x15 && Model == 1)                          
    {
    //Steer Switch Relay 
      if (rxBuf[1] == 0x22 && rxBuf[4] == 0x80)       //Small Go
      {
        digitalWrite(SteerSW_PIN, relayON);
        if(deBug) Serial.print("\t\tSmall GO Pressed");                                                 
      } 
         
      if (rxBuf[1] == 0x23 && rxBuf[4] == 0x80)       //Small END
      {
        digitalWrite(SteerSW_PIN, !relayON);
        if(deBug) Serial.print("\t\tSmall END Pressed");                                                 
      }

    //Work Switch Relay 
      if (rxBuf[1] == 0x20 && rxBuf[4] == 0x80)       //Big Go
      {
        workSwitchCAN = 1;
        workTriggerTime = millis();
        if(deBug) Serial.print("\t\tBig GO Pressed");                                                   
      } 
         
      if (rxBuf[1] == 0x21 && rxBuf[4] == 0x80)       //Big END
      {
        workSwitchCAN = 0;
        workTriggerTime = millis();
        if(deBug) Serial.print("\t\tBig END Pressed");                                                  
      }
      
    }//End if SCR/S4
    
//Com3 Armrest Buttons       
    if(rxId == 0x61F && rxBuf[0] == 0x15 && Model == 0)                        
    {
    //Steer Switch Relay 
      if (rxBuf[1] == 0x35 && rxBuf[4] == 0x80)       //Small Go
      {
        digitalWrite(SteerSW_PIN, relayON);
        if(deBug) Serial.print("\t\tSmall GO Pressed");                                                 
      } 
         
      if (rxBuf[1] == 0x36 && rxBuf[4] == 0x80)       //Small END
      {
        digitalWrite(SteerSW_PIN, !relayON);
        if(deBug) Serial.print("\t\tSmall END Pressed");                                                 
      }

    //Work Switch Relay 
      if (rxBuf[1] == 0x33 && rxBuf[4] == 0x80)       //Big Go
      {
        workSwitchCAN = 1;
        workTriggerTime = millis();
        if(deBug) Serial.print("\t\tBig GO Pressed");                                                 
      } 
         
      if (rxBuf[1] == 0x34 && rxBuf[4] == 0x80)       //Big END
      {
        workSwitchCAN = 0;
        workTriggerTime = millis();
        if(deBug) Serial.print("\t\tBig END Pressed");                                                 
      }
      
    }//End if Com3
        
   if(deBug) Serial.println("");
         
  }//End Read CAN Module

// Serial Receive
      //Do we have a match with 0x8081?    
      if (Serial.available() > 4 && !isHeaderFound && !isPGNFound)
      {
          uint8_t temp = Serial.read();
          if (tempHeader == 0x80 && temp == 0x81)
          {
              isHeaderFound = true;
              tempHeader = 0;
          }
          else
          {
              tempHeader = temp;     //save for next time
              return;
          }
      }

      //Find Source, PGN, and Length
      if (Serial.available() > 2 && isHeaderFound && !isPGNFound)
      {
          Serial.read(); //The 7F or less
          pgn = Serial.read();
          dataLength = Serial.read();
          isPGNFound = true;
          idx = 0;
      }

      //The data package
      if (Serial.available() > dataLength && isHeaderFound && isPGNFound)
      {
          if (pgn == 239) // EF Machine Data
          {
              uTurn = Serial.read();
              gpsSpeed = (float)Serial.read();//actual speed times 4, single uint8_t

              hydLift = Serial.read();
              tramline = Serial.read();  //bit 0 is right bit 1 is left

              //just get the rest of bytes
              Serial.read();   //high,low bytes   
              Serial.read();

              relayLo = Serial.read();          // read relay control from AgOpenGPS
              relayHi = Serial.read();

              //Bit 13 CRC
              Serial.read();

              //reset watchdog
              watchdogTimer = 0;

              //Reset serial Watchdog  
              serialResetTimer = 0;

              //reset for next pgn sentence
              isHeaderFound = isPGNFound = false;
              pgn = dataLength = 0;
          }

          else if (pgn == 238) //EE Machine Settings 
          {
              aogConfig.raiseTime = Serial.read();
              aogConfig.lowerTime = Serial.read();
              Serial.read();

              //set1 
              uint8_t sett = Serial.read();  //setting0     
              if (bitRead(sett,0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;
              if (bitRead(sett,1)) aogConfig.enableToolLift = 1; else aogConfig.enableToolLift = 0;

              aogConfig.user1 = Serial.read();
              aogConfig.user2 = Serial.read();
              aogConfig.user3 = Serial.read();
              aogConfig.user4 = Serial.read();

              //crc
              //udpData[13];        //crc
              Serial.read();

              //save in EEPROM and restart
              EEPROM.put(6, aogConfig);
              resetFunc();

              //reset for next pgn sentence
              isHeaderFound = isPGNFound = false;
              pgn = dataLength = 0;
          }

          else //nothing found, clean up
          {
              isHeaderFound = isPGNFound = false;
              pgn = dataLength = 0;
          }
      }//End Serial Data Receive
    
}//End Main Loop 

//Fendt K-Bus Buttons
  void pressGo()
  {                                     
    CAN0.sendMsgBuf(modelID, 0, 8, goPress);
    goDown = true;
    countHyd = 0;
  }

  void liftGo()
  {
    CAN0.sendMsgBuf(modelID, 0, 8, goLift);
    goDown = false;
  }

  void pressEnd() 
  {
    CAN0.sendMsgBuf(modelID, 0, 8, endPress);
    endDown = true;
    countHyd = 0;
  }

  void liftEnd()
  {
    CAN0.sendMsgBuf(modelID, 0, 8, endLift);
    endDown = false;
}
