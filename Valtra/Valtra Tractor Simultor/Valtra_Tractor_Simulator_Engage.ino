//*******************************************
//Valtra Demo AC Curve Command Generator
//
//Current Wheel Angle Via Pot A0
//Valve State Via Switch A1
//Steer Engage Command Via Button A2
//Displays Set Curve from AOG in Serial Monitor
//*******************************************

#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2                    // Set INT to pin 2
MCP_CAN CAN0(10);                     // Set CS to pin 10

const int steerSwitch = A1;   
byte steerState = 0; 

const int engageButton = A2;   
byte engageState = 0;

const int pot = A0;   
int potValue = 0; 
unsigned int setCurve = 0;

unsigned int curveCommand;

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

byte data[8] = {0, 0, 0, 0, 0, 0, 0, 0} ;                   //Set Curve Data
byte data1[8] = {15, 96, 1, 255, 255, 255, 255, 255} ;      //Engage Data         

unsigned int scanLoopInterval  = 80;
unsigned long scanLoopMs = millis();

//---------------------------------------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);

  pinMode(steerSwitch,INPUT_PULLUP);
  pinMode(engageButton,INPUT_PULLUP);

  if (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else {
    Serial.println("Error Initializing MCP2515...");
  }

  CAN0.setMode(MCP_NORMAL);

  pinMode(CAN0_INT, INPUT);

  Serial.println("Valtra Tractor Simulator");

delay (1000);
         
}

//---------------------------------------------------------------------------------------------------------------------------------

void loop()
{

potValue = analogRead(pot);
potValue = map(potValue,0,1023,0,4000);
setCurve = (potValue + 30128);

steerState = digitalRead(steerSwitch);
if (steerState == 0) steerState = 80; // 0 Or 80 = Not Ready Stop
if (steerState == 1) steerState = 16; // 16 Or 20 = OK

  if ((millis() - scanLoopMs) > scanLoopInterval) {   // Send data
    scanLoopMs = millis();
    data[0] = lowByte(setCurve);
    data[1] = highByte(setCurve);
    data[2] = steerState;
    data[3] = 255;
    data[4] = 255;
    data[5] = 255;
    data[6] = 255;
    data[7] = 255;
    CAN0.sendMsgBuf(0x8CAC1C13, 1, 8, data);

  engageState = digitalRead(engageButton);

if(engageState == 0) CAN0.sendMsgBuf(0x18EF1C32, 1, 8, data1);

  }
  
 if(!digitalRead(CAN0_INT))                // If pin 2 is low, read receive buffer
    {
      CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
      Serial.print("ID: ");
      Serial.print(rxId, HEX);
      Serial.print(" Data: ");
      for(int i = 0; i<len; i++)           // Print each byte of the data
      {
        Serial.print(rxBuf[i], DEC);
        Serial.print(" ");
      }

if (rxId == 0x8CAD131C){
      Serial.print("\t\t AOG Set Curve = ");
      curveCommand = ((rxBuf[1] << 8) + rxBuf[0]);
      Serial.print(curveCommand);
}
      
      Serial.println();
    }
  
}
