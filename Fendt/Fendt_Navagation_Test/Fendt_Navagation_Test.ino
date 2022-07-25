
#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2             // Set INT to pin 2
MCP_CAN CAN0(10);              // Set CS to pin 10

const int steerSwitch = A1;    // On/Off switch connected to A1
int steerState = 0; 

const int pot = A0;            // Potentiometer connected to A0
int potValue = 0; 
int16_t setCurve = 0;

byte data[6] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0} ; //Fendt Curve Command at 40msec
byte data1[8] = {0x00, 0x00, 0xC0, 0x0C, 0x00, 0x17, 0x02, 0x20} ;  //Fendt Set, 2C Navagation Controller
              
unsigned int scanLoopInterval  = 40;
unsigned long scanLoopMs = millis();

//--------------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);

  pinMode(steerSwitch,INPUT_PULLUP);

  if (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else {
    Serial.println("Error Initializing MCP2515...");
  }

  CAN0.setMode(MCP_NORMAL);

  pinMode(CAN0_INT, INPUT);

  delay (1000);
      CAN0.sendMsgBuf(0x18EEFF2C, 1, 8, data1);   //Claim Address
      delay (100);
      Serial.println("Setup Done");          
}

//---------------------------------------------------------------------------------------------------------------------------------

void loop()
{

potValue = analogRead(pot);
setCurve = map(potValue,0,1023,-32768,32767);

steerState = digitalRead(steerSwitch);
steerState = (steerState + 2);

  if ((millis() - scanLoopMs) > scanLoopInterval) {   // Send data
    scanLoopMs = millis();
    data[0] = 5;
    data[1] = 9;
    data[2] = steerState;
    data[3] = 10;
    if (steerState == 3){
      data[4] = highByte(setCurve);
      data[5] = lowByte(setCurve);
    }
    else{
      data[4] = 0;
      data[5] = 0;
    }
    sendCAN_Msg();
  }
  
}

//---------------------------------------------------------------------------------------------------------------------------------

void sendCAN_Msg() {

  byte sndStat = CAN0.sendMsgBuf(0x8CEFF02C, 1, 6, data);
  if (sndStat == CAN_OK) {
    Serial.print(potValue);
    Serial.print("'");
    Serial.print(setCurve);
    Serial.print("'");
    Serial.print(steerState);
    Serial.print("'");
    Serial.println(" Sent Successfully!");
  }
  else {
    Serial.print(potValue);
    Serial.print("'");
    Serial.print(setCurve);
    Serial.print("'");
    Serial.print(steerState);
    Serial.print("'");
    Serial.println("Error Sending Message...");
  }
}
