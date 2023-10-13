/*pollSensors.cpp

Created May 15, 2023 by Joel Legassie

Contains functions used to gather data from MXC4005XC-B Accelerometer

void getAccAxes(uint8_t Port)   -- Controls flow of the sensor reading 
    calls:
    int16_t readAccReg(uint8_t Port, uint8_t r)  --  gets raw bits from sensor  
        calls:
            void changeI2CPort(uint8_t I2CPort) -- sets I2C port on multiplexor     
    int16_t getAxisAcc(int16_t axisHi, int16_t axisLo)  --  Create acceleration vector from raw bits (incl time data)

accVector movingAvg(uint8_t vecIndex) -- Averages three samples to create a moving average vector
vectortoBytes(accVector vector, uint8_t sensorIndex) -- makes byte array for TX

*/
// #include <Arduino.h>
// #include <WiFi.h>
#include "basic.h"
// #include <Wire.h>
// #include <stdlib.h>
// #include "secrets.h"
// #include <math.h>
// #include "Adafruit_VL53L0X.h"


/************************
 * getDist()
*************************/

// uint8_t getDist(Adafruit_VL53L0X toF) {

//   Serial.println();
//   Serial.print("Get distance");
//   Serial.println();

//   toF.getSingleRangingMeasurement(&measure, true);
//   //Wire.begin(I2C_SDA, I2C_SCL);

//   //uint8_t dist = getDist(toF, measure);    //Get a distance measurement from the Tof sensor
//   uint16_t dist16 = measure.RangeMilliMeter;

//   Serial.print("raw distance: ");
//   Serial.println(dist16, HEX);

//   uint8_t dist = (uint8_t) ((dist16) >> 8 );

//   Serial.print("Scaled istance: ");
//   Serial.println(dist, HEX);

//   return dist;

// }

/************************
 * getAccAxes()
*************************/

accVector getAccAxes(uint8_t Port) {
 //Read Axes of Acc1
    Serial.println();
    Serial.print("accVector getAccAxes(), Port: ");
    Serial.println(Port, DEC);

  // Serial.println();
    
  #ifdef DEBUG
    Serial.println();
    Serial.print("accVector getAccAxes(), Port: ");
    Serial.println(Port, DEC);
  #endif /*DEBUG*/
    
    accVector accVector;

    //Get X register values
    //XHi
    int16_t XHi = readAccReg(Port, 3);

    #ifdef DEBUG
      Serial.print("XHi: ");
      Serial.println(XHi, DEC);
    #endif /*DEBUG*/

    //XLo  
    int16_t XLo = readAccReg(Port, 4);

    #ifdef DEBUG
      Serial.print("XLo: ");
      Serial.println(XLo, DEC);
    #endif /*DEBUG*/

    //Combine Hi and Lo to get axis value
    //Serial.print("X: ");
    accVector.XAcc = getAxisAcc(XHi, XLo);

    #ifdef DEBUG
      Serial.print("accVector.XAcc: ");
      Serial.println(accVector.XAcc, DEC);
    #endif /*DEBUG*/

    //Get Y register values
    //YHi
    int16_t YHi = readAccReg(Port, 5);

    #ifdef DEBUG
      Serial.print("YHi: ");
      Serial.println(YHi, DEC);
    #endif /*DEBUG*/

    //YLo  
    int16_t YLo = readAccReg(Port, 6);

    #ifdef DEBUG
      Serial.print("YLo: ");
      Serial.println(YLo, DEC);
    #endif /*DEBUG*/

    //Combine Hi and Lo to get axis value
    //Serial.print("Y: ");
    accVector.YAcc = getAxisAcc(YHi, YLo);

    #ifdef DEBUG
      Serial.print("accVector.YAcc: ");
      Serial.println(accVector.YAcc, DEC);
    #endif /*DEBUG*/

    //Get Z register values
    //Zi  
    int16_t ZHi = readAccReg(Port, 7);

    #ifdef DEBUG
      Serial.print("ZHi: ");
      Serial.println(ZHi, DEC);
    #endif /*DEBUG*/

    //ZLo  
    int16_t ZLo = readAccReg(Port, 8);

    #ifdef DEBUG
      Serial.print("ZLo: ");
      Serial.println(ZLo, DEC);
    #endif /*DEBUG*/
    //axisAccSerial.print("Z: ");
    //Combine Hi and Lo to get axis value
    accVector.ZAcc = getAxisAcc(ZHi, ZLo);

    #ifdef DEBUG
      Serial.print("accVector.ZAcc: ");
      Serial.println(accVector.ZAcc, DEC);
    #endif /*DEBUG*/

    return accVector;
}

/****************************************
 * readAccReg(uint8_t Port, uint8_t r)
****************************************/

int16_t readAccReg(uint8_t Port, uint8_t r) {

    Serial.println();
    Serial.println("readAccReg(uint8_t Port, int r)");
    // Serial.println();
    // Serial.print("readAccReg(uint8_t Port, int r), TxCount:");
    // Serial.println(txCount, DEC);
    // Serial.print("sensor:");
    // Serial.println(Port, DEC);
    // Serial.print("sampleCount:");
    // Serial.println(sampleCount, DEC);
    // Serial.print("register:");
    // Serial.println(r, DEC);


  #ifdef DEBUG
    Serial.println();
    Serial.println("readAccReg(uint8_t Port, int r)");
    Serial.println();
    Serial.print("readAccReg(uint8_t Port, int r), TxCount:");
    Serial.println(txCount, DEC);
    Serial.print("sensor:");
    Serial.println(Port, DEC);
    Serial.print("sampleCount:");
    Serial.println(sampleCount, DEC);
    Serial.print("register:");
    Serial.println(r, DEC);
  #endif /*DEBUG*/

  int16_t regOut = 0;
  // Serial.print("Port: ");
  // Serial.println(Port, DEC);
  if (Port != I2CPort) {
    I2CPort = Port;
    changeI2CPort(Port);
  }
  
  // Serial.println("Multiplexor Port selected");
  // Serial.println("Send Device Address then register address (r)");

  #ifdef DEBUG
    Serial.println("Multiplexor Port selected");
    Serial.println("Send Device Address then register address (r)");
  #endif /*DEBUG*/

  Wire.beginTransmission(MXCI2CADDR);    //Open TX with start address and stop
  Wire.write(r);                  //Send the register we want to read to the sensor
  
  Serial.print("r transmitted: ");
  Serial.println(r, HEX);

  #ifdef DEBUG
    Serial.print("r transmitted: ");
    Serial.println(r, HEX);
  #endif /*DEBUG*/

  uint8_t error = Wire.endTransmission();  //Send a stop
      if (error == 0) {

        Serial.print("I2C device found at address 0x15 using port ");
        Serial.println(Port, DEC);
        #ifdef DEBUG
          Serial.print("I2C device found at address 0x15\n");
        #endif /*DEBUG*/

      } else {
          Serial.print("I2C Error: ");
          Serial.println(error,HEX);
          #ifdef DEBUG
            
            Serial.print("I2C Error: ");
            Serial.println(error,HEX);
          #endif /*DEBUG*/
          //return -1;
      }

    Wire.requestFrom(MXCI2CADDR, 1, 1);   //Send read request
    while(Wire.available()) {
      regOut = Wire.read();

      Serial.print("Register Output: ");
      Serial.println(regOut, HEX);

      #ifdef DEBUG
        Serial.print("Register Output: ");
        Serial.println(regOut, HEX);
      #endif /*DEBUG*/
    }
    
    #ifdef DEBUG
      Serial.println();
    #endif /*DEBUG*/

    return regOut;
}

/****************************************
 * changeI2CPort(uint8_t I2CPort)
****************************************/

void changeI2CPort(uint8_t I2CPort) {   //Change the port of the I2C multiplexor
  Serial.println("changeI2CPort()");
  Serial.print("I2CPort: ");
  Serial.println((I2CPort), DEC);
  Serial.print("I2CPort Shifted: ");
  Serial.println((1 << I2CPort), DEC);
  Wire.beginTransmission(I2CADDR);
  Wire.write(1 << I2CPort);
  Wire.endTransmission();

}

/********************************************
 * getAxisAcc(int16_t axisHi, int16_t axisLo)
*********************************************/

int16_t getAxisAcc(int16_t axisHi, int16_t axisLo) {

  // Serial.println("getAxisAcc(int16_t axisHi, int16_t axisLo)");
  //   Serial.print("axisAccHi First: ");
  //   Serial.println(axisHi, DEC);
  //   Serial.print("axisAccLo First: ");
  //   Serial.println(axisLo, DEC);
  #ifdef DEBUG
    Serial.println();
    Serial.println("getAxisAcc(int16_t axisHi, int16_t axisLo)");
    Serial.print("axisAccHi First: ");
    Serial.println(axisHi, HEX);
    Serial.print("axisAccLo First: ");
    Serial.println(axisLo, HEX);
  #endif /*DEBUG*/

    int16_t axisAcc = 0;
    if (axisHi > 127) {                  //check for negative values
        // Serial.println("************************************************************");
        // Serial.println("************************************************************");
        // Serial.print("axisHi original: ");
        // Serial.println(axisHi);
        axisHi = axisHi - 0x80;          //subtract the sign bit (128)
        // Serial.print("axisHi modified: ");
        // Serial.println(axisHi);
        axisAcc = axisHi << 4;           //High value 
        // Serial.print("axisHi shifted: ");
        // Serial.println(axisAcc);
        axisAcc = axisAcc + (axisLo >> 4);   //Low value
        axisAcc = axisAcc -2048;          //subtract 2^12 to convert 12 bit 2's complement to 16 bit signed int
        // Serial.print("Negative number: ");
        // Serial.println(axisAcc);
        // Serial.println("************************************************************");
        // Serial.println("************************************************************");
    } else {
        axisAcc = axisHi << 4;           //High value 
        axisAcc = axisAcc + (axisLo >> 4);   //Low value
    }
    
    #ifdef DEBUG
      Serial.print("axisAccHi Shift: ");
      Serial.println(axisAcc, HEX);
    #endif /*DEBUG*/
    
    //axisAcc = axisAcc + (axisLo >> 4);
    
    // Serial.print("axisAccLo: ");
    //   Serial.println((axisLo >> 4), HEX);
    //   Serial.print("axisAcc: ");
    //   Serial.println(axisAcc, HEX);
    //   Serial.println();
    #ifdef DEBUG
      Serial.print("axisAccLo: ");
      Serial.println((axisLo >> 4), HEX);
      Serial.print("axisAcc: ");
      Serial.println(axisAcc, HEX);
      Serial.println();
    #endif /*DEBUG*/

    // Serial.print("axisAcc: ");
    // Serial.println(axisAcc, DEC);
    
    
    int8_t axisAccScaled = axisAcc / 16;   //Divide 16 to reduce 12 but signed 12 bit int (+-2047) to a signed 8bit int (+-127)

    // Serial.print("axisAccScaled: ");
    // Serial.println(axisAccScaled, DEC);
    // Serial.println();

    return axisAccScaled;                  //Return single byte value
  }

/********************************************
 * vectortoBytes(accVector vector)
*********************************************/
void vectortoBytes(accVector vector, uint8_t sensorIndex) {
  // Serial.println();
  // Serial.println("VectortoBytes(accVector vector)");
  
  #ifdef DEBUG
    Serial.println();
    Serial.println("VectortoBytes(accVector vector)");
  #endif /*DEBUG*/

  //char bytes[18];
  
  // int16_t XAccTmp = vector.XAcc;
  // char* XAccBytes = (char*) &XAccTmp;

  // // Serial.print("sizeof XAccBytes: ");
  // //   Serial.println(sizeof(XAccBytes), DEC);
  // //   Serial.print(XAccBytes[0], DEC);
  // //   Serial.print(", ");
  // //   Serial.print(XAccBytes[1], DEC);
  // //   Serial.println();
  
  // #ifdef DEBUG
  //   Serial.print("sizeof XAccBytes: ");
  //   Serial.println(sizeof(XAccBytes), DEC);
  //   Serial.print(XAccBytes[0], HEX);
  //   Serial.print(", ");
  //   Serial.print(XAccBytes[1], HEX);
  //   Serial.println();
  // #endif /*DEBUG*/

  // int16_t YAccTmp = vector.YAcc;
  // char* YAccBytes = (char*) &YAccTmp;

  // #ifdef DEBUG
  //   Serial.print("sizeof YAccBytes: ");
  //   Serial.println(sizeof(YAccBytes), DEC);
  //   Serial.print(YAccBytes[0], HEX);
  //   Serial.print(", ");
  //   Serial.print(YAccBytes[1], HEX);
  //   Serial.println();
  // #endif /*DEBUG*/

  // int16_t ZAccTmp = vector.ZAcc;
  // char* ZAccBytes = (char*) &ZAccTmp;

  // #ifdef DEBUG
  //   Serial.print("sizeof ZAccBytes: ");
  //   Serial.println(sizeof(ZAccBytes), DEC);
  //   Serial.print(ZAccBytes[0], HEX);
  //   Serial.print(", ");
  //   Serial.print(ZAccBytes[1], HEX);
  //   Serial.println();
  // #endif /*DEBUG*/
  
  sensorIndex = sensorIndex*ACCPACKSIZE;
  bytes[0 + (sensorIndex)] = vector.XAcc;
  bytes[1 + (sensorIndex)] = vector.YAcc;
  bytes[2 + (sensorIndex)] = vector.ZAcc;


// Serial.println();
//   Serial.print("Bytes: ");
//   for (int i =0; i < sizeof(bytes); i++) {
//     Serial.println(bytes[i], HEX);
//   }
//   Serial.println();

#ifdef DEBUG
  Serial.println();
  Serial.print("Bytes: ");
  for (int i =0; i < sizeof(bytes); i++) {
    Serial.println(bytes[i], HEX);
  }
  Serial.println();
#endif /*DEBUG*/
}


/********************************************
 * movingAvg(uint8_t sensorIndex)
*********************************************/
accVector movingAvg(uint8_t sensorIndex) {
  
  // Serial.println("movingAvg");
  //   Serial.print("TX number: ");
  //   Serial.println(txCount, DEC);
  //   Serial.println("Sensor: ");
  //   Serial.println(sensorIndex, DEC);
  #ifdef DEBUG
    Serial.println("movingAvg");
    Serial.print("TX number: ");
    Serial.println(txCount, DEC);
    Serial.println("Sensor: ");
    Serial.println(sensorIndex, DEC);
  #endif /*DEBUG*/
  //ACC Values
  accVector movingAvgVect;
  //Floats to hold intermediate values
  float Xholder = 0;
  float Yholder = 0;
  float Zholder = 0;
  //Loop through values to get total
  for (int i =0; i < MOVINGAVGSIZE; i++) {
    Xholder += (float)accVecArray[sensorIndex][i].XAcc;
    Yholder += (float)accVecArray[sensorIndex][i].YAcc; 
    Zholder += (float)accVecArray[sensorIndex][i].ZAcc;   
  }

  #ifdef DEBUG
    Serial.print("Xholder Sum: ");
    Serial.println(Xholder, DEC);
    Serial.print("Yholder Sum: ");
    Serial.println(Yholder, DEC);
    Serial.print("Zholder Sum: ");
    Serial.println(Zholder, DEC);
  #endif /*DEBUG*/

  //divide by the number of items in the moving average
  Xholder = Xholder / MOVINGAVGSIZE;
  if (Xholder < ZEROTHRES && Xholder > -ZEROTHRES) {
    Xholder = 0.0;
  }
  Yholder = Yholder/ MOVINGAVGSIZE;
  if (Yholder < ZEROTHRES && Yholder > -ZEROTHRES) {
    Yholder = 0.0;
  }
  Zholder = Zholder/ MOVINGAVGSIZE;
  if (Zholder < ZEROTHRES && Zholder > -ZEROTHRES) {
    Zholder = 0.0;
  }

  #ifdef DEBUG
    Serial.print("Xholder Divided: ");
    Serial.println(Xholder, DEC);
    Serial.print("Yholder Divided: ");
    Serial.println(Yholder, DEC);
    Serial.print("Zholder Divided: ");
    Serial.println(Zholder, DEC);
  #endif /*DEBUG*/

  movingAvgVect.XAcc = (int8_t)round(Xholder);
  movingAvgVect.YAcc = (int8_t)round(Yholder);
  movingAvgVect.ZAcc = (int8_t)round(Zholder);


  // Serial.println(sensorIndex, DEC);
  // Serial.print("movingAvgVect.XAcc: ");
  // Serial.println(movingAvgVect.XAcc, DEC);
  // Serial.print("movingAvgVect.YAcc: ");
  // Serial.println(movingAvgVect.YAcc, DEC);
  // Serial.print("movingAvgVect.ZAcc: ");
  // Serial.println(movingAvgVect.ZAcc, DEC);
  
  #ifdef DEBUG
    Serial.println(sensorIndex, DEC);
    Serial.print("movingAvgVect.XAcc: ");
    Serial.println(movingAvgVect.XAcc, DEC);
    Serial.print("movingAvgVect.YAcc: ");
    Serial.println(movingAvgVect.YAcc, DEC);
    Serial.print("movingAvgVect.ZAcc: ");
    Serial.println(movingAvgVect.ZAcc, DEC);
  #endif /*DEBUG*/


  return movingAvgVect;
}

/********************************************
 * newNetConnect(uint8_t rxStr[50])
*********************************************/
uint8_t newNetConnect(uint8_t rxStr[50]) {
    
    Serial.println("newNetConnect()");
    #ifdef DEBUG
    Serial.println("newNetConnect()");
    #endif /*DEBUG*/

    uint8_t gotSSID = 0;
    uint8_t gotPSWD = 0;
    uint8_t SSIDLength = 0;
    uint8_t PSWDLength = 0;
    char tmpSSID[50];
    char tmpPSWD[50];
    for (int z = 0; z < 50; z++) {
      if (gotSSID = 0) {
        //Need better checking here...
        if (rxStr[z] != '_') {
        tmpSSID[SSIDLength] = rxStr[z];
        SSIDLength++;
        //"__--__"
        } else if (rxStr[z+1] != '_' && rxStr[z+2] == '_' && rxStr[z+2] == '-' && rxStr[z+2] == '-' && rxStr[z+2] == '_' && rxStr[z+2] == '_') {
          gotSSID = 1;
        }
      } else if (gotPSWD == 0) {
        if (rxStr[z] != '_') {
        tmpPSWD[PSWDLength] = rxStr[z];
        PSWDLength++;
        } else if (rxStr[z+1] != '_' && rxStr[z+2] == '_' && rxStr[z+2] == '-' && rxStr[z+2] == '-' && rxStr[z+2] == '_' && rxStr[z+2] == '_') {
          gotPSWD = 1;
          break;
        }
      }
    }
      char newSSID[SSIDLength + 1];
      char newPSWD[PSWDLength + 1];
      Serial.print("newSSID: ");
      for (int i = 0; i < SSIDLength; i++) {
        newSSID[i] = tmpSSID[i];
        Serial.print(newSSID[i]);
        Serial.println("");
        //Serial.print(newSSID[i], CHAR);
      }

      Serial.print("newPSWD: ");
      for (int j = 0; j < SSIDLength; j++) {
        newPSWD[j] = tmpPSWD[j];
        Serial.print(newPSWD[j]);
        Serial.println("");
      }

      Serial.println("Got new connection information. Reconnecting...");
      if (connectWiFi(1, newSSID, newPSWD)) {
        Serial.println("Connection Successful");
        return 1;
      } else {
        Serial.println("Connection Failed");
        return -1;
      }
    }

/********************************************
 * connectWiFi(uint8_t mode, char ssid[], char pswd[])
*********************************************/
uint8_t connectWiFi(uint8_t mode, char ssid[], char pswd[]) {
   Serial.println("connectWiFi()");
    #ifdef DEBUG
    Serial.println("connectWiFi()");
   #endif /*DEBUG*/

  char ip[] = "0.0.0.0";

  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
    Serial.println("Disconnected");
  }  

  if (mode == 0) {   //Ap mode (start up)
      WiFi.mode(WIFI_AP);
      Serial.println("Creating AP network");
      WiFi.softAP(ssid, pswd); 
      Serial.print("Connected: ");
      Serial.println(WiFi.softAPIP());
      //char ip[] = WiFi.softAPIP();
  } else if (mode == 1) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pswd);
  
    uint8_t wifiAttempts = 0;
    while(WiFi.status() != WL_CONNECTED)  {
      WiFi.begin(ssid, pswd);
      delay(1000);
      Serial.println("Connecting to WiFi...");
      wifiAttempts++; 
      //Serial.println(wifiAttempts, DEC);
        //Reset ESP32 after 12 failed connection attempts
          if (wifiAttempts > 5) {
              Serial.println("Unable to connect. Switching to AP mode");
              if (connectWiFi(0, APssid, APpassword)) {
                return -1;
                // Serial.println("Restarting");   //Can't restart because we will loss connection info from client
                // ESP.restart();
              }
            }
      }
  }

  Serial.println("Connected to network");
  tftWriteNetwork(ssid, mode);
  return 1;
}

/********************************************
 * tftSetup()
*********************************************/
void tftSetup() {
  Serial.println("tftSetup()");
    #ifdef DEBUG
    Serial.println("tftSetup()");
   #endif /*DEBUG*/
  tft.init();
  tft.fillScreen(0xFFFF);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setCursor(30,15,1);     //(Left, Top, font)
  tft.setTextSize(2);
  //tft.setTextFont(1);
  tft.println("The Conductor");
  tft.setCursor(30,30,1);
  tft.println("-------------");
}

/********************************************
 * tftWriteNetwork(char ssid[])
*********************************************/
void tftWriteNetwork(char ssid[], uint8_t mode) {
  Serial.println("tftWriteNetwork()");
    #ifdef DEBUG
    Serial.println("tftWriteNetwork()");
   #endif /*DEBUG*/
  //Write the network connection data to the TFT
  tft.setCursor(30,50,1);
  tft.println(ssid);
  tft.setCursor(30,75,1);
  if (mode == 0) {  //AP Network
    tft.println(WiFi.softAPIP());
  } else {
    tft.println(WiFi.localIP());
  }
  Serial.println("TFT written");
}
/*
MXC4005XC-B Accelerometer I2C requirements:
The first byte transmitted by the master following a START is used to address the slave device. The first 7 bits
contain the address of the slave device, and the 8th bit is the R/W* bit (read = 1, write = 0; the asterisk indicates
active low, and is used instead of a bar). If the transmitted address matches up to that of the MXC400xXC, then the
MXC400xXC will acknowledge receipt of the address, and prepare to receive or send data.

If the master is writing to the MXC400xXC, then the next byte that the MXC400xXC receives, following the address
byte, is loaded into the address counter internal to the MXC400xXC. The contents of the address counter indicate
which register on the MXC400xXC is being accessed. If the master now wants to write data to the MXC400xXC, it
just continues to send 8-bit bytes. Each byte of data is latched into the register on the MXC400xXC that the address
counter points to. The address counter is incremented after the transmission of each byte.

If the master wants to read data from the MXC400xXC, it first needs to write the address of the register it wants to
begin reading data from to the MXC400xXC address counter. It does this by generating a START, followed by the
address byte containing the MXC400xXC address, with R/W* = 0. The next transmitted byte is then loaded into the
MXC400xXC address counter. Then, the master repeats the START condition and re-transmits the MXC400xXC
address, but this time with the R/W* bit set to 1. During the next transmission period, a byte of data from the
MXC400xXC register that is addressed by the contents of the address counter will be transmitted from the
MXC400xXC to the master. As in the case of the master writing to the MXC400xXC, the contents of the address
counter will be incremented after the transmission of each byte. 

I2C Address (7bit):
5 - 15H (0x0F)?

Addresses Register:
0x03 XOUT upper [0-7]
0x04 XOUT lower [4-7]

0x05 YOUT upper [0-7]
0x06 YOUT lower [4-7]

0x07 ZOUT upper [0-7]
0x08 ZOUT lower [4-7]

To do: debug  I2C
Get Orientation
Data design
*/