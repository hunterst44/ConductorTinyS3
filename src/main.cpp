/*
main.cpp

Created May 1, 2023 by Joel Legassie
Sensor polling is initiated when the ESP32 receives 0xFF or 0x0F from the client and continues until the client closes the connection
Polls NUMSENSORS of sensors in turn to collect a sample of 3 features per sensor (XYZ Axes)
Each sensor's features take up ACCPACKSIZE bytes (2 bytes per feature)
Collects MOVINGAVGSIZE number of samples and computes a moving average of them to send to the client
Any feature values within ZEROTHRES of 0 are rounded to zero
Sends a packet of SOCKPACKSIZE (ACCPACKSIZE * NUMSENSORS)

If ESP32 receives 0xFF it polls only the accelermoters

If it receives 0x0F it polls accelerometers and VL53l0X ToF sensor
ToF sensor is connected directly to the SDA and SCL I2C pins on the ESP32
ToF GPIO01 is connected to ESP32 Pin 8. It is the interrupt pin

Accelerometer data is prepped for neural network training or prediction
ToF data is scaled to 1 byte and sent straight across to the DAW plugin

Over the air updates - you can upload an firmware.bin file to <ESP32 IP Address>:4040/update
Note ESPAsyncWebServer is required for the elegant OTA library, but is not used for sending sensor data to the client

//hello
*/

#include <Arduino.h>
#include <WiFi.h>
#include "basic.h"
#include <Wire.h>
#include <stdlib.h>
#include "secrets.h"
#include <math.h>
#include <AsyncElegantOTA.h>
#include <ESPAsyncWebServer.h>
#include "Adafruit_VL53L0X.h"
#include <TFT_eSPI.h>
#include <SPI.h>
#include <ESP32Ping.h>


uint8_t testCount = 0;

// // Create AsyncWebServer object on port 80
AsyncWebServer OTAserver(8080);

// //Create time of flight sensor object
Adafruit_VL53L0X toF = Adafruit_VL53L0X();

// // //Structure to hold ToF sensor data
VL53L0X_RangingMeasurementData_t measure;

// //Init Display
TFT_eSPI tft = TFT_eSPI();

// //Globals
uint8_t I2CPort = 0;
uint8_t vecCount = 0;  //Count number of samples taken for timing tests
//uint8_t dataCount = 0;
uint8_t dist = -1;       //For collecting distance data from VL53L0X
uint8_t toFReady = 0;    //Set to one when the toF is ready to measure; 0 when measuring or disabled
uint8_t toFLoopCount = 0; //Counter for number of loops between each ToF reading

// //int16_t Acc1Avg[3];   //XYZ vector

char APssid[] = APNETWORK;
char APpassword[] = APPASS;
WiFiServer wifiServer(80);
WiFiClient client;
int16_t socketTestData = 4040;

uint8_t txIdx = SOCKPACKSIZE;
uint8_t rxIdx = 1; //Size of data expected from client - almost always 1
uint8_t byteCode;
//uint8_t sockRxStrIdx = 0;
char bytes[SOCKPACKSIZE + 1];      //Bytes size will be determined at run time - to accept arbitrary length strings
accVector accVecArray[NUMSENSORS][MOVINGAVGSIZE]; //array of vector arrays 
//accVector Acc1Vectors[accPacketSize];
uint8_t sampleCount = 0;    //Counts number of samples for the moving average filter
uint8_t txCount = 0;

// //Timer stuff
hw_timer_t * timer1 = NULL;

// //Measurement globals - can remove from production
uint32_t AccVecStart;
uint32_t AccVecStartMicro;
uint32_t AccPacketStart;
uint32_t AccPacketStartMicro;
uint32_t AccVectorEnd = 0;
uint32_t AccVectorEndMicro = 0;
uint32_t AccPacketEnd;
uint32_t AccPacketEndMicro;


// /************************
//  * setup()
// *************************/
void setup() {

  Serial.begin(115200);
  Serial.println("I am alive!");
   #ifdef DEBUG
    Serial.println("I am alive!");
  #endif /*DEBUG*/

  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
  }
  
  Wire.begin(I2C_SDA, I2C_SCL);

  while (!toF.begin(0x29,false)) {
    Serial.println("Failed to boot VL53L0X ToF sensor... restarting");
    ESP.restart();
  }
  //toF.setInterruptThresholds();

  // Enable Continous Measurement Mode
  //Serial.println("Set Mode VL53L0X_DEVICEMODE_CONTINUOUS_RANGING... ");
  toF.setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, false);
  toF.configSensor(toF.VL53L0X_SENSE_LONG_RANGE);  //Set to long range
  toFReady = 1;    //Set to one when the toF is ready to measure; 0 when measuring or disabled

  initACC(); //Set up accelerometers

  tftSetup();

  //Wifi stuff
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.setAutoReconnect(true);
  //Check cnt.txt to see if there is a saved network connection
  
  CntInfo cntInfo = getNetworkSpiffs();
  Serial.println(cntInfo.cntMode);
  if (cntInfo.cntMode == 1) {
    //1 means reconnect to this network
    //convert infos to char arrays
    size_t ssidLen = cntInfo.ssid.length();
    char ssidArr[ssidLen-1];
    if (cntInfo.ssid[0] == 'T' && cntInfo.ssid[1] == 'h' && cntInfo.ssid[2] == 'e' && cntInfo.ssid[3] == 'C' && cntInfo.ssid[4] == 'o' && cntInfo.ssid[5] == 'n' && cntInfo.ssid[6] == 'd' && cntInfo.ssid[7] == 'u' && cntInfo.ssid[8] == 'c') {
      Serial.println("softAp enabled");
      cntInfo.cntMode = 0;
    }

    for (uint8_t h; h < ssidLen-1; h++) {
      ssidArr[h] = cntInfo.ssid[h];
      Serial.print(ssidArr[h]);
    } 
    Serial.println(' ');
    //cntInfo.ssid.getBytes(ssidArr, ssidLen);
    size_t pswdLen = cntInfo.pswd.length();
    char pswdArr[pswdLen-1]; 
    for (uint8_t g; g < pswdLen-1; g++) {
      pswdArr[g] = cntInfo.pswd[g];
      Serial.print(pswdArr[g]);
    } 
    Serial.println(' ');
    //cntInfo.ssid.getBytes(pswdArr, pswdLen);
    connectWiFi(cntInfo.cntMode, ssidArr, pswdArr);
  } else {
    //Connect in AP mode
    cntInfo.cntMode = 0;
    // cntInfo.pswd = APPASS;
    // cntInfo.ssid = APNETWORK;
    connectWiFi(0, APNETWORK, APPASS);
  }

// CntInfo cntInfo;
// cntInfo.cntMode = 1;
// cntInfo.ssid = NETWORK;
// cntInfo.pswd = PASS;
// Serial.print("cntInfo.ssid: ");
// Serial.println(cntInfo.ssid);
// Serial.print("cntInfo.pswd: ");
// Serial.println(cntInfo.pswd);
// if (writeNetworkSpiffs(cntInfo) == 1) {
//   Serial.println("Wrote to Spiffs");
// } else {
//   Serial.print("Spiffs write error");
// }

// CntInfo newinfos = getNetworkSpiffs();

  
  //WiFi.begin(NETWORK, PASS);
  // connectWiFi(0, APssid, APpassword);

  AsyncElegantOTA.begin(&OTAserver);    // Start ElegantOTA

  wifiServer.begin();
  Serial.println("Started Wifi server");
  OTAserver.begin();  //server for OTA
  Serial.println("Started OTA server");

  //Test wifi
  // bool ret = Ping.ping("www.google.com", 10);
  // if (ret == true) {
  //   Serial.println("Ping Success!");
  // } else {
  //   Serial.println("ping failure");
  // }

  //Start the timer
  timer1 = timerBegin(0, 10, true);
  timerStart(timer1);

  //Find the I2C ports
  // for (uint8_t i = 0; i < 9; i++) {
  // //accVecArray[0][sampleCount] = getAccAxes(i);
  // readAccReg(i, 3);
  // }
//   // #ifdef DEBUG
//   //   Serial.print("Core: ");
//   //   Serial.println(xPortGetCoreID());
//   // #endif /*DEBUG*/
}

// /************************
//  * loop()
// *************************/
void loop() {

  //char bytes[SOCKPACKSIZE];
  //txIdx = SOCKPACKSIZE; 

  if (vecCount == 0) {
      AccPacketStartMicro = timerReadMicros(timer1);
  }

  client = wifiServer.available();
 
  if (client) {
    while (client.connected()) {
        //Serial.println("Client Connected");
      while (client.available() > 0) {
        //Serial.println("Client Available");

//***************************************************************/
                       //Receive Data
//***************************************************************/
        
        uint8_t rxStr[rxIdx];
        if (rxIdx == 1) {
          byteCode = client.read();
          Serial.println("rxIdx == 1 Should come here everytime...");
        } else {
          Serial.println("rxIdx != 1 Should conly be here when we write text...");
          //uint8_t rxStr[rxIdx];
          
          for (uint8_t k; k < rxIdx; k++) {
            rxStr[k] = client.read();
            // Serial.print("rxStr[");
            // Serial.print(k, DEC);
            // Serial.print("]: ");
            // Serial.println(rxStr[k], HEX);
          }
          // Serial.print("rxStr[0]: ");
          // Serial.println(rxStr[0], HEX);
          byteCode = 0x44;   //Tells the loop to process the network data received
        // uint8_t txIdx = SOCKPACKSIZE; 
        }

        Serial.print("byteCode: ");
        Serial.println(byteCode, HEX);

        #ifdef DEBUG
          Serial.print("byteCode: ");
          Serial.println(byteCode, HEX);
        #endif /*DEBUG*/

      if (byteCode == 0x0F) {
          //0F asks for sensor readings w/ ToF
          txIdx = SOCKPACKSIZE + 1;
          //char bytes[SOCKPACKSIZE + 1];
        } else {
          //All others just send SockPackSize
          txIdx = SOCKPACKSIZE;
          //char bytes[SOCKPACKSIZE];
        }

        //Client wants ACC data
//***************************************************************/
                       //Get Acc Data
//***************************************************************/
        if (byteCode == 0xFF || byteCode == 0x0F) {  //0xFF is normal case, 0x0F is normal case plus distance
            //Set up index and array to receive data                
          uint8_t dist = -1;         //Distance measurement in mm
          //Send Acc data only
          #ifdef DEBUG
            Serial.println("Start Acc data packet");
          #endif /*DEBUG*/

          //accVector Acc1Vector;

          //Measuring Time
          //AccVecStart = timerRead(timer1);
          #ifdef DEBUG
            Serial.print("AccVecStart: ");
            Serial.println(AccVecStart);
          #endif /*DEBUG*/
          AccVecStartMicro = timerReadMicros(timer1);
          
          //Get data
          while (sampleCount < MOVINGAVGSIZE) {
            uint32_t getDataStart = timerReadMicros(timer1);
            for (uint8_t i = 0; i < NUMSENSORS; i++) {
              // Serial.print("Sensor: ");
              // Serial.println(i, DEC);
              uint8_t portNoShift = 0;
              switch (i) {   //I2C Mux ports are not consecutive, so have to do a switch case :(
                case 0:
                  portNoShift = 7;
                  break;
                case 1:
                  portNoShift = 0;
                case 2:
                  portNoShift = 4;
                  break;
                case 3:
                  portNoShift = 5;
                  break;
                default:
                  portNoShift = 7;
              }
              //For breadboard prototype - hit the sensor at port 7 NUMSENSORS times
              //portNoShift = 7;
              accVecArray[i][sampleCount] = getAccAxes(portNoShift); //Use when their is only one sensor. Reads the same sensor over and over
              //USE this line with more than one sensor //accVecArray[i][sampleCount] = getAccAxes(i+1);  //Gets data from the accelerometer on I2C port 1 (SCL0 /SDA0)
              // accVecArray[1][sampleCount] = getAccAxes(2);  //Gets data from the accelerometer on I2C port 2 (SCL1 /SDA1)
              // accVecArray[2][sampleCount] = getAccAxes(1);  //Gets data from the accelerometer on I2C port 1 (SCL0 /SDA0)
              // accVecArray[3][sampleCount] = getAccAxes(2);  //Gets data from the accelerometer on I2C port 2 (SCL1 /SDA1)
            }

            uint32_t getDataEnd = timerReadMicros(timer1);
            Serial.print("Sample Time Micros: ");
            Serial.println(getDataEnd - getDataStart);
            sampleCount++;
          }
          // Serial.print("accVecArray[0][2]: ");
          // Serial.println(accVecArray[0][2].XAcc, DEC);
          // Serial.print("accVecArray[2][4]: ");
          // Serial.println(accVecArray[2][4].YAcc, DEC);
          // Serial.print("accVecArray[0][2]: ");
          // Serial.println(accVecArray[0][2].XAcc, DEC);
          // Serial.print("accVecArray[1][2]: ");
          // Serial.println(accVecArray[1][2].ZAcc, DEC);
//***************************************************************/
                       //Moving Average
//***************************************************************/
          uint32_t MvgAvgStart = timerReadMicros(timer1);
          if (sampleCount == MOVINGAVGSIZE) {        //After moving average size of samples (3) filter
            accVector AccVectorMAVG[NUMSENSORS];
            for (uint8_t b = 0; b < NUMSENSORS; b++) {   //One vector per sensor
              //vectortoBytes(accVecArray[i][0], i);  //Puts data into byte format for socket TX
              //Serial.print("movingAvg sensor: ");
              //Serial.println(i, DEC);
              AccVectorMAVG[b] = movingAvg(b);  
              //sensorIndex = sensorIndex*ACCPACKSIZE;
              
              bytes[0 + (b*ACCPACKSIZE)] = AccVectorMAVG[b].XAcc; //0 + (3*3) = 9 
              bytes[1 + (b*ACCPACKSIZE)] = AccVectorMAVG[b].YAcc;
              bytes[2 + (b*ACCPACKSIZE)] = AccVectorMAVG[b].ZAcc;   
              //vectortoBytes(AccVectorMAVG[i], i);  //Puts data into byte format for socket TX
            }
            uint32_t MvgAvgEnd = timerReadMicros(timer1);
            Serial.print("Moving Avg Time Micros: ");
            Serial.println(MvgAvgEnd - MvgAvgStart);
          }
        }

        //Client also needs data from the ToF
//***************************************************************/
                       //Get Distance
//***************************************************************/
        if (byteCode == 0x0F) {
          Serial.print("Byte code 0x0F send dist ");
          uint32_t getDistStart = timerReadMicros(timer1);
      
          Serial.print("Get distance");
          Serial.println();

          //Structure to hold ToF sensor data
          //VL53L0X_RangingMeasurementData_t measure;
          if (toFReady) {
            toF.startMeasurement();
            toFReady = 0;
            toFLoopCount = 0;
          }
          else {
            toFLoopCount++;
          }  

          uint8_t distReady = digitalRead(TOFINTPIN);
          if (distReady == LOW) {
            toF.getRangingMeasurement(&measure, false);
            //toF.getVcselPulsePeriod();
            // toF.setMeasurementTimingBudgetMicroSeconds();
            //toF.configSensor();
            // toF
            //toF.getSingleRangingMeasurement(&measure, true);
            //toF.setGpioConfig();

            uint16_t dist16 = measure.RangeMilliMeter;

            if (measure.RangeStatus == 0 || measure.RangeStatus == 2) {
              Serial.println("Range Valid");
              Serial.println("****************************************");
              Serial.println("raw distance: ");
              Serial.println(dist16, DEC);
              Serial.println(dist16, DEC);
              Serial.println(dist16, DEC);
              Serial.println(dist16, DEC);
              Serial.println(dist16, DEC);
              Serial.println("****************************************");

              dist = (uint8_t) ((dist16) >> 2);   //Divide by 8 to get range of 0 - 2000mm in 8 bits

              Serial.print("Scaled distance: ");
              Serial.println(dist, HEX);
            
              Serial.print("distance Decimal: ");
              Serial.println(dist, DEC);

              Serial.print("samples per distance measurement: ");
              Serial.println(toFLoopCount, DEC);

            } else {
              dist = -1;
            
              if (measure.RangeStatus == 1) {
                Serial.print("Sigma Fail");
              } else if (measure.RangeStatus == 2) {
                Serial.print("Signal Fail");
              } else if ((measure.RangeStatus == 3)) {
                Serial.print("Min Range Fail");
              } else if (measure.RangeStatus == 4) {
                Serial.print("Phase Fail");
              } else if ((measure.RangeStatus == 255)) {
                Serial.print("No Data Fail");
                //ESP.restart();
              }
            }
            toF.clearInterruptMask(false);    //Reset the interrupt for the next measurement
            toFReady = 1; 
          } else {
            Serial.println("distReady High");
          }
          
          uint32_t getDistEnd = timerReadMicros(timer1);
          Serial.print("Dist measurement micros: ");
          Serial.println(getDistEnd - getDistStart);
          Serial.print("distance Deximal: ");
          Serial.println(dist, DEC);
          if (dist < 0 && dist > 250) {    //If distance is less than zero or greater than 250 it is an error send 0xFF
            dist = 0xFF;
          } 
          bytes[SOCKPACKSIZE] = dist; 
          Serial.print("byteCode: ");
          Serial.println(byteCode, DEC);

        }    //End 0x0F- Get distance 

//***************************************************************/
                //Prepare for Network Data - Send Ack
//***************************************************************/        
        if (byteCode == 0x22) {    //Client is going to send text
              Serial.println("byteCode 0x22: Prepare for text");  //Next message is length of the text (1 byte)
              //Send the secret code to let them know we are ready.
              bytes[0] = 0xFF;
              bytes[1] = 0x0F;
              for (int i = 2; i < txIdx; i++) {
                bytes[i] = 0;
              }
              rxIdx = 50;   // Network info is always 50 characters with data at the front
        }

//***************************************************************/
              //Received Network data - Send Ack
//***************************************************************/ 
        if (byteCode == 0x44) {
          //Received network infos from client
          //TODO 
          //Call function to parse the data out and reconnect
          // Serial.print("rxStr[0]: ");
          // Serial.println(rxStr[0], HEX);
          if (newNetConnect(rxStr)) {
            //Reset Variables 
            rxIdx = 1;
          }
      }


        #ifdef DEBUG
          Serial.print("accVector.XAcc: ");
          Serial.println(accVector.XAcc, DEC);
          Serial.print("accVector.YAcc: ");
          Serial.println(accVector.YAcc, DEC);
          Serial.print("accVector.ZAcc: ");
          Serial.println(accVector.ZAcc, DEC);
          Serial.print("accVector.XT: ");
          Serial.println(accVector.XT, DEC);
          Serial.print("accVector.YT: ");
          Serial.println(accVector.YT, DEC);
          Serial.print("accVector.ZT: ");
          Serial.println(accVector.ZT, DEC);
        #endif /*DEBUG*/

      uint32_t TXStart = timerReadMicros(timer1);
      // if (RXMODE == "byteRx") {
        // Serial.print("Byte Rx Mode");
        //Write vector byte array to socket one byte at a time

        //Get packet size in bytes SOCKPACKSIZE + 0 or 1, or text
        
//***************************************************************/
                       //Send Data
//***************************************************************/
      if ((byteCode == 0xFF || byteCode == 0x0F || byteCode == 0x22)) {   //Don't need to send data after changing network
        uint8_t bytesSent = 0;
        Serial.print("txIdx: ");
        Serial.println(txIdx, DEC);
        for(int i = 0; i < txIdx; i++) {
          uint8_t byte = client.write(bytes[i]);
          bytesSent += byte;

          Serial.print("Byte  ");
          Serial.print(i);
          Serial.print(": ");
          Serial.println(bytes[i], DEC);

          #ifdef DEBUG
            Serial.print("DEC ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(bytes[i], DEC);
            Serial.print("HEX ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(bytes[i], HEX);
          #endif /*DEBUG*/
        
      }
            uint32_t TXEnd = timerReadMicros(timer1);
            Serial.print("Tx Time Micros: ");
            Serial.println(TXEnd - TXStart);
            Serial.println();

            txCount++;
            sampleCount = 0;
      } 

              #ifdef DEBUG
                Serial.print("socketTestData Sent: ");
                Serial.println(socketTestData, HEX);
              #endif /*DEBUG*/

//               // //Timing Tests 
//               // AccVectorEnd = timerRead(timer1);
//               // AccVectorEndMicro = timerReadMicros(timer1);

//               // uint32_t AccVectorTime = AccVectorEnd - AccVecStart;
//               // uint32_t AccVectorTimeMicro = AccVectorEndMicro - AccVecStartMicro;

//               // Serial.print("AccVectorTime: ");
//               // Serial.println(AccVectorTime);
//               // Serial.print("AccVectorTimeMicro: ");
//               // Serial.println(AccVectorTimeMicro);

              #ifdef DEBUG
                Serial.print("AccVectorTime: ");
                Serial.println(AccVectorTime);
                Serial.print("AccVectorTimeMicro: ");
                Serial.println(AccVectorTimeMicro);
                Serial.print("AccVecStartMicro: ");
                Serial.println(AccVecStartMicro);
              #endif /*DEBUG*/

              #ifdef DEBUG
                
                Serial.print("AccVectorEnd: ");
                Serial.println(AccVectorEnd);

                Serial.print("AccVectorEndMicro: ");
                Serial.println(AccVectorEndMicro);
              
                Serial.print("AccVectorTime: ");
                Serial.println(AccVectorTime);

                Serial.print("AccVectorTimeMicro: ");
                Serial.println(AccVectorTimeMicro);
              #endif /*DEBUG*/
          } 
        }
      }
//     //client.stop();
//     // Serial.println("Client disconnected");
//     // Serial.println();  
  if (timerRead(timer1) >= 0x100000000) {   //Full 32 bits = 0x100000000 (~ 9min with 8MHz timer); 24 bits = 0x1000000 (2s with 8MHz timer)
    uint32_t rollOver = timerRead(timer1);

      #ifdef DEBUG
        Serial.print("rollOver: ");
        Serial.println(rollOver);
      #endif /*DEBUG*/

      #ifdef DEBUG
        uint32_t rollOverMicro = timerReadMicros(timer1);
        Serial.print("rollOverMicro: ");
        Serial.println(rollOverMicro);
      #endif /*DEBUG*/

    timerRestart(timer1);
    #ifdef DEBUG
      Serial.println("TIMER ROLLOVER");
      Serial.println("TIMER ROLLOVER");
      Serial.println("TIMER ROLLOVER");
      Serial.println("TIMER ROLLOVER");
      Serial.println("TIMER ROLLOVER");
      Serial.println("TIMER ROLLOVER");
      #endif /*DEBUG*/
  }
  
  }
