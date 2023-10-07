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

uint8_t testCount = 0;

// // Create AsyncWebServer object on port 80
AsyncWebServer OTAserver(80);

// //Create time of flight sensor object
// Adafruit_VL53L0X toF = Adafruit_VL53L0X();

// // //Structure to hold ToF sensor data
// VL53L0X_RangingMeasurementData_t measure;

// //Init Display
// TFT_eSPI tft = TFT_eSPI();

// //Globals
// uint8_t I2CPort = 0;
// uint8_t vecCount = 0;  //Count number of samples taken for timing tests
// //uint8_t dataCount = 0;
// uint8_t dist = -1;       //For collecting distance data from VL53L0X
// uint8_t toFReady = 0;    //Set to one when the toF is ready to measure; 0 when measuring or disabled
// uint8_t toFLoopCount = 0; //Counter for number of loops between each ToF reading

// //int16_t Acc1Avg[3];   //XYZ vector

const char* ssid = NETWORK;
const char* password = PASS;
WiFiServer wifiServer(80);
WiFiClient client;
int16_t socketTestData = 4040;

// char bytes[SOCKPACKSIZE];
// accVector accVecArray[NUMSENSORS][MOVINGAVGSIZE]; //array of vector arrays 
// //accVector Acc1Vectors[accPacketSize];
// uint8_t sampleCount = 0;    //Counts number of samples for the moving average filter
// uint8_t txCount = 0;

// //Timer stuff
// hw_timer_t * timer1 = NULL;

// //Measurement globals - can remove from production
// uint32_t AccVecStart;
// uint32_t AccVecStartMicro;
// uint32_t AccPacketStart;
// uint32_t AccPacketStartMicro;
// uint32_t AccVectorEnd = 0;
// uint32_t AccVectorEndMicro = 0;
// uint32_t AccPacketEnd;
// uint32_t AccPacketEndMicro;


// /************************
//  * setup()
// *************************/
void setup() {

  Serial.begin(115200);
  Serial.println("I am alive!");
//    #ifdef DEBUG
//     Serial.println("I am alive!");
//   #endif /*DEBUG*/
  
//   Wire.begin(I2C_SDA, I2C_SCL);

//   while (!toF.begin(0x29,false)) {
//     Serial.println("Failed to boot VL53L0X ToF sensor... restarting");
//     ESP.restart();
//   }
//   //toF.setInterruptThresholds();

//   // Enable Continous Measurement Mode
//   //Serial.println("Set Mode VL53L0X_DEVICEMODE_CONTINUOUS_RANGING... ");
//   toF.setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, false);
//   toF.configSensor(toF.VL53L0X_SENSE_LONG_RANGE);  //Set to long range
//   toFReady = 1;    //Set to one when the toF is ready to measure; 0 when measuring or disabled


  AsyncElegantOTA.begin(&OTAserver);    // Start ElegantOTA

  WiFi.begin(ssid, password);
  uint8_t wifiAttempts = 0;
  while(WiFi.status() != WL_CONNECTED)  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    wifiAttempts++;
    //Serial.println(wifiAttempts, DEC);
      //Reset ESP32 after 12 failed connection attempts
        if (wifiAttempts > 5) {
        Serial.print("Restarting");
        ESP.restart();
      }
    }
  Serial.println("Connected to the WiFi Network");
  Serial.println(WiFi.localIP());
  wifiServer.begin();
  Serial.println("Started Wifi server");
  OTAserver.begin();  //server for OTA
  Serial.println("Started OTA server");

//   //Start the timer
//   timer1 = timerBegin(0, 10, true);
//   timerStart(timer1);

//   tft.init();
//   // uint16_t CursorX = tft.getCursorX();
//   // uint16_t CursorY = tft.getCursorY();
//   // Serial.print("TFT CursorX: ");
//   // Serial.println(CursorX, DEC);
//   // Serial.print("TFT CursorY: ");
//   // Serial.println(CursorY, DEC);
  
//   // CursorX = tft.getCursorX();
//   // CursorY = tft.getCursorY();
//   // Serial.print("TFT CursorX: ");
//   // Serial.println(CursorX, DEC);
//   // Serial.print("TFT CursorY: ");
//   // Serial.println(CursorY, DEC);
//   tft.fillScreen(0xFFFF);
//   tft.setTextColor(TFT_BLACK, TFT_WHITE);
//   tft.setCursor(30,15,1);     //(Left, Top, font)
//   tft.setTextSize(2);
//   //tft.setTextFont(1);
//   tft.println("The Conductor");
//   tft.setCursor(30,30,1);
//   tft.println("-------------");

//   // tft.setTextSize(2);
//   // tft.setCursor(5,50,1);
//   // tft.print("IP: ");
//   tft.setCursor(30,50,1);
//   tft.println(WiFi.SSID());
  

//   // tft.setCursor(2,75,1);
//   // tft.print("SSID: ");
//   tft.setCursor(30,75,1);
//   tft.println(WiFi.localIP());
//   // CursorX = tft.getCursorX();
//   // CursorY = tft.getCursorY();
//   // Serial.print("TFT CursorX: ");
//   // Serial.println(CursorX, DEC);
//   // Serial.print("TFT CursorY: ");
//   // Serial.println(CursorY, DEC);
//   Serial.println("TFT written");

//   // #ifdef DEBUG
//   //   Serial.print("Core: ");
//   //   Serial.println(xPortGetCoreID());
//   // #endif /*DEBUG*/
}

// /************************
//  * loop()
// *************************/
void loop() {

    // if (testCount < 254){
    //   testCount++;
    // }
    // else {
    //   Serial.println("Help me!");
    //   testCount = 0;
    // }

//   if (vecCount == 0) {
//       AccPacketStartMicro = timerReadMicros(timer1);
//   }

//   client = wifiServer.available();
 
//   if (client) {
//     while (client.connected()) {
//       while (client.available() > 0) {
//         uint8_t byteCode = client.read();

//         Serial.print("byteCode: ");
//         Serial.println(byteCode, HEX);

//         #ifdef DEBUG
//           Serial.print("byteCode: ");
//           Serial.println(byteCode, HEX);
//         #endif /*DEBUG*/

//         if (byteCode == 0xFF || byteCode == 0x0F) {  //0xFF is normal case, 0x0F is normal case plus distance
//           uint8_t dist = -1;         //Distance measurement in mm
//           //Send Acc data only
//           #ifdef DEBUG
//             Serial.println("Start Acc data packet");
//           #endif /*DEBUG*/

//           //accVector Acc1Vector;

//           //Measuring Time
//           //AccVecStart = timerRead(timer1);
//           #ifdef DEBUG
//             Serial.print("AccVecStart: ");
//             Serial.println(AccVecStart);
//           #endif /*DEBUG*/
//           AccVecStartMicro = timerReadMicros(timer1);
          
//           //Get data
//           while (sampleCount < MOVINGAVGSIZE) {
//             uint32_t getDataStart = timerReadMicros(timer1);
//             for (uint8_t i = 0; i < NUMSENSORS; i++) {
//               // Serial.print("Sensor: ");
//               // Serial.println(i, DEC);
//               accVecArray[i][sampleCount] = getAccAxes(i+1);  //Gets data from the accelerometer on I2C port 1 (SCL0 /SDA0)
//               // accVecArray[1][sampleCount] = getAccAxes(2);  //Gets data from the accelerometer on I2C port 2 (SCL1 /SDA1)
//               // accVecArray[2][sampleCount] = getAccAxes(1);  //Gets data from the accelerometer on I2C port 1 (SCL0 /SDA0)
//               // accVecArray[3][sampleCount] = getAccAxes(2);  //Gets data from the accelerometer on I2C port 2 (SCL1 /SDA1)
//             }

//             uint32_t getDataEnd = timerReadMicros(timer1);
//             Serial.print("Sample Time Micros: ");
//             Serial.println(getDataEnd - getDataStart);
//             sampleCount++;
//           }

//           uint32_t MvgAvgStart = timerReadMicros(timer1);
//           if (sampleCount == MOVINGAVGSIZE) {        //After moving average size of samples (3) filter
//             accVector AccVectorMAVG[NUMSENSORS];
//             for (int i = 0; i < NUMSENSORS; i++) {   //One vector per sensor
//               //vectortoBytes(accVecArray[i][0], i);  //Puts data into byte format for socket TX
//               AccVectorMAVG[i] = movingAvg(i);     
//               vectortoBytes(AccVectorMAVG[i], i);  //Puts data into byte format for socket TX
//             }
//             uint32_t MvgAvgEnd = timerReadMicros(timer1);
//             Serial.print("Moving Avg Time Micros: ");
//             Serial.println(MvgAvgEnd - MvgAvgStart);

//             if (byteCode == 0x0F) {
//               uint32_t getDistStart = timerReadMicros(timer1);
          
//               Serial.print("Get distance");
//               Serial.println();

//               //Structure to hold ToF sensor data
//               //VL53L0X_RangingMeasurementData_t measure;
//               if (toFReady) {
//                 toF.startMeasurement();
//                 toFReady = 0;
//                 toFLoopCount = 0;
//               }
//               else {
//                 toFLoopCount++;
//               }  

//               uint8_t distReady = digitalRead(TOFINTPIN);
//               if (distReady == LOW) {
//                 toF.getRangingMeasurement(&measure, false);
//                 //toF.getVcselPulsePeriod();
//                 // toF.setMeasurementTimingBudgetMicroSeconds();
//                 //toF.configSensor();
//                 // toF
//                 //toF.getSingleRangingMeasurement(&measure, true);
//                 //toF.setGpioConfig();

//                 uint16_t dist16 = measure.RangeMilliMeter;

//                 if (measure.RangeStatus == 0 || measure.RangeStatus == 2) {
//                   Serial.println("Range Valid");
//                   Serial.println("****************************************");
//                   Serial.println("raw distance: ");
//                   Serial.println(dist16, DEC);
//                   Serial.println(dist16, DEC);
//                   Serial.println(dist16, DEC);
//                   Serial.println(dist16, DEC);
//                   Serial.println(dist16, DEC);
//                   Serial.println("****************************************");

//                   dist = (uint8_t) ((dist16) >> 2);   //Divide by 8 to get range of 0 - 2000mm in 8 bits

//                   Serial.print("Scaled distance: ");
//                   Serial.println(dist, HEX);
                
//                   Serial.print("distance Deximal: ");
//                   Serial.println(dist, DEC);

//                   Serial.print("samples per distance measurement: ");
//                   Serial.println(toFLoopCount, DEC);

//                 } else {
//                   dist = -1;
                
//                   if (measure.RangeStatus == 1) {
//                     Serial.print("Sigma Fail");
//                   } else if (measure.RangeStatus == 2) {
//                     Serial.print("Signal Fail");
//                   } else if ((measure.RangeStatus == 3)) {
//                     Serial.print("Min Range Fail");
//                   } else if (measure.RangeStatus == 4) {
//                     Serial.print("Phase Fail");
//                   } else if ((measure.RangeStatus == 255)) {
//                     Serial.print("No Data Fail");
//                     ESP.restart();
//                   }
//                 }
//                 toF.clearInterruptMask(false);    //Reset the interrupt for the next measurement
//                 toFReady = 1; 
//               } else {
//                 Serial.println("distReady High");
//               }
              
//               uint32_t getDistEnd = timerReadMicros(timer1);
//               Serial.print("Dist measurement micros: ");
//               Serial.println(getDistEnd - getDistStart);
//             }

//             #ifdef DEBUG
//               Serial.print("accVector.XAcc: ");
//               Serial.println(accVector.XAcc, DEC);
//               Serial.print("accVector.YAcc: ");
//               Serial.println(accVector.YAcc, DEC);
//               Serial.print("accVector.ZAcc: ");
//               Serial.println(accVector.ZAcc, DEC);
//               Serial.print("accVector.XT: ");
//               Serial.println(accVector.XT, DEC);
//               Serial.print("accVector.YT: ");
//               Serial.println(accVector.YT, DEC);
//               Serial.print("accVector.ZT: ");
//               Serial.println(accVector.ZT, DEC);
//             #endif /*DEBUG*/

//           uint32_t TXStart = timerReadMicros(timer1);
//           // if (RXMODE == "byteRx") {
//             // Serial.print("Byte Rx Mode");
//             //Write vector byte array to socket one byte at a time

//             uint8_t bytesSent = 0;
//             for(int i = 0; i < SOCKPACKSIZE; i++) {
//               uint8_t byte = client.write(bytes[i]);
//               bytesSent += byte;

//               Serial.print("Byte  ");
//               Serial.print(i);
//               Serial.print(": ");
//               Serial.println(bytes[i], DEC);

//               #ifdef DEBUG
//                 Serial.print("DEC ");
//                 Serial.print(i);
//                 Serial.print(": ");
//                 Serial.println(bytes[i], DEC);
//                 Serial.print("HEX ");
//                 Serial.print(i);
//                 Serial.print(": ");
//                 Serial.println(bytes[i], HEX);
//               #endif /*DEBUG*/
           
//           }

//           if (byteCode == 0x0F) {
//               Serial.print("Byte code 0x0F send dist ");
//               Serial.print("distance Deximal: ");
//               Serial.println(dist, DEC);

//             if (dist > 0 && dist < 250) {                        //Send the dist data we have if it is in range
//               uint8_t byte = client.write(dist);
//               bytesSent += byte;
              
//             }
//               else { //We have to send something because the client is waiting for it - range is 0-250 so 255 is okay
//               uint8_t byte = client.write(0xFF);
//               bytesSent += byte;
              
//               Serial.print("Byte  ");
//               Serial.print(SOCKPACKSIZE + 1);
//               Serial.print(": ");
//               Serial.println(0xFF, DEC);

//             }
//               Serial.print("Bytes sent: ");
//               Serial.println(bytesSent, DEC);
//           } 
          
//           // } else if (RXMODE == "sampleRx") {
//           //   Serial.print("Sample Rx Mode");
//           //   //Print the whole packet at once
//           //     uint8_t byte = client.print(bytes);

//           //     Serial.print("Bytes sent: ");
//           //     Serial.println(byte, DEC);
//           // }

//             uint32_t TXEnd = timerReadMicros(timer1);
//             Serial.print("Tx Time Micros: ");
//             Serial.println(TXEnd - TXStart);
//             Serial.println();

//             txCount++;
//             sampleCount = 0;
//           }

//               #ifdef DEBUG
//                 Serial.print("socketTestData Sent: ");
//                 Serial.println(socketTestData, HEX);
//               #endif /*DEBUG*/

//               // //Timing Tests 
//               // AccVectorEnd = timerRead(timer1);
//               // AccVectorEndMicro = timerReadMicros(timer1);

//               // uint32_t AccVectorTime = AccVectorEnd - AccVecStart;
//               // uint32_t AccVectorTimeMicro = AccVectorEndMicro - AccVecStartMicro;

//               // Serial.print("AccVectorTime: ");
//               // Serial.println(AccVectorTime);
//               // Serial.print("AccVectorTimeMicro: ");
//               // Serial.println(AccVectorTimeMicro);

//               #ifdef DEBUG
//                 Serial.print("AccVectorTime: ");
//                 Serial.println(AccVectorTime);
//                 Serial.print("AccVectorTimeMicro: ");
//                 Serial.println(AccVectorTimeMicro);
//                 Serial.print("AccVecStartMicro: ");
//                 Serial.println(AccVecStartMicro);
//               #endif /*DEBUG*/

//               #ifdef DEBUG
                
//                 Serial.print("AccVectorEnd: ");
//                 Serial.println(AccVectorEnd);

//                 Serial.print("AccVectorEndMicro: ");
//                 Serial.println(AccVectorEndMicro);
              
//                 Serial.print("AccVectorTime: ");
//                 Serial.println(AccVectorTime);

//                 Serial.print("AccVectorTimeMicro: ");
//                 Serial.println(AccVectorTimeMicro);
//               #endif /*DEBUG*/
//           } 
//         }
//       }
//     //client.stop();
//     // Serial.println("Client disconnected");
//     // Serial.println();

//   }
  
//   if (timerRead(timer1) >= 0x100000000) {   //Full 32 bits = 0x100000000 (~ 9min with 8MHz timer); 24 bits = 0x1000000 (2s with 8MHz timer)
//     uint32_t rollOver = timerRead(timer1);

//       #ifdef DEBUG
//         Serial.print("rollOver: ");
//         Serial.println(rollOver);
//       #endif /*DEBUG*/

//       #ifdef DEBUG
//         uint32_t rollOverMicro = timerReadMicros(timer1);
//         Serial.print("rollOverMicro: ");
//         Serial.println(rollOverMicro);
//       #endif /*DEBUG*/

//     timerRestart(timer1);
//     #ifdef DEBUG
//       Serial.println("TIMER ROLLOVER");
//       Serial.println("TIMER ROLLOVER");
//       Serial.println("TIMER ROLLOVER");
//       Serial.println("TIMER ROLLOVER");
//       Serial.println("TIMER ROLLOVER");
//       Serial.println("TIMER ROLLOVER");
//       #endif /*DEBUG*/
//   }
  
  }
