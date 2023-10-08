#include "Adafruit_VL53L0X.h"

// ----------------------------------------------------------------------------
// Definition of macros
// ----------------------------------------------------------------------------

//#define DEBUG
#define HTTP_PORT 80
#define LOCALADDR 1
#define I2CADDR 0x70  //I2C Address for multiplexor
#define I2C_SDA 8     //I2C pins
#define I2C_SCL 9
#define MXCI2CADDR 0x15   //I2C Address for MXC400 Accelerometer
#define VL53L0XADDR 0x29  //I2C Address for VL53L0X Time of flight sensor
// #define AccPort1 1        //Ports for Accelerometer 1 (for multiplexor)
// #define AccPort2 2
// #define AccPort3 3
// #define AccPort4 4
#define XOUTHI 0x03      //Registers on MXC400 Accelerometer for data output
#define XOUTLO 0x04 
#define YOUTHI 0x05 
#define YOUTLO 0x06 
#define ZOUTHI 0x07 
#define ZOUTLO 0x08 
//#define accPacketSize 500     //Size of a unit of acc samples
#define NUMSENSORS 2       //Number of sensors
#define ACCPACKSIZE 3     //Size in bytes to send a sample from 1 accelerometer
#define SOCKPACKSIZE 6   //Total size of packet set to socket client (ACCPACKSIZE * number of sensors) 
#define MOVINGAVGSIZE 5   //Number samples to include in moving average [12.5ms * 8 = 100ms]
#define ZEROTHRES 18.0     //All sensor values between +- of this value are set to zero
#define RXMODE "byteRx"
#define TOFINTPIN 8 //Interupt pin for VL53L0X ToF sensor

///************************************
//          Data Globals
//*************************************

// extern uint8_t state;
// extern uint8_t debug; 
struct accVector {
    int8_t XAcc;
    int8_t YAcc;
    int8_t ZAcc;
    // uint32_t XT;
    // uint32_t YT;
    // uint32_t ZT;
};

extern hw_timer_t * timer1;
extern uint8_t I2CPort;
extern char bytes[SOCKPACKSIZE];
extern accVector accVecArray[NUMSENSORS][MOVINGAVGSIZE];
extern uint8_t txCount;
extern uint8_t sampleCount;
extern uint8_t dist;
extern uint8_t toFReady;

///************************************
//          I2C Globals
//*************************************
extern accVector getAccAxes(uint8_t Port);
extern int16_t readAccReg(uint8_t Port, uint8_t r);
extern void changeI2CPort(uint8_t I2CPort);
extern int16_t getAxisAcc(int16_t axisHi, int16_t axisLo);
extern void vectortoBytes(accVector vector, uint8_t sensorIndex);
extern accVector movingAvg(uint8_t vecIndex);
extern uint8_t getDist(Adafruit_VL53L0X toF);


//**********************************
//           WiFI Server Globals
//**********************************
//extern AsyncWebServer server;
//extern AsyncWebSocket ws;

//extern uint8_t socketRXArr[24];
extern uint8_t socketDataIn;