#ifndef _CONFIG_H
#define _CONFIG_H
/*
 * 
 * This is where you should do all the configuration
 * 
 * First you need to make sure that you have all the require Arduino libraries to compile it
 * 
 * required libraries:
 * Adafruit_BMP085 or BMP280 or BMP085_stm32
 * avdweb_VirtualDelay
 */



/////////////// config changes start here ///////////


// choose the pressure sensor that you are using
// for most board the pressure sensor is either BMP085 or BMP180 
// note that BMP085 and 180 are compatible no need to use the new BMP180 library
#define BMP085_180

// if you have a custom ATMega 328 board using a BMP280 pressure sensor 
//#define BMP280

// If you want to have additionnal debugging uncomment it
//#define SERIAL_DEBUG
#undef SERIAL_DEBUG


////////////// config changes end here /////////////
//////////// do not change anything after unless you know what you are doing /////////////////////

#define MAJOR_VERSION 1
#define MINOR_VERSION 1
#define CONFIG_START 32




#define BOARD_FIRMWARE "AltiServo"

#define SerialCom Serial




#include "Arduino.h"
//used for writing in the microcontroler internal eeprom
#include <EEPROM.h>

#include "avdweb_VirtualDelay.h"

//pyro out 1
extern const int pyroOut1;
extern int pinApogee;
//pyro out 2
extern const int pyroOut2;
extern int pinMain;
//pyro out 3
extern const int pyroOut3;
extern int pinOut3;

//pyro out 4
extern const int pyroOut4;
extern int pinOut4;


extern int pinOut2;
extern int pinOut1;

//extern int continuityPins[4];

struct ConfigStruct {
  int unit;             //0 = meter 1 = feet
  int beepingMode;      // decide which way you want to report the altitude
  int outPut1;          // assign a function to each pyro
  int outPut2;
  int outPut3;
  int mainAltitude;     //deployment altitude for the main chute
  int superSonicYesNo;  // if set to yes do not do any altitude measurement when altimeter starts
  int outPut1Delay;      // delay output by x ms
  int outPut2Delay;
  int outPut3Delay;
  int beepingFrequency;  // this beeping frequency can be changed
  int nbrOfMeasuresForApogee; //how many measure to decide that apogee has been reached
  int endRecordAltitude;  // stop recording when landing define under which altitude we are not recording
  int recordTemperature;  //decide if we want to record temperature
  int superSonicDelay;   //nbr of ms during when we ignore any altitude measurements
  long connectionSpeed;   //altimeter connection baudrate
  int altimeterResolution; // BMP sensor resolution
  int eepromSize; //Size of the eeprom used
  int noContinuity;
  int outPut4;
  int outPut4Delay;
  int liftOffAltitude; //Lift Altitude in meters
  int batteryType; // 0= Unknown, 1= "2S (7.4 Volts)", 2 = "9 Volts",3 = "3S (11.1 Volts)
  int servo1OnPos;
  int servo2OnPos;
  int servo3OnPos;
  int servo4OnPos;
  int servo1OffPos;
  int servo2OffPos;
  int servo3OffPos;
  int servo4OffPos;
  
  int cksum;  
};
extern ConfigStruct config;

extern void defaultConfig();
extern boolean readAltiConfig();
extern int getOutPin(int );
extern void writeAltiConfig( char * );
extern void printAltiConfig();
extern void writeConfigStruc();
extern bool CheckValideBaudRate(long);
extern unsigned int CheckSumConf( ConfigStruct );
#endif
