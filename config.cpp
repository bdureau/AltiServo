#include "config.h"

//pyro out 1
const int pyroOut1 = 9;
//pyro out 2
const int pyroOut2 = 10;
//pyro out 3
const int pyroOut3 = 5;
//pyro out 4
const int pyroOut4 = 6;

ConfigStruct config;
//================================================================
// read and write in the microcontroler eeprom
//================================================================
void defaultConfig()
{
  config.unit = 0;
  config.beepingMode = 0;
  config.outPut1 = 0;
  config.outPut2 = 1;
  config.outPut3 = 3;
  config.outPut1Delay = 0;
  config.outPut2Delay = 0;
  config.outPut3Delay = 0;
  config.mainAltitude = 50;
  config.superSonicYesNo = 0;
  config.beepingFrequency = 440;
  //config.separationVelocity = 10;
  config.nbrOfMeasuresForApogee = 5;
  config.endRecordAltitude = 3; // stop recording when landing define under which altitude we are not recording
  config.recordTemperature = 0; //decide if we want to record temperature
  config.superSonicDelay = 0;
  config.connectionSpeed = 38400;
  config.altimeterResolution = 0; //0 to 4 ie: from low resolution to high
  config.eepromSize = 512;
  config.noContinuity = 0;
  config.outPut4 = 3;
  config.outPut4Delay = 0;
  config.liftOffAltitude = 10;
  config.batteryType = 0; // 0= Unknown, 1= "2S (7.4 Volts)", 2 = "9 Volts",3 = "3S (11.1 Volts)
  config.recordingTimeout = 120;
  config.reserved1 = 0;
  config.reserved2 = 0;
  config.servoOnOff = 0;
  config.servo1OnPos = 60;
  config.servo2OnPos = 60;
  config.servo3OnPos = 60;
  config.servo4OnPos = 60;
  config.servo1OffPos = 20;
  config.servo2OffPos = 20;
  config.servo3OffPos = 20;
  config.servo4OffPos = 20;
  config.servoStayOn = 1;
  config.cksum = CheckSumConf(config);
}
boolean readAltiConfig() {
  //set the config to default values so that if any have not been configured we can use the default ones
  defaultConfig();
  int i;
  for ( i = 0; i < sizeof(config); i++ ) {
    *((char*)&config + i) = EEPROM.read(CONFIG_START + i);
  }

  if ( config.cksum != CheckSumConf(config) ) {
    return false;
  }
  return true;
}


/*
  write the config received by the console

*/


bool writeAltiConfigV2( char *p ) {

  char *str;
  int i = 0;
  int command = 0;
  long commandVal = 0;
  int strChk = 0;
  char msg[100] = "";

  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {
    //SerialCom.println(str);
    if (i == 1) {
      command = atoi(str);
      strcat(msg, str);
    }
    if (i == 2) {
      commandVal =  atol(str);
      strcat(msg, str);
    }
    if (i == 3) {
      strChk  =  atoi(str);
    }
    i++;

  }
  //we have a partial config
  if (i < 4)
    return false;
  //checksum is ivalid ?
  if (msgChk(msg, sizeof(msg)) != strChk)
    return false;

  switch (command)
  {
    case 1:
      config.unit = (int) commandVal;
      break;
    case 2:
      config.beepingMode = (int)commandVal;
      break;
    case 3:
      config.outPut1 = (int) commandVal;
      break;
    case 4:
      config.outPut2 = (int) commandVal;
      break;
    case 5:
      config.outPut3 = (int) commandVal;
      break;
    case 6:
      config.mainAltitude = (int) commandVal;
      break;
    case 7:
      config.superSonicYesNo = (int) commandVal;
      break;
    case 8:
      config.outPut1Delay = (int) commandVal;
      break;
    case 9:
      config.outPut2Delay = (int)commandVal;
      break;
    case 10:
      config.outPut3Delay = (int) commandVal;
      break;
    case 11:
      config.beepingFrequency = (int) commandVal;
      break;
    case 12:
      config.nbrOfMeasuresForApogee = (int) commandVal;
      break;
    case 13:
      config.endRecordAltitude = (int) commandVal;
      break;
    case 14:
      config.recordTemperature = (int) commandVal;
      break;
    case 15:
      config.superSonicDelay = (int) commandVal;
      break;
    case 16:
      config.connectionSpeed = commandVal;
      break;
    case 17:
      config.altimeterResolution = (int) commandVal;
      break;
    case 18:
      config.eepromSize = (int) commandVal;
      break;
    case 19:
      config.noContinuity = (int) commandVal;
      break;
    case 20:
      config.outPut4 = (int) commandVal;
      break;
    case 21:
      config.outPut4Delay = (int) commandVal;
      break;
    case 22:
      config.liftOffAltitude = (int) commandVal;
      break;
    case 23:
      config.batteryType = (int) commandVal;
      break;
    case 24:
      config.recordingTimeout = (int) commandVal;
      break;
    case 25:
      config.reserved1 = (int) commandVal;
      break;
    case 26:
      config.reserved2 = (int) commandVal;
      break;
    case 27:
      config.servoOnOff = (int) commandVal;
      break;
    case 28:
      config.servo1OnPos = (int) commandVal;
      break;
    case 29:
      config.servo2OnPos = (int) commandVal;
      break;
    case 30:
      config.servo3OnPos = (int) commandVal;
      break;
    case 31:
      config.servo4OnPos = (int) commandVal;
      break;
    case 32:
      config.servo1OffPos = (int) commandVal;
      break;
    case 33:
      config.servo2OffPos = (int) commandVal;
      break;
    case 34:
      config.servo3OffPos = (int) commandVal;
      break;
    case 35:
      config.servo4OffPos = (int) commandVal;
      break;
    case 36:
      config.servoStayOn = (int) commandVal;
      break;
  }

  // add checksum
  config.cksum = CheckSumConf(config);

  return true;
}
//////////////////////////////////////////////////////////////////////////////////////
void writeConfigStruc()
{
  int i;
  for ( i = 0; i < sizeof(config); i++ ) {
    EEPROM.write(CONFIG_START + i, *((char*)&config + i));
  }
}

void printAltiConfig()
{
  char altiConfig[150] = "";
  char temp[10] = "";
  bool ret = readAltiConfig();
  if (!ret)
    SerialCom.print(F("invalid conf"));

  strcat(altiConfig, "alticonfig,");
  //Unit
  sprintf(temp, "%i,", config.unit);
  strcat(altiConfig, temp);
  //beepingMode
  sprintf(temp, "%i,", config.beepingMode);
  strcat(altiConfig, temp);
  //output1
  sprintf(temp, "%i,", config.outPut1);
  strcat(altiConfig, temp);
  //output2
  sprintf(temp, "%i,", config.outPut2);
  strcat(altiConfig, temp);
  //output3
  sprintf(temp, "%i,", config.outPut3);
  strcat(altiConfig, temp);
  //supersonicYesNo
  sprintf(temp, "%i,", config.superSonicYesNo);
  strcat(altiConfig, temp);
  //mainAltitude
  sprintf(temp, "%i,", config.mainAltitude);
  strcat(altiConfig, temp);
  //AltimeterName
  strcat(altiConfig, BOARD_FIRMWARE);
  strcat(altiConfig, ",");
  //alti major version
  sprintf(temp, "%i,", MAJOR_VERSION);
  strcat(altiConfig, temp);
  //alti minor version
  sprintf(temp, "%i,", MINOR_VERSION);
  strcat(altiConfig, temp);
  //output1 delay
  sprintf(temp, "%i,", config.outPut1Delay);
  strcat(altiConfig, temp);
  //output2 delay
  sprintf(temp, "%i,", config.outPut2Delay);
  strcat(altiConfig, temp);
  //output3 delay
  sprintf(temp, "%i,", config.outPut3Delay);
  strcat(altiConfig, temp);
  //Beeping frequency
  sprintf(temp, "%i,", config.beepingFrequency);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.nbrOfMeasuresForApogee);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.endRecordAltitude);
  strcat(altiConfig, temp);
  //unused but keep it for compatibility with the other alti
  sprintf(temp, "%i,", config.recordTemperature);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.superSonicDelay);
  strcat(altiConfig, temp);
  sprintf(temp, "%ld,", config.connectionSpeed);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.altimeterResolution);
  strcat(altiConfig, temp);
  //unused but keep it for compatibility with the other alti
  sprintf(temp, "%i,", config.eepromSize);
  strcat(altiConfig, temp);
  //unused but keep it for compatibility with the other alti
  sprintf(temp, "%i,", config.noContinuity);
  strcat(altiConfig, temp);
  //output4
  sprintf(temp, "%i,", config.outPut4);
  strcat(altiConfig, temp);
  //output4 delay
  sprintf(temp, "%i,", config.outPut4Delay);
  strcat(altiConfig, temp);
  //Lift off altitude
  sprintf(temp, "%i,", config.liftOffAltitude);
  strcat(altiConfig, temp);
  //Battery type
  sprintf(temp, "%i,", config.batteryType);
  strcat(altiConfig, temp);
  //Recording timeout (not used)
  sprintf(temp, "%i,", config.recordingTimeout);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.reserved1);// for future use
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.reserved2);// for future use
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.servoOnOff);// use servo has a switch
  strcat(altiConfig, temp);
  //servo1OnPos
  sprintf(temp, "%i,", config.servo1OnPos);
  strcat(altiConfig, temp);
  //servo2OnPos
  sprintf(temp, "%i,", config.servo2OnPos);
  strcat(altiConfig, temp);
  //servo3OnPos
  sprintf(temp, "%i,", config.servo3OnPos);
  strcat(altiConfig, temp);
  //servo4OnPos
  sprintf(temp, "%i,", config.servo4OnPos);
  strcat(altiConfig, temp);
  //servo1OffPos
  sprintf(temp, "%i,", config.servo1OffPos);
  strcat(altiConfig, temp);
  //servo2OffPos
  sprintf(temp, "%i,", config.servo2OffPos);
  strcat(altiConfig, temp);
  //servo3OffPos
  sprintf(temp, "%i,", config.servo3OffPos);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.servo4OffPos);
  strcat(altiConfig, temp);
  //leave the servo on after it has been triggered
  sprintf(temp, "%i,", config.servoStayOn);
  strcat(altiConfig, temp);
  unsigned int chk = 0;
  chk = msgChk( altiConfig, sizeof(altiConfig) );
  sprintf(temp, "%i;\n", chk);
  strcat(altiConfig, temp);

  SerialCom.print("$");
  SerialCom.print(altiConfig);
}
bool CheckValideBaudRate(long baudRate)
{
  bool valid = false;
  if (baudRate == 300 ||
      baudRate == 1200 ||
      baudRate == 2400 ||
      baudRate == 4800 ||
      baudRate == 9600 ||
      baudRate == 14400 ||
      baudRate == 19200 ||
      baudRate == 28800 ||
      baudRate == 38400 ||
      baudRate == 57600 ||
      baudRate == 115200 ||
      baudRate == 230400)
    valid = true;
  return valid;
}


/*
    Calculate the checksum for the structure

*/
unsigned int CheckSumConf( ConfigStruct cnf)
{
  int i;
  unsigned int chk = 0;

  for (i = 0; i < (sizeof(cnf) - sizeof(int)); i++)
    chk += *((char*)&cnf + i);

  return chk;
}
/*


*/
unsigned int msgChk( char * buffer, long length ) {

  long index;
  unsigned int checksum;

  for ( index = 0L, checksum = 0; index < length; checksum += (unsigned int) buffer[index++] );
  return (unsigned int) ( checksum % 256 );

}
