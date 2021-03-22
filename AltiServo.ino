/*
  Rocket Servo altimeter ver 1.2
  Copyright Boris du Reau 2012-2021

  The following is board for triggering servo's on event during a rocket flight.

  This is using a BMP085 or BMP180 presure sensor and an Atmega 328

  For the BMP085 or BMP180 pressure sensor
  Connect VCC of the BMP085 sensor to 5.0V! make sure that you are using the 5V sensor (GY-65 model for the BMP085 )
  Connect GND to Ground
  Connect SCL to i2c clock - on 328 Arduino Uno/Duemilanove/etc thats Analog 5
  Connect SDA to i2c data - on 328 Arduino Uno/Duemilanove/etc thats Analog 4
  EOC is not used, it signifies an end of conversion

  For the BMP280


  Major changes on version 1.0
  Altimeter initial version
  Major changes on version 1.1
  code clean up
  adding checksum
  Major changes on version 1.2
  Allow multiple drogue or main
  added landing and lift off events
*/
#include <Servo.h>
//altimeter configuration lib
#include "config.h"
#include <Wire.h> //I2C library


#ifdef BMP085_180
#include <Adafruit_BMP085.h>
#endif

#ifdef BMP280
#include <BMP280.h>
#define P0 1013.25
#endif


#include "kalman.h"
#include "beepfunc.h"


//////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////
int mode = 0; //0 = read; 1 = write;

#ifdef BMP085_180
Adafruit_BMP085 bmp;
#endif
#ifdef BMP280
BMP280 bmp;
#endif

//ground level altitude
long initialAltitude;
long liftoffAltitude;
long lastAltitude;
//current altitude
long currAltitude;
//Apogee altitude
long apogeeAltitude;
long mainAltitude;
boolean liftOff = false;
unsigned long initialTime;

boolean FastReading = false;
boolean mainHasFired = false;
//Our drogue has been ejected i.e: apogee has been detected
boolean apogeeHasFired = false;
boolean landingHasFired = false;
boolean liftOffHasFired = false;

//nbr of measures to do so that we are sure that apogee has been reached
unsigned long measures = 5;
unsigned long mainDeployAltitude;

// pin used by the jumpers

const int pinAltitude1 = 8;
const int pinAltitude2 = 7;

//soft configuration
boolean softConfigValid = false;

//#define NBR_MEASURE_APOGEE 5
float FEET_IN_METER = 1;
boolean canRecord = true;

// to store all event
boolean timerEvent1_enable = false;
boolean timerEvent2_enable = false;
boolean timerEvent3_enable = false;
boolean timerEvent4_enable = false;

boolean apogeeEvent_Enable = false;
boolean mainEvent_Enable = false;
boolean landingEvent_Enable = false;
boolean liftOffEvent_Enable = false;
// enable/disable output
boolean out1Enable = true;
boolean out2Enable = true;
boolean out3Enable = true;
boolean out4Enable = true;

int apogeeDelay = 0;
int mainDelay = 0;
int landingDelay = 0;
int liftOffDelay = 0;
int out1Delay = 0;
int out2Delay = 0;
int out3Delay = 0;
int out4Delay = 0;

boolean Output1Fired = false;
boolean Output2Fired = false;
boolean Output3Fired = false;
boolean Output4Fired = false;

boolean telemetryEnable = false;

boolean apogeeReadyToFire = false;
  boolean mainReadyToFire = false;
  boolean landingReadyToFire = false;
  boolean liftOffReadyToFire = false;
  unsigned long apogeeStartTime = 0;
  unsigned long mainStartTime = 0;
  unsigned long landingStartTime = 0;
  unsigned long liftOffStartTime = 0;
  boolean ignoreAltiMeasure = false;

  boolean Event1Fired = false;
  boolean Event2Fired = false;
  boolean Event3Fired = false;
  boolean Event4Fired = false;

  boolean MainFiredComplete = false;
  boolean LandingFiredComplete = false;
  boolean LiftOffFiredComplete = false;

Servo Servo1;   // First Servo off the chassis
Servo Servo2;   // Second Servo off the chassis
Servo Servo3;
Servo Servo4;

long lastTelemetry = 0;
// main loop
boolean mainLoopEnable = true;


void assignPyroOutputs();
void MainMenu();

#ifdef BMP085_180
double ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude());
}
#endif
#ifdef BMP085_180_STM32
double ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude());
}
#endif
#ifdef BMP280
// Only used by BMP280
double ReadAltitude()
{
  double T, P, A;
  char result = bmp.startMeasurment();
  if (result != 0) {
    delay(result);
    result = bmp.getTemperatureAndPressure(T, P);
    A = KalmanCalc(bmp.altitude(P, P0));
  }
  return A;
}
#endif

/*

   initAlti()

*/
void initAlti() {
  // set main altitude (if in feet convert to metrics)
  if (config.unit == 0)
    FEET_IN_METER = 1;
  else
    FEET_IN_METER = 3.28084 ;

  mainDeployAltitude = int(config.mainAltitude / FEET_IN_METER);
  // beepFrequency
  beepingFrequency = config.beepingFrequency;

 apogeeReadyToFire = false;
  mainReadyToFire = false;
   landingReadyToFire = false;
   liftOffReadyToFire = false;
   apogeeStartTime = 0;
   mainStartTime = 0;
 landingStartTime = 0;
   liftOffStartTime = 0;
   ignoreAltiMeasure = false;

   Event1Fired = false;
   Event2Fired = false;
   Event3Fired = false;
   Event4Fired = false;

   MainFiredComplete = false;
   LandingFiredComplete = false;
   LiftOffFiredComplete = false;
  landingHasFired = false;
  liftOffHasFired = false;
  //our drogue has not been fired
  apogeeHasFired = false;
  mainHasFired = false;


  assignPyroOutputs();

  //SerialCom.print(F("Set outputs\n"));
  //Initialise the output pin
  Servo1.attach(pyroOut1);
  Servo2.attach(pyroOut2);
  Servo3.attach(pyroOut3);
  Servo4.attach(pyroOut4);

  //reset all servo pos
  fireOutput(pyroOut1, false);
  fireOutput(pyroOut2, false);
  fireOutput(pyroOut3, false);
  fireOutput(pyroOut4, false);
 if (out1Enable == false) Output1Fired = true;
  if (out2Enable == false) Output2Fired = true;
  if (out3Enable == false) Output3Fired = true;
  if (out4Enable == false) Output4Fired = true;
}
//================================================================
// Start program
//================================================================
void setup()
{
  int val = 0;     // variable to store the read value
  int val1 = 0;     // variable to store the read value

  // Read altimeter softcoded configuration
  softConfigValid = readAltiConfig();

  // check if configuration is valid
  if (!softConfigValid)
  {
    //default values
    defaultConfig();
    writeConfigStruc();
  }

  // if the baud rate is invalid let's default it
  if (!CheckValideBaudRate(config.connectionSpeed))
  {
    config.connectionSpeed = 38400;
    writeConfigStruc();
  }
  SerialCom.print(F("Start program\n"));
  initAlti();


  // init Kalman filter
  KalmanInit();

  // initialise the connection
  Wire.begin();

  //You can change the baud rate here
  //and change it to 57600, 115200 etc..
  //Serial.begin(BAUD_RATE);
  SerialCom.begin(config.connectionSpeed);
  //software pull up so that all bluetooth modules work!!! took me a good day to figure it out
  pinMode(PD0, INPUT_PULLUP);
  //Presure Sensor Initialisation
#ifdef BMP085_180
  // Note that BMP180 is compatible with the BMP085 library
  // Low res should work better at high speed
  bmp.begin( config.altimeterResolution);
#endif

#ifdef BMP280
  bmp.begin();
  bmp.setOversampling(config.altimeterResolution)
#endif


  pinMode(pinSpeaker, OUTPUT);

  pinMode(pinAltitude1, INPUT);
  pinMode(pinAltitude2, INPUT);

  digitalWrite(pinSpeaker, LOW);

  //initialisation give the version of the altimeter
  //One long beep per major number and One short beep per minor revision
  //For example version 1.2 would be one long beep and 2 short beep
  beepAltiVersion(MAJOR_VERSION, MINOR_VERSION);

  //number of measures to do to detect Apogee
  measures = config.nbrOfMeasuresForApogee;

  if (!softConfigValid)
  {
    //initialise the deployement altitude for the main
    mainDeployAltitude = 100;

    // On the Alti duo when you close the jumper you set it to 1
    // val is the left jumper and val1 is the right jumper
    //as of version 1.4 only use the jumper if no valid softconfiguration

    val = digitalRead(pinAltitude1);
    val1 = digitalRead(pinAltitude2);
    if (val == 0 && val1 == 0)
    {
      mainDeployAltitude = 50;
    }
    if (val == 0 && val1 == 1)
    {
      mainDeployAltitude = 100;
    }
    if (val == 1 && val1 == 0)
    {
      mainDeployAltitude = 150;
    }
    if (val == 1 && val1 == 1)
    {
      mainDeployAltitude = 200;
    }
  }

  // let's do some dummy altitude reading
  // to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
    ReadAltitude();
  }

  //let's read the launch site altitude
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += ReadAltitude();
    delay(50);
  }
  initialAltitude = (sum / 10.0);
  lastAltitude = 0;//initialAltitude;
  liftoffAltitude = config.liftOffAltitude;//20;

}
/*
   assignPyroOutputs()
   Assign the pyro outputs different fonctionalities such as
   apogge, main, timer, landing or liftoff events

*/
void assignPyroOutputs()
{
  timerEvent1_enable = false;
  timerEvent2_enable = false;
  timerEvent3_enable = false;
  out1Delay = 0;
  out2Delay = 0;
  out3Delay = 0;
  timerEvent4_enable = false;
  out4Delay = 0;

  landingDelay = 0;
  liftOffDelay = 0;
  apogeeDelay = 0;
  mainDelay = 0;
  mainEvent_Enable = false;
  apogeeEvent_Enable = false;
  landingEvent_Enable = false;
  liftOffEvent_Enable = false;

  for (int a = 0 ; a < 4 ; a++ ) {
    pinMain[a] = -1;
    pinApogee[a] = -1;
    pinLanding[a] = -1;
    pinLiftOff[a] = -1;
  }
  pinOut1 = -1;
  pinOut2 = -1;
  pinOut3 = -1;
  pinOut4 = -1;

  switch (config.outPut1)
  {
    case 0:
      mainEvent_Enable = true;
      mainDelay = config.outPut1Delay;
      pinMain[0] = pyroOut1;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut1Delay;
      pinApogee[0] = pyroOut1;
      break;
    case 2:
      timerEvent1_enable = true;
      out1Delay = config.outPut1Delay;
      pinOut1 = pyroOut1;
      break;
    case 4:
      landingEvent_Enable = true;
      landingDelay = config.outPut1Delay;
      pinLanding[0] = pyroOut1;
      break;
    case 5:
      liftOffEvent_Enable = true;
      liftOffDelay = config.outPut1Delay;
      pinLiftOff[0] = pyroOut1;
      break;
    default:
      out1Enable = false;
      break;
  }

  switch (config.outPut2)
  {
    case 0:
      mainEvent_Enable = true;
      pinMain[1] = pyroOut2;
      mainDelay = config.outPut2Delay;
      break;
    case 1:
      apogeeEvent_Enable = true;
      pinApogee[1] = pyroOut2;
      apogeeDelay = config.outPut2Delay;
      break;
    case 2:
      timerEvent2_enable = true;
      out2Delay = config.outPut2Delay;
      pinOut2 = pyroOut2;
      break;
    case 4:
      landingEvent_Enable = true;
      landingDelay = config.outPut2Delay;
      pinLanding[1] = pyroOut2;
      break;
    case 5:
      liftOffEvent_Enable = true;
      liftOffDelay = config.outPut2Delay;
      pinLiftOff[1] = pyroOut2;
      break;
    default:
      out2Enable = false;
      break;
  }
  switch (config.outPut3)
  {
    case 0:
      mainEvent_Enable = true;
      mainDelay = config.outPut3Delay;
      pinMain[2] = pyroOut3;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut3Delay;
      pinApogee[2] = pyroOut3;
      break;
    case 2:
      timerEvent3_enable = true;
      out3Delay = config.outPut3Delay;
      pinOut3 = pyroOut3;
      break;
    case 4:
      landingEvent_Enable = true;
      landingDelay = config.outPut3Delay;
      pinLanding[2] = pyroOut3;
      break;
    case 5:
      liftOffEvent_Enable = true;
      liftOffDelay = config.outPut3Delay;
      pinLiftOff[2] = pyroOut3;
      break;
    default:
      out3Enable = false;
      break;
  }

  //output 4
  switch (config.outPut4)
  {
    case 0:
      mainEvent_Enable = true;
      mainDelay = config.outPut4Delay;
      pinMain[3] = pyroOut4;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut4Delay;
      pinApogee[3] = pyroOut4;
      break;
    case 2:
      timerEvent4_enable = true;
      out4Delay = config.outPut4Delay;
      pinOut4 = pyroOut4;
      break;
    case 4:
      landingEvent_Enable = true;
      landingDelay = config.outPut4Delay;
      pinLanding[3] = pyroOut4;
      break;
    case 5:
      liftOffEvent_Enable = true;
      liftOffDelay = config.outPut4Delay;
      pinLiftOff[3] = pyroOut4;
      break;
    default:
      out4Enable = false;
      break;
  }

}

void setEventState(int pyroOut, boolean state)
{
  if (pyroOut == pyroOut1)
  {
    Output1Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output1Fired"));
#endif
  }

  if (pyroOut == pyroOut2)
  {
    Output2Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output2Fired"));
#endif
  }

  if (pyroOut == pyroOut3)
  {
    Output3Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output3Fired"));
#endif
  }

  if (pyroOut == pyroOut4)
  {
    Output4Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output4Fired"));
#endif
  }

}
/*
   Send telemety so that we can plot the flight

*/
void SendTelemetry(long sampleTime, int freq) {
  char altiTelem[200] = "";
  char temp[10] = "";
  if (telemetryEnable && (millis() - lastTelemetry) > freq) {
    lastTelemetry = millis();
    int val = 0;
    //check liftoff
    int li = 0;
    if (liftOff)
      li = 1;

    //check apogee
    int ap = 0;
    if (apogeeHasFired)
      ap = 1;

    //check main
    int ma = 0;
    if (mainHasFired)
      ma = 1;
    int landed = 0;
    if ( mainHasFired && currAltitude < 10)
      landed = 1;

    strcat(altiTelem, "telemetry," );
    sprintf(temp, "%i,", currAltitude);
    strcat(altiTelem, temp);

    sprintf(temp, "%i,", li);
    strcat(altiTelem, temp);

    sprintf(temp, "%i,", ap);
    strcat(altiTelem, temp);

    sprintf(temp, "%i,", apogeeAltitude);
    strcat(altiTelem, temp);

    sprintf(temp, "%i,", ma);
    strcat(altiTelem, temp);

    sprintf(temp, "%i,", mainAltitude);
    strcat(altiTelem, temp);

    sprintf(temp, "%i,", landed);
    strcat(altiTelem, temp);

    sprintf(temp, "%i,", sampleTime);
    strcat(altiTelem, temp);
    //No continuity
    strcat(altiTelem, "-1,");

    strcat(altiTelem, "-1,");

    strcat(altiTelem, "-1,");

    strcat(altiTelem, "-1,");

    //no bat voltage
    strcat(altiTelem, "-1,");
    // temperature
    float temperature;
    temperature = bmp.readTemperature();

    sprintf(temp, "%i,", (int)temperature );
    strcat(altiTelem, temp);

    unsigned int chk;
    chk = msgChk(altiTelem, sizeof(altiTelem));
    sprintf(temp, "%i", chk);
    strcat(altiTelem, temp);
    strcat(altiTelem, ";\n");
    SerialCom.print("$");
    SerialCom.print(altiTelem);
  }
}
//================================================================
// Main loop which call the menu
//================================================================
void loop()
{
  MainMenu();
}

int currentVelocity(int prevTime, int curTime, int prevAltitude, int curAltitude)
{
  int curSpeed = int (float(curAltitude - prevAltitude) / (float( curTime - prevTime) / 1000));
  return curSpeed;
}

//================================================================
// Function:  recordAltitude()
// called for normal recording
//================================================================
void recordAltitude()
{
  boolean exitLoop = false;
  apogeeReadyToFire = false;
  mainReadyToFire = false;
  landingReadyToFire = false;
  liftOffReadyToFire = false;
  apogeeStartTime = 0;
  mainStartTime = 0;
  landingStartTime = 0;
  liftOffStartTime = 0;
  ignoreAltiMeasure = false;

  Event1Fired = false;
  Event2Fired = false;
  Event3Fired = false;
  Event4Fired = false;

  MainFiredComplete = false;
  LandingFiredComplete = false;
  LiftOffFiredComplete = false;
  landingHasFired = false;
  liftOffHasFired = false;
  mainHasFired = false;
  apogeeHasFired = false;

  if (out1Enable == false) Output1Fired = true;
  if (out2Enable == false) Output2Fired = true;
  if (out3Enable == false) Output3Fired = true;
  if (out4Enable == false) Output4Fired = true;


#ifdef SERIAL_DEBUG

  SerialCom.println(F("Config delay:"));
  SerialCom.println(config.outPut1Delay);
  SerialCom.println(config.outPut2Delay);
  SerialCom.println(config.outPut3Delay);
  SerialCom.println(config.outPut4Delay);

  SerialCom.println(apogeeDelay);
  SerialCom.println(mainDelay);
#endif

  while (!exitLoop )
  {

    //read current altitude
    currAltitude = (ReadAltitude() - initialAltitude);
    if (liftOff)
      SendTelemetry(millis() - initialTime, 200);
    if (( currAltitude > liftoffAltitude)  && !liftOff  && !mainHasFired )
    {
      liftOff = true;
      SendTelemetry(0, 200);
      // save the time
      initialTime = millis();
      if (config.superSonicYesNo == 1)
        ignoreAltiMeasure = true;
    }
    if (liftOff)
    {
#ifdef SERIAL_DEBUG
      SerialCom.println(F("we have lift off\n"));
#endif

      unsigned long prevTime = 0;
      long prevAltitude = 0;
      // loop until we have reach an altitude of 3 meter
      //while(currAltitude > 3 && !MainFiredComplete && liftOff )
      while (liftOff)
      {
        unsigned long currentTime;
        unsigned long diffTime;

        currAltitude = (ReadAltitude() - initialAltitude);

        currentTime = millis() - initialTime;
        if (mainHasFired && !landingHasFired && !landingReadyToFire) {

          if (abs(currentVelocity(prevTime, currentTime, prevAltitude, currAltitude)) < 1  ) {
            //we have landed
            landingReadyToFire = true;
            landingStartTime = millis();
          }
        }
        prevAltitude = currAltitude;
        SendTelemetry(currentTime, 200);
        diffTime = currentTime - prevTime;
        prevTime = currentTime;

        if (!liftOffHasFired) {
          liftOffReadyToFire = true;
          liftOffStartTime = millis();
        }


        if (liftOffReadyToFire)
        {
          if ((millis() - liftOffStartTime) >= liftOffDelay)
          {
            //fire liftOff
            for (int lo = 0; lo < 4; lo++ ) {
              fireOutput(pinLiftOff[lo], true);
            }

            liftOffReadyToFire = false;
            liftOffHasFired = true;
            SendTelemetry(millis() - initialTime, 200);
          }
        }

        if (liftOffHasFired)
        {
          if ((millis() - (liftOffStartTime + liftOffDelay)) >= 1000 && !LiftOffFiredComplete)
          {
            for (int lo = 0; lo < 4; lo++ ) {
              if (config.servoStayOn == 0)
                fireOutput(pinLiftOff[lo], false);
              setEventState(pinLiftOff[lo], true);
            }
            LiftOffFiredComplete = true;
          }
        }
        if (timerEvent1_enable && !Event1Fired )
        {
          if (currentTime >= config.outPut1Delay)
          {
            //Trigger servo 1
            fireOutput(pyroOut1, true);
            Event1Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Fired 1st out"));
#endif
          }
        }
        if (timerEvent1_enable && Event1Fired )
        {
          if ((currentTime - config.outPut1Delay) >= 1000 && !Output1Fired )
          {
            //switch off Servo 1
            if (config.servoStayOn == 0)
              fireOutput(pyroOut1, false);
            Output1Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Finished Firing 1st out"));
#endif
          }
        }
        if (timerEvent2_enable && !Event2Fired )
        {
          if (currentTime >= config.outPut2Delay)
          {
            //Trigger Servo 2
            fireOutput(pyroOut2, true);
            Event2Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Fired 2nd out"));
#endif
          }
        }
        if (timerEvent2_enable && Event2Fired  )
        {
          if ((currentTime - config.outPut2Delay) >= 1000 && !Output2Fired )
          {
            //switch off servo 2
            if (config.servoStayOn == 0)
              fireOutput(pyroOut2, false);
            Output2Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Finished Firing 2nd out"));
#endif
          }
        }
        if (timerEvent3_enable && !Event3Fired )
        {
          if (currentTime >= config.outPut3Delay)
          {
            //fire output pyroOut3
            fireOutput(pyroOut3, true);
            Event3Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Fired 3rd out"));
#endif
          }
        }
        if (timerEvent3_enable && Event3Fired )
        {
          if ((currentTime - config.outPut3Delay) >= 1000 && !Output3Fired)
          {
            //switch off output
            if (config.servoStayOn == 0)
              fireOutput(pyroOut3, false);
            Output3Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Finished Firing 3rd out"));
#endif
          }
        }

        if (timerEvent4_enable && !Event4Fired )
        {
          if (currentTime >= config.outPut4Delay)
          {
            //fire output pyroOut4
            fireOutput(pyroOut4, true);
            Event4Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Fired 4th out"));
#endif
          }
        }
        if (timerEvent4_enable && Event4Fired )
        {
          if ((currentTime - config.outPut4Delay) >= 1000 && !Output4Fired)
          {
            //switch off output pyroOut4
            if (config.servoStayOn == 0)
              fireOutput(pyroOut4, false);
            Output4Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Finished Firing 4th out"));
#endif
          }
        }


        if (config.superSonicYesNo == 1)
        {
          //are we still in superSonic mode?
          if (currentTime > 3000)
            ignoreAltiMeasure = false;
        }
        if (currAltitude < lastAltitude && !apogeeHasFired  && !ignoreAltiMeasure )
        {
          measures = measures - 1;
          if (measures == 0)
          {
            //fire drogue
            apogeeReadyToFire = true;
            apogeeStartTime = millis();
            apogeeAltitude = currAltitude;
          }
        }
        else
        {
          lastAltitude = currAltitude;
          //number of measures to do to detect Apogee
          measures = config.nbrOfMeasuresForApogee;
        }
        if (apogeeReadyToFire)
        {
          if ((millis() - apogeeStartTime) >= apogeeDelay)
          {
            //fire drogue
            for (int ap = 0; ap < 4 ; ap++) {
              fireOutput(pinApogee[ap], true);
              setEventState(pinApogee[ap], true);
            }
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Apogee has fired"));
#endif
            apogeeReadyToFire = false;
            apogeeHasFired = true;
            SendTelemetry(millis() - initialTime, 200);
          }
        }

        if ((currAltitude  < mainDeployAltitude) && apogeeHasFired && !mainHasFired )
        {
          // Deploy main chute  X meters or feet  before landing...
          for (int ap = 0; ap < 4 ; ap++) {
            if (config.servoStayOn == 0)
              fireOutput(pinApogee[ap], false);
          }
#ifdef SERIAL_DEBUG
          SerialCom.println(F("Apogee firing complete"));
#endif
          mainReadyToFire = true;
#ifdef SERIAL_DEBUG
          SerialCom.println(F("preparing main"));
#endif
          mainStartTime = millis();
          //digitalWrite(pinMain, HIGH);
          //mainHasFired=true;

          mainAltitude = currAltitude;
#ifdef SERIAL_DEBUG
          SerialCom.println(F("main altitude"));

          SerialCom.println(mainAltitude);
#endif
        }
        if (mainReadyToFire)
        {
          if ((millis() - mainStartTime) >= mainDelay)
          {
            //fire main
#ifdef SERIAL_DEBUG
            SerialCom.println(F("firing main"));
#endif
            for (int ma = 0; ma < 4; ma++ ) {
              fireOutput(pinMain[ma], true);
              // SerialCom.println(pinMain[ma]);
            }

            mainReadyToFire = false;
            mainHasFired = true;
            SendTelemetry(millis() - initialTime, 200);
          }
        }

        if (mainHasFired)
        {

          if ((millis() - (mainStartTime + mainDelay)) >= 1000 && !MainFiredComplete)
          {
            for (int ma = 0; ma < 4; ma++ ) {
              // turn off main servo(s)
              if (config.servoStayOn == 0)
                fireOutput(pinMain[ma], false);
              setEventState(pinMain[ma], true);
            }
            //setEventState(pinMain, true);
#ifdef SERIAL_DEBUG
            SerialCom.println("Main fired");
#endif
            MainFiredComplete = true;
          }
        }

        if (landingReadyToFire)
        {
          if ((millis() - landingStartTime) >= landingDelay)
          {
            //fire landing
            for (int la = 0; la < 4; la++ ) {
              fireOutput(pinLanding[la], true);
            }

            landingReadyToFire = false;
            landingHasFired = true;
            SendTelemetry(millis() - initialTime, 200);
          }
        }

        if (landingHasFired)
        {
          if ((millis() - (landingStartTime + landingDelay)) >= 1000 && !LandingFiredComplete)
          {
            for (int la = 0; la < 4; la++ ) {
              // turn off servo
              if (config.servoStayOn == 0)
                fireOutput(pinLanding[la], false);
              setEventState(pinLanding[la], true);
            }
            LandingFiredComplete = true;
          }
        }
        //if (MainFiredComplete && currAltitude < 10)
        if (MainFiredComplete && LandingFiredComplete)
        {
          liftOff = false;
          SendTelemetry(millis() - initialTime, 500);
        }
        if (Output1Fired  && Output2Fired  && Output3Fired  && Output4Fired && MainFiredComplete && LandingFiredComplete)
        {
#ifdef SERIAL_DEBUG
          SerialCom.println(F("all event have fired"));
#endif
          exitLoop = true;
          SendTelemetry(millis() - initialTime, 500);
        }

      }
    }
  }
}




//================================================================
// Main menu to interpret all the commands sent by the altimeter console
//================================================================
void MainMenu()
{
  char readVal = ' ';
  int i = 0;

  char commandbuffer[200];

  /* SerialCom.println(F("Rocket flight data logger. A maximum of 25 flight can be logged \n"));
    SerialCom.println(F("Commands are: \n"));
    SerialCom.println(F("w = record flight \n"));
    SerialCom.println(F("r (followed by the flight number) = read flight data\n"));
    SerialCom.println(F("l  = print flight list \n"));
    SerialCom.println(F("e  = erase all flight data \n"));
    SerialCom.println(F("c  = toggle continuity on/off \n"));
    SerialCom.println(F("b  = print alti config \n"));
    SerialCom.println(F("Enter Command and terminate it by a ; >>\n"));*/
  i = 0;
  readVal = ' ';
  while ( readVal != ';')
  {
    if (FastReading == false)
    {
      currAltitude = (ReadAltitude() - initialAltitude);
      if (liftOff)
        SendTelemetry(millis() - initialTime, 200);
      if (( currAltitude > liftoffAltitude) != true)
      {
        SendTelemetry(0, 500);
      }
      else
      {
        recordAltitude();
      }
      long savedTime = millis();
      while (apogeeHasFired == true && mainHasFired == true)
      {

        // check if we have anything on the serial port
        if (SerialCom.available())
        {
          readVal = SerialCom.read();
          if (readVal != ';' )
          {
            if (readVal != '\n')
              commandbuffer[i++] = readVal;
          }
          else
          {
            commandbuffer[i++] = '\0';
            resetFlight();
            interpretCommandBuffer(commandbuffer);
          }
        }

        //beep last altitude every 10 second
        while ((millis() - savedTime) > 10000) {

          beginBeepSeq();

          if (config.beepingMode == 0)
            beepAltitude(apogeeAltitude * FEET_IN_METER);
          else
            beepAltitudeNew(apogeeAltitude * FEET_IN_METER);
          beginBeepSeq();

          if (config.beepingMode == 0)
            beepAltitude(mainAltitude * FEET_IN_METER);
          else
            beepAltitudeNew(mainAltitude * FEET_IN_METER);

          savedTime = millis();
        }
      }
    }

    while (SerialCom.available())
    {
      readVal = SerialCom.read();
      if (readVal != ';' )
      {
        if (readVal != '\n')
          commandbuffer[i++] = readVal;
      }
      else
      {
        commandbuffer[i++] = '\0';
        break;
      }
    }
  }
  interpretCommandBuffer(commandbuffer);
}


/*

   This interprets menu commands. This can be used in the commend line or
   this is used by the Android console

   Commands are as folow:
   e  erase all saved flights
   r  followed by a number which is the flight number.
      This will retrieve all data for the specified flight
   w  Start or stop recording
   n  Return the number of recorded flights in the EEprom
   l  list all flights
   c  toggle continuity on and off
   a  get all flight data
   b  get altimeter config
   s  write altimeter config
   d  reset alti config
   h  hello. Does not do much
   k  folowed by a number turn on or off the selected output
   y  followed by a number turn telemetry on/off. if number is 1 then
      telemetry in on else turn it off
   m  followed by a number turn main loop on/off. if number is 1 then
      main loop in on else turn it off
*/
void interpretCommandBuffer(char *commandbuffer) {
  SerialCom.println((char*)commandbuffer);

  //this will erase all flight
  if (commandbuffer[0] == 'e')
  {
    SerialCom.println(F("Not implemented \n"));
  }
  //this will read one flight
  else if (commandbuffer[0] == 'r')
  {
    SerialCom.println(F("Not implemented \n"));
  }
  //start or stop recording
  else if (commandbuffer[0] == 'w')
  {
    SerialCom.println(F("Not implemented \n"));
  }
  //Number of flight
  else if (commandbuffer[0] == 'n')
  {
    SerialCom.println(F("Not implemented \n"));
  }
  //list all flights
  else if (commandbuffer[0] == 'l')
  {
    SerialCom.println(F("Not implemented \n"));
  }

  //toggle continuity on and off
  else if (commandbuffer[0] == 'c')
  {
    SerialCom.println(F("Not implemented \n"));
  }
  //get all flight data
  else if (commandbuffer[0] == 'a')
  {
    SerialCom.println(F("Not implemented \n"));
  }
  //get altimeter config
  else if (commandbuffer[0] == 'b')
  {
    SerialCom.print(F("$start;\n"));

    printAltiConfig();

    SerialCom.print(F("$end;\n"));
  }
  //write altimeter config
  else if (commandbuffer[0] == 's')
  {
    if (writeAltiConfig(commandbuffer)) {
      readAltiConfig();
      assignPyroOutputs();
      SerialCom.print(F("$OK;\n"));
    }
    else
      SerialCom.print(F("$KO;\n"));
  }
  //reset alti config
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    writeConfigStruc();
    readAltiConfig();
    assignPyroOutputs();
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    //FastReading = false;
    SerialCom.print(F("$OK;\n"));
  }
  //FastReading
  else if (commandbuffer[0] == 'f')
  {
    FastReading = true;
    SerialCom.print(F("$OK;\n"));
  }
  //turn on or off the selected output
  else if (commandbuffer[0] == 'k')
  {
    char temp[2];
    boolean fire = true;

    temp[0] = commandbuffer[1];
    temp[1] = '\0';
    if (commandbuffer[2] == 'F')
      fire = false;

    if (atol(temp) > -1)
    {
      switch (atoi(temp))
      {
        case 1:
          fireOutput(pyroOut1, fire);
          break;
        case 2:
          fireOutput(pyroOut2, fire);
          break;
        case 3:
          fireOutput(pyroOut3, fire);
          break;
        case 4:
          fireOutput(pyroOut4, fire);
          break;
      }
    }
  }
  //telemetry on/off
  else if (commandbuffer[0] == 'y')
  {
    if (commandbuffer[1] == '1') {
      SerialCom.print(F("Telemetry enabled\n"));
      telemetryEnable = true;
    }
    else {
      SerialCom.print(F("Telemetry disabled\n"));
      telemetryEnable = false;
    }
    SerialCom.print(F("$OK;\n"));
  }
  //mainloop on/off
  else if (commandbuffer[0] == 'm')
  {
    if (commandbuffer[1] == '1') {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main Loop enabled\n"));
#endif
      //mainLoopEnable = true;
      FastReading = false;
    }
    else {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main loop disabled\n"));
#endif
      //mainLoopEnable = false;
      FastReading = true;
    }
    SerialCom.print(F("$OK;\n"));
  }
  //FastReading off
  else if (commandbuffer[0] == 'g')
  {
    FastReading = false;
    SerialCom.print(F("$OK;\n"));
  }

  else if (commandbuffer[0] == 't')
  {
    //reset config
    defaultConfig();
    writeConfigStruc();
    SerialCom.print(F("config reseted\n"));
  }
  else if (commandbuffer[0] == 'i')
  {
    //exit continuity mode
  }
  else if (commandbuffer[0] == ' ')
  {
    SerialCom.print(F("$K0;\n"));
  }

  else
  {
    SerialCom.print(F("$UNKNOWN;"));
    SerialCom.println(commandbuffer[0]);

  }
}

void resetFlight() {
  // re-nitialise all flight related global variables
  apogeeHasFired = false;
  mainHasFired = false;
  liftOff = false;
  Output1Fired = false;
  Output2Fired = false;
  Output3Fired = false;
  Output4Fired = false;
}

void fireOutput(int pin, boolean On) {

  int angle;
  if (pyroOut1 == pin) {

    if (On)
      angle = config.servo1OnPos;
    else
      angle = config.servo1OffPos;
    Servo1.write(angle);
  }
  else if (pyroOut2 == pin) {
    if (On)
      angle = config.servo2OnPos;
    else
      angle = config.servo2OffPos;
    Servo2.write(angle);
  }
  else if (pyroOut3 == pin) {
    if (On)
      angle = config.servo3OnPos;
    else
      angle = config.servo3OffPos;
    Servo3.write(angle);
  }
  else if (pyroOut4 == pin) {
    if (On)
      angle = config.servo4OnPos;
    else
      angle = config.servo4OffPos;
    Servo4.write(angle);
  }
}
