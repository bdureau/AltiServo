/*
  Rocket Servo altimeter ver 1.0
  Copyright Boris du Reau 2012-2019

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
// enable/disable output
boolean out1Enable = true;
boolean out2Enable = true;
boolean out3Enable = true;
boolean out4Enable = true;

int apogeeDelay = 0;
int mainDelay = 0;
int out1Delay = 0;
int out2Delay = 0;
int out3Delay = 0;
int out4Delay = 0;

boolean Output1Fired = false;
boolean Output2Fired = false;
boolean Output3Fired = false;
boolean Output4Fired = false;

boolean telemetryEnable = false;

Servo Servo1;   // First Servo off the chassis
Servo Servo2;   // Second Servo off the chassis
Servo Servo3;
Servo Servo4;
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

  // set main altitude (if in feet convert to metrics)
  if (config.unit == 0)
    FEET_IN_METER = 1;
  else
    FEET_IN_METER = 3.28084 ;

  mainDeployAltitude = int(config.mainAltitude / FEET_IN_METER);
  // beepFrequency
  beepingFrequency = config.beepingFrequency;

  // init Kalman filter
  KalmanInit();

  // initialise the connection
  Wire.begin();

  //You can change the baud rate here
  //and change it to 57600, 115200 etc..
  //Serial.begin(BAUD_RATE);
  SerialCom.begin(config.connectionSpeed);

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
  //our drogue has not been fired
  apogeeHasFired = false;
  mainHasFired = false;

  SerialCom.print(F("Start program\n"));
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

  pinMode(pinSpeaker, OUTPUT);

  pinMode(pinAltitude1, INPUT);
  pinMode(pinAltitude2, INPUT);

  digitalWrite(pinSpeaker, LOW);

  //enable or disable continuity check
  if (config.noContinuity == 1)
    noContinuity = true;
  else
    noContinuity = false;

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
  liftoffAltitude = 20;

}
void assignPyroOutputs()
{
  pinMain = -1;
  pinApogee = -1;
  pinOut1 = -1;
  pinOut2 = -1;
  pinOut3 = -1;
  pinOut4 = -1;

  switch (config.outPut1)
  {
    case 0:
      mainEvent_Enable = true;
      mainDelay = config.outPut1Delay;
      pinMain = pyroOut1;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut1Delay;
      pinApogee = pyroOut1;
      break;
    case 2:
      timerEvent1_enable = true;
      out1Delay = config.outPut1Delay;
      pinOut1 = pyroOut1;
      break;
    default:
      out1Enable = false;
      break;
  }

  switch (config.outPut2)
  {
    case 0:
      mainEvent_Enable = true;
      pinMain = pyroOut2;
      mainDelay = config.outPut2Delay;
      break;
    case 1:
      apogeeEvent_Enable = true;
      pinApogee = pyroOut2;
      apogeeDelay = config.outPut2Delay;
      break;
    case 2:
      timerEvent2_enable = true;
      out2Delay = config.outPut2Delay;
      pinOut2 = pyroOut2;
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
      pinMain = pyroOut3;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut3Delay;
      pinApogee = pyroOut3;
      break;
    case 2:
      timerEvent3_enable = true;
      out3Delay = config.outPut3Delay;
      pinOut3 = pyroOut3;
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
      pinMain = pyroOut4;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut4Delay;
      pinApogee = pyroOut4;
      break;
    case 2:
      timerEvent4_enable = true;
      out4Delay = config.outPut4Delay;
      pinOut4 = pyroOut4;
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

void SendTelemetry(long sampleTime) {
  if (telemetryEnable) {
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
    SerialCom.print(F("$telemetry,"));
    SerialCom.print(currAltitude);
    SerialCom.print(F(","));
    SerialCom.print(li);
    SerialCom.print(F(","));
    SerialCom.print(ap);
    SerialCom.print(F(","));
    SerialCom.print(apogeeAltitude);
    SerialCom.print(F(","));
    SerialCom.print(ma);
    SerialCom.print(F(","));
    SerialCom.print(mainAltitude);
    SerialCom.print(F(","));
    SerialCom.print(landed);
    SerialCom.print(F(","));
    SerialCom.print(sampleTime);
    SerialCom.print(F(","));
    if (out1Enable) {
      //check continuity
      SerialCom.print(0);
    }
    else {
      SerialCom.print(-1);
    }
    SerialCom.print(F(","));
    if (out2Enable) {
      //check continuity
      SerialCom.print(0);
    }
    else {
      SerialCom.print(-1);
    }
    SerialCom.print(F(","));
    if (out3Enable) {
      //check continuity
      SerialCom.print(0);
    }
    else {
      SerialCom.print(-1);
    }

    SerialCom.print(F(","));
    if (out4Enable) {
      //check continuity
      SerialCom.print(0);
    }
    else {
      SerialCom.print(-1);
    }
    SerialCom.print(F(","));
    SerialCom.print(-1);
     // temperature
    SerialCom.print(F(","));
    float temperature;
    temperature = bmp.readTemperature();
    SerialCom.print((int)temperature );
    SerialCom.println(F(";"));
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
  int curSpeed = int ((curAltitude - prevAltitude) / ( curTime - prevTime));
  return curSpeed;
}

//================================================================
// Function:  recordAltitude()
// called for normal recording
//================================================================
void recordAltitude()
{
  boolean exit = false;
  boolean apogeeReadyToFire = false;
  boolean mainReadyToFire = false;
  unsigned long apogeeStartTime = 0;
  unsigned long mainStartTime = 0;
  //unsigned long liftoffStartTime=0;
  boolean ignoreAltiMeasure = false;
  // boolean finishedEvent = false;
  boolean Event1Fired = false;
  boolean Event2Fired = false;
  boolean Event3Fired = false;
  boolean Event4Fired = false;

  boolean MainFiredComplete = false;

  if (out1Enable == false) Output1Fired = true;
  if (out2Enable == false) Output2Fired = true;
  if (out3Enable == false) Output3Fired = true;
  if (out4Enable == false) Output4Fired = true;


#ifdef SERIAL_DEBUG
  if (pinMain == -1) SerialCom.println(F("Main disable\n"));
  if (pinApogee == -1) SerialCom.println(F("Apogee disable\n"));
  if (pinOut1 == -1) SerialCom.println(F("pinOut1 disable\n"));
  if (pinOut2 == -1) SerialCom.println(F("pinOut2 disable\n"));
  if (pinOut3 == -1) SerialCom.println(F("pinOut3 disable\n"));

  if (pinOut4 == -1) SerialCom.println(F("pinOut4 disable\n"));

  SerialCom.println(F("Config delay:"));
  SerialCom.println(config.outPut1Delay);
  SerialCom.println(config.outPut2Delay);
  SerialCom.println(config.outPut3Delay);

  SerialCom.println(config.outPut4Delay);


  SerialCom.println(apogeeDelay);
  SerialCom.println(mainDelay);
#endif

  while (exit == false)
  {

    //read current altitude
    currAltitude = (ReadAltitude() - initialAltitude);
    if (liftOff)
      SendTelemetry(millis() - initialTime);
    if (( currAltitude > liftoffAltitude) == true && liftOff == false && mainHasFired == false)
    {
      liftOff = true;
      SendTelemetry(0);
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
      // loop until we have reach an altitude of 3 meter
      //while(currAltitude > 3 && MainFiredComplete==false && liftOff ==true;)
      while (liftOff == true)
      {
        unsigned long currentTime;
        unsigned long diffTime;
        unsigned long timerEvent1_startime;

        currAltitude = (ReadAltitude() - initialAltitude);

        currentTime = millis() - initialTime;
        SendTelemetry(currentTime);
        diffTime = currentTime - prevTime;
        prevTime = currentTime;
        if (timerEvent1_enable && Event1Fired == false)
        {
          if (currentTime >= config.outPut1Delay)
          {
            //fire output pyroOut1
            fireOutput(pyroOut1, true);
            timerEvent1_startime = currentTime;
            Event1Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Fired 1st out"));
#endif
          }
        }
        if (timerEvent1_enable && Event1Fired == true)
        {
          if ((currentTime - config.outPut1Delay) >= 1000 && Output1Fired == false)
          {
            //switch off output pyroOut1
            Output1Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Finished Firing 1st out"));
#endif
          }
        }
        if (timerEvent2_enable && Event2Fired == false)
        {
          if (currentTime >= config.outPut2Delay)
          {
            //fire output pyroOut2
            fireOutput(pyroOut2, true);
            Event2Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Fired 2nd out"));
#endif
          }
        }
        if (timerEvent2_enable && Event2Fired == true )
        {
          if ((currentTime - config.outPut2Delay) >= 1000 && Output2Fired == false)
          {
            //switch off output pyroOut2
            fireOutput(pyroOut2, false);
            Output2Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Finished Firing 2nd out"));
#endif
          }
        }
        if (timerEvent3_enable && Event3Fired == false)
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
        if (timerEvent3_enable && Event3Fired == true)
        {
          if ((currentTime - config.outPut3Delay) >= 1000 && Output3Fired == false)
          {
            //switch off output pyroOut3
            fireOutput(pyroOut3, false);
            Output3Fired = true;
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Finished Firing 3rd out"));
#endif
          }
        }

        if (timerEvent4_enable && Event4Fired == false)
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
        if (timerEvent4_enable && Event4Fired == true)
        {
          if ((currentTime - config.outPut4Delay) >= 1000 && Output4Fired == false)
          {
            //switch off output pyroOut4
            //digitalWrite(pyroOut4, LOW);
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
        if (currAltitude < lastAltitude && apogeeHasFired == false && ignoreAltiMeasure == false)
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
            fireOutput(pinApogee, true);
            setEventState(pinApogee, true);
#ifdef SERIAL_DEBUG
            SerialCom.println(F("Apogee has fired"));
#endif
            apogeeReadyToFire = false;
            apogeeHasFired = true;
            SendTelemetry(millis() - initialTime);
          }
        }

        if ((currAltitude  < mainDeployAltitude) && apogeeHasFired == true && mainHasFired == false)
        {
          // Deploy main chute  X meters or feet  before landing...
          //digitalWrite(pinApogee, LOW);
          fireOutput(pinApogee, false);
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
          //Serial.println("conf delay main" + config.outPut1Delay );
          //Serial.println("conf delay" + config.outPut2Delay );
          SerialCom.println(mainStartTime);

          if ((millis() - mainStartTime) >= mainDelay)
          {
            //fire main
#ifdef SERIAL_DEBUG
            SerialCom.println(F("firing main"));
#endif
            //digitalWrite(pinMain, HIGH);
            fireOutput(pinMain, true);
            mainReadyToFire = false;
            //setEventState(pinMain, true);
            mainHasFired = true;
            SendTelemetry(millis() - initialTime);
          }
        }

        if (mainHasFired)
        {

          if ((millis() - (mainStartTime + mainDelay)) >= 1000 && MainFiredComplete == false)
          {
            //digitalWrite(pinMain, LOW);
            setEventState(pinMain, true);
            //liftOff =false;
#ifdef SERIAL_DEBUG
            SerialCom.println("Main fired");
#endif
            MainFiredComplete = true;
          }
        }

        if (MainFiredComplete && currAltitude < 10)
        {
          liftOff = false;
          SendTelemetry(millis() - initialTime);
        }
        if (Output1Fired == true && Output2Fired == true && Output3Fired == true && Output4Fired == true)
        {
#ifdef SERIAL_DEBUG
          SerialCom.println(F("all event have fired"));
#endif
          exit = true;
          SendTelemetry(millis() - initialTime);
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

  SerialCom.println(F("Rocket flight data logger. A maximum of 25 flight can be logged \n"));
  SerialCom.println(F("Commands are: \n"));
  SerialCom.println(F("w = record flight \n"));
  SerialCom.println(F("r (followed by the flight number) = read flight data\n"));
  SerialCom.println(F("l  = print flight list \n"));
  SerialCom.println(F("e  = erase all flight data \n"));
  SerialCom.println(F("c  = toggle continuity on/off \n"));
  SerialCom.println(F("b  = print alti config \n"));
  SerialCom.println(F("Enter Command and terminate it by a ; >>\n"));
  i = 0;
  readVal = ' ';
  while ( readVal != ';')
  {
    if (FastReading == false)
    {
      currAltitude = (ReadAltitude() - initialAltitude);
      if (liftOff)
        SendTelemetry(millis() - initialTime);
      if (( currAltitude > liftoffAltitude) != true)
      {
        SendTelemetry(0);
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


void interpretCommandBuffer(char *commandbuffer) {
  SerialCom.println((char*)commandbuffer);


  //toggle continuity on and off
  if (commandbuffer[0] == 'c')
  {
    if (noContinuity == false)
    {
      noContinuity = true;
      SerialCom.println(F("Continuity off \n"));
    }
    else
    {
      noContinuity = false;
      SerialCom.println(F("Continuity on \n"));
    }
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
    writeAltiConfig(commandbuffer);
  }
  //reset alti config
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    writeConfigStruc();
  }
  //FastReading
  else if (commandbuffer[0] == 'f')
  {
    FastReading = true;
    SerialCom.print(F("$OK;\n"));
  }
  //FastReading off
  else if (commandbuffer[0] == 'g')
  {
    FastReading = false;
    SerialCom.print(F("$OK;\n"));
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    //FastReading = false;
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
    if(commandbuffer[1]=='1') {
      SerialCom.print(F("Telemetry enabled\n"));
      telemetryEnable = true;
    }   
    else{
      SerialCom.print(F("Telemetry disabled\n"));
      telemetryEnable = false;
    }   
    SerialCom.print(F("$OK;\n"));
  }
  else if (commandbuffer[0] == ' ')
  {
    SerialCom.print(F("$K0;\n"));
  }

  else
  {
    // Serial.println(F("Unknown command" ));
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
