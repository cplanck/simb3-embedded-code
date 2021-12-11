////////////////////////////////////////////////////////////
//#define ECHO_TO_SERIAL // Allows serial output if uncommented
#define DEBUG 1
////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Wire.h>
#include <RTCZero.h>
#include <LineBuffer.h>
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
//#include <TimeLib.h>

#include <IridiumSBD.h>
#include <GPS.h>
#include <LTC2945.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Maxbotix.h>
#include <AirmarSS510.h>
#include <SFE_LSM9DS0_TAV.h>
#include <BruncinDTC.h>
#include <DS2482.h>
#include <Adafruit_ADS1015.h>
#include <SD.h>


// Version ----------------------------------------------------------

#define PROGRAM_VERSION         0x00

#define SBD_RECORD_ID           0xA0
#define SBD_RECORD_VERSION      0x00
#define SBD_RECORD_HEADER       (SBD_RECORD_ID | SBD_RECORD_VERSION)

// Program Options --------------------------------------------------

#define IRIDIUM_ATTEMPTS         10
#define IRIDIUM_RETRY_DELAY      10000    //10 seconds

#define GPS_TIMEOUT              60000    //60 seconds

// Pin mappings -----------------------------------------------------

#define NUBC_MAXBOTIX_ENABLE    A0
#define NUBC_ENVELOPE_ENABLE    A1
#define NUBC_GPS_ENABLE         12
#define NUBC_IRIDIUM_ENABLE     13
#define NUBC_5V_ENABLE          5
#define NUBC_12V_ENABLE         6

#define NUBC_LED_G              A3
#define NUBC_CHIP_SELECT        A4

#define NUBC_WDT_RESET          A1

// I2C Addresses ----------------------------------------------------

#define NUBC_GYRO_ADDR          0x6A
#define NUBC_MAG_ADDR           0x1E
#define NUBC_POWERMON_ADDR      0x6F
#define NUBC_BMP280             0x77
#define CIBC_ADS                0x48

#define NUBC_POWERMON_RESISTOR  0.02
#define TAC_BYTES               120

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
// Required for Serial on Zero based boards
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

Uart          Serial2         (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

LTC2945           powermon        (NUBC_POWERMON_ADDR, NUBC_POWERMON_RESISTOR);
DS2482            oneWireA        (0);
DS2482            oneWireB        (1);

IridiumSBD        iridium         (Serial1, NUBC_IRIDIUM_ENABLE);
Adafruit_GPS      GPS             (&Serial1);
RTCZero           rtc;    // Create RTC object
BruncinDTC        dtc             (&Serial1);
AirmarSS510       airmar          (Serial2);
Maxbotix          maxbotix        (Serial2);
Adafruit_BME280   bme; // I2C


boolean           errorState;
//boolean           transmitSuccessful = true;

uint32_t          startTime;
uint32_t          prevTime;
float             accPower;

int               iridiumSignal;
uint8_t           iridiumCount;
int               iridiumError;
int               NextAlarmHour; // Variable to hold next alarm time in hours

/* Change these values to set the current initial time */
const byte hours = 13;
const byte minutes = 49;
const byte seconds = 33;
/* Change these values to set the current initial date */
const byte DAY = 15;
const byte MONTH = 11;
const byte YEAR = 18;

int iSQ;

char fileName[13];

int sleepCycleCount;
uint8_t runCounter;
bool sdPresent; 

// SBD message format -----------------------------------------------

typedef union {

  struct {
    uint8_t     header;
    uint8_t     programVersion;
    int32_t     timestamp;
    int32_t     latitude;
    int32_t     longitude;
    int16_t     airTemp;
    uint16_t    airPressure;
    uint16_t    waterDepth;
    int16_t     waterTemp;
    uint16_t    snowDist;

    int16_t     pitch;
    int16_t     roll;
    int16_t     heading;

    uint16_t    batteryVoltage;
    uint8_t     gpsSatellites;
    uint8_t     iridiumSignal;
    uint8_t     iridiumRetries;

    int16_t     TAC[TAC_BYTES];

  } __attribute__((packed));

  uint8_t bytes[0];

} SBDMessage;

SBDMessage message;

/////////////////////////////////////////////////////////////////////

//-- Helper functions for printing ----------------------------------

void printDigits(int digits, char sep = ' ')
{
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
  Serial.print(sep);
}

void printFloat(float value)
{
  if (value < 10) {
    Serial.print(F("   "));
  } else if (value < 100) {
    Serial.print(F("  "));
  } else if (value < 1000) {
    Serial.write(' ');
  }

  if (value >= 0) {
    Serial.write(' ');
  }

  Serial.print(value, 2);
}

void printFloatUnits(float value, FlashString units)
{
  printFloat(value);
  Serial.print(' ');
  Serial.print(units);
  Serial.print(F("  "));
}


void printLabelFloat(FlashString msg, float value, FlashString units)
{
  Serial.print(msg);
  Serial.print(": ");
  Serial.print(value, 2);
  Serial.print(' ');
  Serial.print(units);
  Serial.print("  ");
}

void printLabelFloat(FlashString msg, float value)
{
  printLabelFloat(msg, value, F(""));
}

void printLabelInt(FlashString msg, int value)
{
  Serial.print(msg);
  Serial.print(": ");
  Serial.print(value);
  Serial.print(" ");
}

void printLabelInt(FlashString msg, unsigned int value)
{
  Serial.print(msg);
  Serial.print(": ");
  Serial.print(value);
  Serial.print(" ");
}

void printString(FlashString msg, int width = 12)
{
  size_t k = Serial.print(msg);

  for (; k < width; k++) {
    Serial.print(' ');
  }
}

void printSeparator(char c, uint8_t len = 75)
{
  for (int k = 0; k < len; k++) {
    Serial.print(c);
  }
  Serial.println();
}

void displayBanner(char* msg, char top = '=', char bottom = '-')
{
  printSeparator(top);
  Serial.println(msg);
  printSeparator(bottom);
}

//-- SBD Message ----------------------------------------------------

void clearMessage() {

  memset(message.bytes, 0, sizeof(message));

  //message.header = SBD_RECORD_HEADER;
  message.header = runCounter;
  message.programVersion = PROGRAM_VERSION;
  message.timestamp = rtc.getEpoch();
  
}

int16_t floatToInt(float value, float scale)
{
  if (value < 0) {
    return int16_t(value * scale - 0.5);
  } else {
    return int16_t(value * scale + 0.5);
  }
}

/////////////////////////////////////////////////////////////////////

void setup()
{
  rtc.begin();    // Start the RTC in 24hr mode
  rtc.setTime(hours, minutes, seconds);   // Set the time
  rtc.setDate(DAY, MONTH, YEAR);    // Set the date

#ifdef ECHO_TO_SERIAL
  while (! Serial); // Wait until Serial is ready
  Serial.begin(115200);
  Serial.println("\r\nCryosphere Innovation CryoBoard");
#endif
  Wire.begin(); // Join the I2C bus as master

  configureIO();
  resetWatchdog(); //Pet the watchdog

  digitalWrite(NUBC_5V_ENABLE, HIGH);
  delay(20); //Let the SD PSU power up (Probably not needed)
  Serial.print(F("Initializing SD card..."));

  // see if the card is present and can be initialized:
  if (!SD.begin(NUBC_CHIP_SELECT)) {
    Serial.println(F("Card failed, or not present"));
    sdPresent = 0;
  } else {
    Serial.println(F("card initialized."));
  sdPresent = 1;
  }
  digitalWrite(NUBC_5V_ENABLE, LOW);

  Serial.println();
  Serial.print(F("CRYOSPHERE INNOVATION SIMB3 Ver "));
  Serial.print(PROGRAM_VERSION);
  Serial.print(F(" ... "));
  Serial.println(F("Ready"));

  digitalWrite(NUBC_LED_G, HIGH);
  //delay(15000);
  //digitalWrite(13, LOW);
}

void loop()
{
  resetWatchdog(); //Pet the watchdog
  startTime = prevTime = millis();
  accPower = 0;
  runCounter++;

//  Serial.println(F("Waiting for 10 seconds...in case you screwed up..."));
//  delay(10000);

  Serial.println(F("Begin loop()..."));

  displayBanner("Start Sampling", '=', '-');

//  if (iridiumError != 0) {
//    message.programVersion = 0xF5;
//    Serial.println(F("Transmitting previous message on Iridium..."));
//    iridiumOn();
//    sendIridium();
//    iridiumOff();
//    showElapsed(F("Iridium"));
//    Serial.println(F("Done transmitting..."));
//    Serial.print(F("Iridium status: "));
//    Serial.println(iridiumError);
//  }

  clearMessage();

  powermon.wakeup();

  digitalWrite(NUBC_5V_ENABLE, HIGH);
  digitalWrite(NUBC_MAXBOTIX_ENABLE, HIGH);
  digitalWrite(NUBC_GPS_ENABLE, HIGH);

  Serial.println(F("Reading Air Temp..."));
  readAirTemp();
  showElapsed(F("1-Wire A"));

  Serial.println(F("Reading Barometer..."));
  readBarometer();
  showElapsed(F("BMP280 Barometer"));
  resetWatchdog(); //Pet the watchdog

  Serial.println(F("Reading GPS..."));
  GPS.begin(9600);
  //GPS.begin(9600);
  configureGPS();
  if (readGPS()) {
    rtc.setTime(GPS.hour, GPS.minute, GPS.seconds);   // Set the time
    //rtc.setTime(GPS.hour, 45, GPS.seconds);   // Short test, don't leave me in!!
    rtc.setDate(GPS.day, GPS.month, GPS.year);    // Set the date
    message.timestamp = rtc.getEpoch(); //Second place this gets set, just in case.
  }
  //delay(5000);
  showElapsed(F("GPS"));
  Serial.println(F("GPS Read...Turning off GPS..."));
  digitalWrite(NUBC_GPS_ENABLE, LOW);
  resetWatchdog(); //Pet the watchdog

  digitalWrite(NUBC_12V_ENABLE, LOW);

  int dtcLoop = 0;

  while (dtcLoop < 1) {//change this to something like 2.
    Serial.println(F("Reading DTC..."));
    //insert some better DTC checking code in case it failed...
    dtc.reset();
    Serial1.begin(19200);
    errorState = dtc.read();
    if (errorState) {
      uint16_t *c;
      //Serial.println(message.airTemp);
      if(dtc.packBits()){
        //Serial.println("Should quit trying!");
        dtcLoop = 3;
      }
      c = dtc.outBits();
      memcpy(message.TAC, c, sizeof(message.TAC));
      //memset(c, 0, sizeof(c));
    }
    dtc.reset();
    dtcLoop++;
  }
  showElapsed(F("DTC"));

  Serial.println(F("Reading Snow Depth..."));
  readMaxbotix();
  showElapsed(F("Maxbotix"));
  digitalWrite(NUBC_MAXBOTIX_ENABLE, LOW);

  Serial.println(F("Reading Airmar..."));
  readAirmar();
  showElapsed(F("Airmar"));
  digitalWrite(NUBC_12V_ENABLE, HIGH);

  ///////////////////////////////////////////////////////////////////////////////
  //SD CARD CODE
  ///////////////////////////////////////////////////////////////////////////////
  if (sdPresent) {
    snprintf( fileName, sizeof(fileName), "BL%02d%02d%02d.BIN", rtc.getDay(), rtc.getMonth(), rtc.getYear());

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open(fileName, FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.write(message.bytes, sizeof(message));
      dataFile.println();
      dataFile.close();
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.print("error opening ");
      Serial.println(fileName);
    }
  }

  displayMessage();


// -------Disable Iridium transmission for testing--------
  Serial.println(F("Transmitting on Iridium..."));
  iridiumOn();
  Serial.println(F("Iridium is on."));
  resetWatchdog(); //Pet the watchdog
  sendIridium();
  iridiumOff();
  showElapsed(F("Iridium"));
  Serial.println(F("Done transmitting..."));
  Serial.print(F("Iridium status: "));
  Serial.println(iridiumError);
  if (iridiumError == 0) {
    digitalWrite(NUBC_LED_G, LOW);
  }
// -------Disable Iridium transmission for testing--------


  
//  if (iridiumError == 0) {
//    transmitSuccessful = true;
//  } else {
//    transmitSuccessful = false;
//  }

  //displayMessage();

  digitalWrite(NUBC_12V_ENABLE, HIGH);
  digitalWrite(NUBC_5V_ENABLE, LOW);

  powermon.shutdown();

#if DEBUG
  printSeparator('-');
  Serial.print(F("Integrated power: "));
  printFloatUnits(accPower, F("mWH"));
  Serial.println();
  printSeparator('-');

//  displayMessage();

  displayBanner("Going to sleep", '-', '=');
#endif
  sleepCycleCount = 0;
  buoySleep();
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void configureIO()
{
  pinMode(NUBC_MAXBOTIX_ENABLE, OUTPUT);
  //pinMode(NUBC_ENVELOPE_ENABLE, OUTPUT); //Not used
  pinMode(NUBC_GPS_ENABLE, OUTPUT);
  pinMode(NUBC_IRIDIUM_ENABLE, OUTPUT);
  pinMode(NUBC_5V_ENABLE, OUTPUT);
  pinMode(NUBC_12V_ENABLE, OUTPUT);
  pinMode(NUBC_LED_G, OUTPUT);
  pinMode(NUBC_WDT_RESET, OUTPUT);

  digitalWrite(NUBC_MAXBOTIX_ENABLE, LOW);
  //digitalWrite(NUBC_ENVELOPE_ENABLE, LOW); //Not used
  digitalWrite(NUBC_GPS_ENABLE, LOW);
  digitalWrite(NUBC_IRIDIUM_ENABLE, LOW);
  digitalWrite(NUBC_5V_ENABLE, LOW);
  digitalWrite(NUBC_12V_ENABLE, HIGH);
  digitalWrite(NUBC_LED_G, LOW);
  digitalWrite(NUBC_WDT_RESET, LOW);

  detachInterrupt(0);
  detachInterrupt(1);

  Serial.print(F("32"));
  powermon.wakeup();
  Serial.print(F("32"));
  powermon.write(LTC2945_CONTROL_REG, LTC2945_SENSE_MONITOR);
}

void configureIridium()
{
  Serial.println(F("Configuring the iridium"));
  iridium.attachConsole(Serial);
  iridium.attachDiags(Serial);
  //iridium.begin();
  Serial.println(F("after iridium begin"));
  iridium.setPowerProfile(1);
  Serial.println(F("after set power profile"));
  iridium.useMSSTMWorkaround(false);
}

void iridiumOn()
{
  digitalWrite(NUBC_IRIDIUM_ENABLE, HIGH);
  Serial.println(F("Turned on the iridium"));
  Serial1.begin(19200);
  configureIridium();
  Serial.println(F("configured the iridium"));
  //  delay(100);
  iridium.isAsleep();
  iridium.begin();
}

void iridiumOff()
{
  iridium.sleep();
  digitalWrite(NUBC_IRIDIUM_ENABLE, LOW);
}

void sendIridium()
{
  iridiumSignal = -1;
  iridiumCount  = 0;
  iridiumError  = -1;

  // Check Iridium signal strength, loop until above 2 or timeout

//      for (iridiumCount=0; iridiumCount<IRIDIUM_ATTEMPTS; iridiumCount++) {
//          if (iridium.getSignalQuality(iridiumSignal)==ISBD_SUCCESS) {
//              if (iridiumSignal>1) {
//                  break;
//              }
//          }
//          delay(IRIDIUM_RETRY_DELAY);
//      }
//  
//      if (iridiumSignal<2)
//          return;
  iridium.getSignalQuality(iridiumSignal);
  message.iridiumSignal = iridiumSignal;
  //    message.iridiumRetries = iridium.getRetries();   // From previous send attempt

  // Test code to replace message segments
  //message.waterDepth = 0xAB;
  //message.waterTemp = 0xCD;  
  
  iridiumError = iridium.sendSBDBinary(message.bytes, sizeof(message)); //actually transmit
  //const uint8_t mess[] = "This is a test of SIMB3 comms.";
  //iridiumError = iridium.sendSBDBinary(mess, sizeof(mess));
}

void configureGPS()
{
  digitalWrite(NUBC_GPS_ENABLE, HIGH);

  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.reset();
}

boolean readGPS()
{
  uint32_t timeout = millis() + GPS_TIMEOUT;
  bool found = false;

  //gpsSerial.flush();
  GPS.reset();

  while ((millis() < timeout) && !found) {
    GPS.read();
    found = GPS.newNMEAreceived() &&
            GPS.parse(GPS.lastNMEA()) &&
            GPS.fix &&
            GPS.satellites > 0;
  }

  message.latitude  = int32_t(GPS.latitude * 1000000);
  message.longitude = int32_t(GPS.longitude * 1000000);
  message.gpsSatellites = GPS.satellites;

  return found;
}

void displayMessage()
{
  // Header

  printString(F("Header"));
  Serial.print(F("ID: "));
  Serial.print(message.header & 0xF0);
  Serial.print(F(" Format: "));
  Serial.print(message.header & 0x0F);
  Serial.print(F(" Software Version: "));
  Serial.println(message.programVersion);

  printString(F("Timestamp"));
  Serial.println(message.timestamp);

  // GPS

  printString(F("GPS Time"));
  Serial.print("20");
  printDigits(GPS.year, '-');
  printDigits(GPS.month, '-');
  printDigits(GPS.day);

  printDigits(GPS.hour, ':');
  printDigits(GPS.minute, ':');
  printDigits(GPS.seconds, '\n');

  printString(F("GPS Pos"));
  Serial.print(GPS.latitude, 5);
  Serial.print(F(", "));
  Serial.print(GPS.longitude, 5);

  printLabelInt(F(", fix"), GPS.fix);
  printLabelInt(F("satellites"), message.gpsSatellites);
  Serial.println();

  // Compass

  printString(F("Compass"));
  printLabelFloat(F("Heading"), message.heading / 10.);
  printLabelFloat(F("pitch"), message.pitch / 10.);
  printLabelFloat(F("roll"), message.roll / 10.);
  Serial.println();

  // Barometer

  printString(F("Barometer"));
  printLabelFloat(F("Pressure"), message.airPressure / 10.0);
  Serial.println();

  // 1-Wire bus A

  printString(F("1-Wire A"));
  printLabelFloat(F("Air temp"), float(message.airTemp) * 0.0625);
  Serial.println();

  // Maxbotix

  printString(F("Maxbotix"));
  Serial.println(message.snowDist);

  // Airmar

  printString(F("Airmar"));
  printLabelFloat(F("Depth"), message.waterDepth / 100.);
  printLabelFloat(F("Temp"), message.waterTemp / 100.);
  Serial.println();

  // 1-Wire bus B (TAC)

  printString(F("1-Wire B"));

  for (int k = 0; k < TAC_BYTES; k++) {
    /*printFloat(message.TAC[k]);
      if (k%8) {
        Serial.print(' ');
      } else {
        Serial.println();
        printString(F(" "));
      }
    */
    if ((k % 8) == 0) {
      Serial.println();
    }
    printFloat(message.TAC[k]);
  }
  Serial.println();

  // Iridium

  printString(F("Iridium"));
  printLabelInt(F("SQF"), message.iridiumSignal);
  printLabelInt(F("Count"), iridiumCount);
  printLabelInt(F("Bytes"), sizeof(message));

  Serial.print(F("Result: "));
  if (iridiumError) {
    Serial.print(iridiumError);
  } else {
    Serial.print(F("OK"));
  }
  Serial.println();

}

int16_t readDS18B20 (DS2482& oneWire, byte* addr)
{
  byte data[9];

  oneWire.wireReset();
  oneWire.wireSelect(addr);
  oneWire.wireWriteByte(0x44); // Start conversion w/ parasite power

  delay(1000);

  oneWire.wireReset();
  oneWire.wireSelect(addr);
  oneWire.wireWriteByte(0xBE); // Read scratchpad

  for (int k = 0; k < 9; k++) {
    data[k] = oneWire.wireReadByte();
  }

  return (data[1] << 8) + data[0];
}

void readAirTemp()
{
  byte addr[8];

  oneWireA.wireResetSearch();
  delay(250);
  oneWireA.wireSearch(addr);

  message.airTemp = readDS18B20(oneWireA, addr);
}

void readBarometer()
{
  boolean bmeError;
  int16_t hPa;

  unsigned long startTime = millis();
  while (millis() < startTime + 5000) { //Give it 5 seconds just in case
    if (bme.begin()) {
      bmeError = false;
      break;
    }
  }

  delay(200);

  if (!bmeError) {
    hPa = floatToInt(bme.readPressure() / 100.0F, 10);
    message.airPressure = (uint16_t)hPa;
  } else {
    message.airPressure = 0xFFFF;
  }
}

boolean readMaxbotix()
{
  uint32_t timeout = millis() + 5000;
  boolean done = false;
  message.snowDist = 0xBEE5;

  maxbotix.begin(9600);
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  maxbotix.reset();

  while (!done && (millis() < timeout)) {
    maxbotix.read();
    done = maxbotix.newLineReceived() && maxbotix.parse(maxbotix.lastLine());
    if(maxbotix.distance == 0){
      done = false;
    }
  }

  message.snowDist = maxbotix.distance;

  return done;
}

boolean readAirmar()
{
  uint32_t timeout = millis() + 20000;
  boolean done = false;

  airmar.begin(4800);
  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  airmar.reset();

  while (!done && (millis() < timeout)) {
    airmar.read();
    //Serial.println(airmar.lastNMEA());
    done = airmar.newNMEAreceived() && airmar.parse(airmar.lastNMEA()) && airmar.hasFix();
  }

  message.waterDepth = floatToInt(airmar.waterDepth, 100);
  message.waterTemp = floatToInt(airmar.waterTemp, 100);

  Serial.println(message.waterDepth);

  return done;
}

void alarmMatch() // Do something when interrupt called
{

}

float displayPowerMonitor()
{
  float vin = powermon.read12(LTC2945_VIN_MSB_REG) * 0.025;
  float ain = powermon.read12(LTC2945_DELTA_SENSE_MSB_REG) * 25 / NUBC_POWERMON_RESISTOR / 1000;
  float pwr = vin * ain;

  message.batteryVoltage = floatToInt(vin, 100);

#if DEBUG
  printFloatUnits(vin, F("V"));
  printFloatUnits(ain, F("mA"));
  //printFloatUnits(pwr,F("mW"));
  //Serial.println();
#endif

  return pwr;
}

void showElapsed(FlashString msg)                                           //---- showElapsed function ----//
{
  uint32_t now = millis();                                                // Store the time in milliseconds since power up
  uint32_t deltaTime = now - prevTime;                                    // define time difference
  prevTime = now;                                                         // save the time to be used next time the function is called

#if DEBUG                                                                   //---- only executes if debug is set ----//
  printFloatUnits(float(now - startTime) / 1000, F("s"));
  printFloatUnits(float(deltaTime) / 1000, F("s"));
  printString(msg);
#endif                              a                                        //---------------------------------------//

  float mW = displayPowerMonitor();                                       // Determines current draw
  float mWh = deltaTime / 1000.0 / 3600 * mW;                             // Calculates milliWatt hours
  accPower += mWh;                                                        // Stores a running tally of total power used so far

#if DEBUG                                                                   //---- only executes if debug is set ----//
  printFloatUnits(mWh, F("mWh"));
  Serial.println();
#endif                                                                      //---------------------------------------//
}

void resetWatchdog() {
  //Pets the WDT and keeps the program alive. If the WDT trips the chip will experience a hard reset.
  digitalWrite(NUBC_WDT_RESET, HIGH);
  delay(20);
  digitalWrite(NUBC_WDT_RESET, LOW);
}

void buoySleep() {
  rtc.setAlarmSeconds(0); // RTC time to wake, currently seconds only
  rtc.enableAlarm(rtc.MATCH_SS); // Match seconds only, wakes every minute
  rtc.attachInterrupt(alarmMatch); // Attaches function to be called, currently blank
  Serial.print(F("Sleep until..."));
  delay(20); // Brief delay prior to sleeping not really sure its required
  
  Serial.end();
  USBDevice.detach();
  rtc.standbyMode();    // Sleep until next alarm match  
  postSleepRoutine();
}

void postSleepRoutine() {
  sleepCycleCount++;
  USBDevice.attach();
  serialConnect();
  resetWatchdog(); //Pet the watchdog
  Serial.println(F("Just woke up."));
  if (rtc.getMinutes() == 0 || sleepCycleCount > 65) {
    return;  
  } else {
    Serial.println(F("Sleeping some more."));
    buoySleep();
  }
}

void serialConnect() {
#ifdef ECHO_TO_SERIAL
  delay(1000);
  Serial.begin(115200);
  while (! Serial); // Wait until Serial is ready
  Serial.println("\r\nFeather M0 LowPower Logger\r\n");
#endif
}
