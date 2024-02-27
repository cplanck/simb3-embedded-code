////////////////////////////////////////////////////////////
// Development SIMB3 full code (version 4) with one-wire temperature string.
// Written 19 January 2023 by Cameron Planck

#define DEBUG 1
#include <Arduino.h>
#include <Wire.h>
#include <RTCZero.h>
#include <LineBuffer.h>
#include "wiring_private.h"  // pinPeripheral() function
#include <SPI.h>

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
#include "SIMB3_Onewire_Controller.h"

// Version ----------------------------------------------------------

#define PROGRAM_VERSION 8
#define SBD_RECORD_ID 0xA0
#define SBD_RECORD_VERSION 0x00
#define SBD_RECORD_HEADER (SBD_RECORD_ID | SBD_RECORD_VERSION)

// Program Options --------------------------------------------------

#define IRIDIUM_ENABLE false     // deactivate for testing
#define TRANSMISSION_INTERVAL 4  //1 = hourly, 4 = every 4 hours
#define IRIDIUM_ATTEMPTS 10
#define IRIDIUM_RETRY_DELAY 10000  //10 seconds

// Timeouts

#define GPS_TIMEOUT 6             //60 seconds
#define TEMPSTRING_ATTEMPTS      5
#define AIR_TEMP_ATTEMPTS        5

// Hardware
#define TOP_TEMP_STRING true
#define BOTTOM_TEMP_STRING true
#define INCIDENT_PYRANOMETER false
#define REFLECTED_PYRANOMETER false

// Pin mappings -----------------------------------------------------

#define SIMB3_MAXBOTIX_ENABLE A0
#define SIMB3_ENVELOPE_ENABLE A1
#define SIMB3_GPS_ENABLE 12
#define SIMB3_IRIDIUM_ENABLE 13
#define SIMB3_5V_ENABLE 5
#define SIMB3_12V_ENABLE 6

#define SIMB3_LED_G A3
#define SIMB3_CHIP_SELECT A4

#define SIMB3_WDT_RESET A1

// I2C Addresses ----------------------------------------------------

#define SIMB3_GYRO_ADDR 0x6A
#define SIMB3_MAG_ADDR 0x1E
#define SIMB3_POWERMON_ADDR 0x6F
#define SIMB3_BMP280 0x77
#define CIBC_ADS 0x48

#define SIMB3_POWERMON_RESISTOR 0.02
#define TAC_BYTES 120

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

Uart Serial2(&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler() {
  Serial2.IrqHandler();
}

LTC2945 powermon(SIMB3_POWERMON_ADDR, SIMB3_POWERMON_RESISTOR);
DS2482 oneWireA(0);
DS2482 oneWireB(1);

IridiumSBD iridium(Serial1, SIMB3_IRIDIUM_ENABLE);
Adafruit_GPS GPS(&Serial1);
RTCZero rtc;  // Create RTC object
BruncinDTC dtc(&Serial1);
AirmarSS510 airmar(Serial2);
Maxbotix maxbotix(Serial2);
Adafruit_BME280 bme;  // I2C
Adafruit_ADS1115 ADS1115;  //ADC + PGA for Apogee Pyranometers


// One-Wire Controller Variables -----------------------------------

SIMB3_Onewire_Controller SIMB3_ONEWIRE_CONTROLLER;

byte topTempStringBuffer[160];
byte topTempStringPackedBuffer[120];
byte bottomTempStringBuffer[160];
byte bottomTempStringPackedBuffer[120];
byte airTemperatureBuffer[2];

// General Variables -----------------------------------------------

boolean errorState;

uint32_t startTime;
uint32_t prevTime;
float accPower;

int iridiumSignal;
uint8_t iridiumCount;
int iridiumError;
int NextAlarmHour;

const byte hours = 13;
const byte minutes = 49;
const byte seconds = 33;

const byte DAY = 15;
const byte MONTH = 11;
const byte YEAR = 18;

int iSQ;

char fileName[13];

int sleepCycleCount;
uint8_t runCounter;

// SBD message format -----------------------------------------------

typedef union {

  struct {
    uint8_t     wdtCounter;
    uint8_t     programVersion;
    int32_t     timestamp;
    int32_t     latitude;
    int32_t     longitude;
    byte        airTemp[2];
    uint16_t    airPressure;
    uint16_t    waterDepth;
    int16_t     waterTemp;
    uint16_t    snowDist;

    #if INCIDENT_PYRANOMETER
    int16_t     incident;
    #endif
    #if REFLECTED_PYRANOMETER
    int16_t     reflected;
    #endif

    uint16_t    batteryVoltage;
    uint8_t     gpsSatellites;
    uint8_t     iridiumSignal;
    uint8_t     iridiumRetries;

    #if TOP_TEMP_STRING
    byte        topTempStringMessage[120];
    #endif
    #if BOTTOM_TEMP_STRING
    byte        bottomTempStringMessage[120];
    #endif

  } __attribute__((packed));

  uint8_t bytes[0];

} SBDMessage;

SBDMessage message;

//-- Function for printing ---------------------------------------

void printDigits(int digits, char sep = ' ') {
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
  Serial.print(sep);
}

void printFloat(float value) {
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

void printFloatUnits(float value, FlashString units) {
  printFloat(value);
  Serial.print(' ');
  Serial.print(units);
  Serial.print(F("  "));
}


void printLabelFloat(FlashString msg, float value, FlashString units) {
  Serial.print(msg);
  Serial.print(": ");
  Serial.print(value, 2);
  Serial.print(' ');
  Serial.print(units);
  Serial.print("  ");
}

void printLabelFloat(FlashString msg, float value) {
  printLabelFloat(msg, value, F(""));
}

void printLabelInt(FlashString msg, int value) {
  Serial.print(msg);
  Serial.print(": ");
  Serial.print(value);
  Serial.print(" ");
}

void printLabelInt(FlashString msg, unsigned int value) {
  Serial.print(msg);
  Serial.print(": ");
  Serial.print(value);
  Serial.print(" ");
}

void printString(FlashString msg, int width = 12) {
  size_t k = Serial.print(msg);

  for (; k < width; k++) {
    Serial.print(' ');
  }
}

void printSeparator(char c, uint8_t len = 75) {
  for (int k = 0; k < len; k++) {
    Serial.print(c);
  }
  Serial.println();
}

void displayBanner(char* msg, char top = '=', char bottom = '-') {
  printSeparator(top);
  Serial.println(msg);
  printSeparator(bottom);
}

//-- SBD Message ----------------------------------------------------

void clearMessage() {

  memset(message.bytes, 0, sizeof(message));

  message.wdtCounter = runCounter;
  message.programVersion = PROGRAM_VERSION;
  message.timestamp = rtc.getEpoch();
}

int16_t floatToInt(float value, float scale) {
  if (value < 0) {
    return int16_t(value * scale - 0.5);
  } else {
    return int16_t(value * scale + 0.5);
  }
}

//-- Setup Loop ----------------------------------------------------

void setup() {
  rtc.begin();                           // Start the RTC in 24hr mode
  rtc.setTime(hours, minutes, seconds);  // Set the time
  rtc.setDate(DAY, MONTH, YEAR);         // Set the date

  Wire.begin();  // Join the I2C bus as master

  configureIO();
  resetWatchdog();  //Pet the watchdog

  Serial.println();
  Serial.print(F("CRYOSPHERE INNOVATION SIMB3 Ver "));
  Serial.print(PROGRAM_VERSION);
  Serial.print(F(" ... "));
  Serial.println(F("Ready"));


  if(INCIDENT_PYRANOMETER || REFLECTED_PYRANOMETER){
    ADS1115.setGain(GAIN_EIGHT);
    ADS1115.begin();
  }

  // flash LED to indicated transmission interval
  if (TRANSMISSION_INTERVAL == 1) {
    digitalWrite(SIMB3_LED_G, HIGH);
    delay(500);
    digitalWrite(SIMB3_LED_G, LOW);
    delay(500);
  }
  if (TRANSMISSION_INTERVAL == 4) {
    for (int i = 0; i < 4; i++) {
      digitalWrite(SIMB3_LED_G, HIGH);
      delay(500);
      digitalWrite(SIMB3_LED_G, LOW);
      delay(500);
    }
  }
  digitalWrite(SIMB3_LED_G, HIGH);
}


//== Main Loop (this runs iteratively) ===============================

void loop() {
  resetWatchdog();  //Pet the watchdog
  startTime = prevTime = millis();
  accPower = 0;
  runCounter++;

  Serial.println(F("Beginning Loop()..."));
  displayBanner("Start Sampling", '=', '-');

  // Clear SBD message and turn on power monitor
  clearMessage();
  powermon.wakeup();

  digitalWrite(SIMB3_5V_ENABLE, HIGH);
  digitalWrite(SIMB3_MAXBOTIX_ENABLE, HIGH);
  digitalWrite(SIMB3_GPS_ENABLE, HIGH);

  // Read Barometers
  Serial.println(F("Reading Barometer..."));
  readBarometer();
  showElapsed(F("BMP280 Barometer"));
  resetWatchdog();  //Pet the watchdog

  // Read GPS
  Serial.println(F("Reading GPS..."));
  GPS.begin(9600);
  configureGPS();
  if (readGPS()) {
    rtc.setTime(GPS.hour, GPS.minute, GPS.seconds);  // Set the time
    rtc.setDate(GPS.day, GPS.month, GPS.year);       // Set the date
    message.timestamp = rtc.getEpoch();              //Second place this gets set, just in case.
  }
  showElapsed(F("GPS"));
  Serial.println(F("GPS Read...Turning off GPS..."));
  digitalWrite(SIMB3_GPS_ENABLE, LOW);
  resetWatchdog();  //Pet the watchdog

  digitalWrite(SIMB3_12V_ENABLE, LOW);

  // Read Battery Voltage
  readBatteryVoltage();
  
  // powermon.shutdown();
  delay(1000);

  // Read One-Wire Controller
  readOnewireController();
  showElapsed(F("One-wire Controller"));

  // Read Incident Apogee
  #if INCIDENT_PYRANOMETER
    Serial.println(F("Reading Incident Pyranometer..."));
    readIncidentPyranometer();
    showElapsed(F("Apogee Incident Pyranometer"));
  #endif

  // Read Reflected Apogee
  #if REFLECTED_PYRANOMETER
    Serial.println(F("Reading Reflected Pyranometer..."));
    readReflectedPyranometer();
    showElapsed(F("Apogee Reflected Pyranometer"));
  #endif

  // Read Maxbotix
  Serial.println(F("Reading Snow Depth..."));
  readMaxbotix();
  showElapsed(F("Maxbotix"));
  digitalWrite(SIMB3_MAXBOTIX_ENABLE, LOW);

  // Read Airmar
  Serial.println(F("Reading Airmar..."));
  readAirmar();
  showElapsed(F("Airmar"));
  digitalWrite(SIMB3_12V_ENABLE, HIGH);

  displayMessage();

  // Transmit over Iridium
  if (IRIDIUM_ENABLE) {
    Serial.println(F("\nTransmitting on Iridium..."));
    iridiumOn();
    Serial.println(F("Iridium is on."));
    resetWatchdog();  //Pet the watchdog
    sendIridium();
    iridiumOff();
    showElapsed(F("Iridium"));
    Serial.println(F("Done transmitting..."));
    Serial.print(F("Iridium status: "));
    Serial.println(iridiumError);
    if (iridiumError == 0) {
      digitalWrite(SIMB3_LED_G, LOW);
    }
  }

  // Turn off power rails
  digitalWrite(SIMB3_12V_ENABLE, HIGH);
  digitalWrite(SIMB3_5V_ENABLE, LOW);

  // Shut down power monitor
  powermon.shutdown();

#if DEBUG
  printSeparator('-');
  Serial.print(F("Integrated power: "));
  printFloatUnits(accPower, F("mWH"));
  Serial.println();
  printSeparator('-');
  displayBanner("Going to sleep", '-', '=');
#endif

  // Put buoy to sleep
  sleepCycleCount = 0;
  buoySleep();
};

//== End Main Loop ==============================================

//-- Operational functions --------------------------------------
void configureIO() {
  pinMode(SIMB3_MAXBOTIX_ENABLE, OUTPUT);
  pinMode(SIMB3_GPS_ENABLE, OUTPUT);
  pinMode(SIMB3_IRIDIUM_ENABLE, OUTPUT);
  pinMode(SIMB3_5V_ENABLE, OUTPUT);
  pinMode(SIMB3_12V_ENABLE, OUTPUT);
  pinMode(SIMB3_LED_G, OUTPUT);
  pinMode(SIMB3_WDT_RESET, OUTPUT);

  digitalWrite(SIMB3_MAXBOTIX_ENABLE, LOW);
  digitalWrite(SIMB3_GPS_ENABLE, LOW);
  digitalWrite(SIMB3_IRIDIUM_ENABLE, LOW);
  digitalWrite(SIMB3_5V_ENABLE, LOW);
  digitalWrite(SIMB3_12V_ENABLE, HIGH);
  digitalWrite(SIMB3_LED_G, LOW);
  digitalWrite(SIMB3_WDT_RESET, LOW);

  detachInterrupt(0);
  detachInterrupt(1);

  Serial.print(F("32"));
  // powermon.wakeup();
  Serial.print(F("32"));
  // powermon.write(LTC2945_CONTROL_REG, LTC2945_SENSE_MONITOR);

}

void configureIridium() {
  Serial.println(F("Configuring the iridium"));
  iridium.attachConsole(Serial);
  iridium.attachDiags(Serial);
  Serial.println(F("after iridium begin"));
  iridium.setPowerProfile(1);
  Serial.println(F("after set power profile"));
  iridium.useMSSTMWorkaround(false);
}

void iridiumOn() {
  digitalWrite(SIMB3_IRIDIUM_ENABLE, HIGH);
  Serial.println(F("Turned on the iridium"));
  Serial1.begin(19200);
  configureIridium();
  Serial.println(F("configured the iridium"));
  iridium.isAsleep();
  iridium.begin();
}

void iridiumOff() {
  iridium.sleep();
  digitalWrite(SIMB3_IRIDIUM_ENABLE, LOW);
}

void sendIridium() {
  iridiumSignal = -1;
  iridiumCount = 0;
  iridiumError = -1;

  iridium.getSignalQuality(iridiumSignal);
  message.iridiumSignal = iridiumSignal;

  iridiumError = iridium.sendSBDBinary(message.bytes, sizeof(message));  //actually transmit
}

void configureGPS() {
  digitalWrite(SIMB3_GPS_ENABLE, HIGH);

  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.reset();
}

boolean readGPS() {
  uint32_t timeout = millis() + GPS_TIMEOUT;
  bool found = false;

  GPS.reset();

  while ((millis() < timeout) && !found) {
    GPS.read();
    found = GPS.newNMEAreceived() && GPS.parse(GPS.lastNMEA()) && GPS.fix && GPS.satellites > 0;
  }

  message.latitude = int32_t(GPS.latitude * 1000000);
  message.longitude = int32_t(GPS.longitude * 1000000);
  message.gpsSatellites = GPS.satellites;

  return found;
}

void displayMessage() {
  Serial.println("\n---------------------- SBD Message Preview ----------------------\n");

  Serial.print(F(" Software Version: "));
  Serial.println(message.programVersion);

  // Timestamp
  printString(F("Timestamp"));
  Serial.println(message.timestamp);

  // WD Timer
  printString(F("Watch Dog Timer Counter "));
  Serial.println(message.wdtCounter);

  // Voltage
  printString(F("Battery Voltage "));
  Serial.println(message.batteryVoltage/100.0);

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

  // Barometer
  printString(F("Barometer"));
  printLabelFloat(F("Pressure"), message.airPressure / 10.0);
  Serial.println();

  // Air Temp
  printString(F("Air Temp"));
  printLabelFloat(F("Deg C"), SIMB3_ONEWIRE_CONTROLLER.decodeTemperatureBytes(message.airTemp[0], message.airTemp[1]));
  Serial.println();

  #if INCIDENT_PYRANOMETER
    float incident_multiplier = 0.0156F; /* ADS1115  @ +/- 2.048V gain (16-bit results) */
    printString(F("Incident Sunlight"));
    printLabelFloat(F(""), message.incident * incident_multiplier * 22);
    Serial.println();
  #endif

  #if REFLECTED_PYRANOMETER
    float reflected_multiplier = 0.0156F; /* ADS1115  @ +/- 2.048V gain (16-bit results) */
    printString(F("Reflected Sunlight"));
    printLabelFloat(F(""), message.reflected * reflected_multiplier * 28.5);
    Serial.println();
  #endif

  // Maxbotix
  printString(F("Maxbotix"));
  Serial.println(message.snowDist);

  // Airmar
  printString(F("Airmar"));
  printLabelFloat(F("Depth"), message.waterDepth / 100.);
  printLabelFloat(F("Temp"), message.waterTemp / 100.);
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
  Serial.println("\n");

  // One-Wire Controller
  Serial.println(F("One-Wire Temperature String (Deg C)"));
  SIMB3_ONEWIRE_CONTROLLER.printTemperatureString(1, topTempStringBuffer);
  Serial.println(F(""));
  SIMB3_ONEWIRE_CONTROLLER.printTemperatureString(2, bottomTempStringBuffer);
  Serial.println(F(""));
  SIMB3_ONEWIRE_CONTROLLER.printAirTemperature(airTemperatureBuffer);
}

void readBarometer() {
  boolean bmeError;
  int16_t hPa;

  unsigned long startTime = millis();
  while (millis() < startTime + 5000) {  //Give it 5 seconds just in case
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

void readOnewireController(){

  // Read top temperature string
  #if TOP_TEMP_STRING
    bool done = false;
    int reattempts = 0;
    while(!done && reattempts < TEMPSTRING_ATTEMPTS){
    done = SIMB3_ONEWIRE_CONTROLLER.readTopString();
    if(!done){
      delay(2000);
      reattempts++;
      // message.aux1 = reattempts;
    }
    }
    delay(6000);
    SIMB3_ONEWIRE_CONTROLLER.requestTemperatureStringBytes(1, topTempStringBuffer);
    SIMB3_ONEWIRE_CONTROLLER.packTemperaturesForTransmission(80, topTempStringBuffer, topTempStringPackedBuffer);
    memcpy(message.topTempStringMessage, topTempStringPackedBuffer, 120);
  #endif

  #if BOTTOM_TEMP_STRING
    // Read bottom temperature string
    done = false;
    reattempts = 0;
    while(!done && reattempts < TEMPSTRING_ATTEMPTS){
    done = SIMB3_ONEWIRE_CONTROLLER.readBottomString();
    if(!done){
      delay(2000);
      reattempts++;
      // message.aux2 = reattempts;
    }
    }
    delay(6000);
    SIMB3_ONEWIRE_CONTROLLER.requestTemperatureStringBytes(2, bottomTempStringBuffer);
    SIMB3_ONEWIRE_CONTROLLER.packTemperaturesForTransmission(80, bottomTempStringBuffer, bottomTempStringPackedBuffer);
    memcpy(message.bottomTempStringMessage, bottomTempStringPackedBuffer, 120);
  #endif
  
  // Read air temperature
  done = false;
  reattempts = 0;
  while(!done && reattempts < AIR_TEMP_ATTEMPTS){
  done = SIMB3_ONEWIRE_CONTROLLER.readAirTemperature();
  if(!done){
     delay(2000);
     reattempts++;
    //  message.aux3 = reattempts;
   }
  }

  delay(2000);
  SIMB3_ONEWIRE_CONTROLLER.requestAirTempBytes(airTemperatureBuffer);
  memcpy(message.airTemp, airTemperatureBuffer, 2);
  
}

boolean readMaxbotix() {
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
    if (maxbotix.distance == 0) {
      done = false;
    }
  }

  message.snowDist = maxbotix.distance;

  return done;
}

boolean readAirmar() {
  uint32_t timeout = millis() + 20000;
  boolean done = false;

  airmar.begin(4800);
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  airmar.reset();

  while (!done && (millis() < timeout)) {
    airmar.read();
    done = airmar.newNMEAreceived() && airmar.parse(airmar.lastNMEA()) && airmar.hasFix();
  }

  message.waterDepth = floatToInt(airmar.waterDepth, 100);
  message.waterTemp = floatToInt(airmar.waterTemp, 100);

  return done;
}

#if INCIDENT_PYRANOMETER
void readIncidentPyranometer()
{
  int16_t results510;
  results510 = ADS1115.readADC_Differential_0_1();
  message.incident = results510;
}
#endif

#if REFLECTED_PYRANOMETER
void readReflectedPyranometer()
{
  int16_t results610;
  results610 = ADS1115.readADC_Differential_2_3();
  message.reflected = results610;
}
#endif

void alarmMatch() // Do something when interrupt called
{

}

void readBatteryVoltage() {
  float vin = powermon.read12(LTC2945_VIN_MSB_REG) * 0.025;
  float ain = powermon.read12(LTC2945_DELTA_SENSE_MSB_REG) * 25 / SIMB3_POWERMON_RESISTOR / 1000;
  float pwr = vin * ain;

  message.batteryVoltage = floatToInt(vin, 100);
}

float displayPowerMonitor() {
  float vin = powermon.read12(LTC2945_VIN_MSB_REG) * 0.025;
  float ain = powermon.read12(LTC2945_DELTA_SENSE_MSB_REG) * 25 / SIMB3_POWERMON_RESISTOR / 1000;
  float pwr = vin * ain;

  // message.batteryVoltage = floatToInt(vin, 100);

#if DEBUG
  printFloatUnits(vin, F("V"));
  printFloatUnits(ain, F("mA"));
#endif

  return pwr;
}

void showElapsed(FlashString msg)  //---- showElapsed function ----//
{
  uint32_t now = millis();              // Store the time in milliseconds since power up
  uint32_t deltaTime = now - prevTime;  // define time difference
  prevTime = now;                       // save the time to be used next time the function is called

#if DEBUG  //---- only executes if debug is set ----//
  printFloatUnits(float(now - startTime) / 1000, F("s"));
  printFloatUnits(float(deltaTime) / 1000, F("s"));
  printString(msg);
#endif  //---------------------------------------//

  float mW = displayPowerMonitor();            // Determines current draw
  float mWh = deltaTime / 1000.0 / 3600 * mW;  // Calculates milliWatt hours
  accPower += mWh;                             // Stores a running tally of total power used so far

#if DEBUG  //---- only executes if debug is set ----//
  printFloatUnits(mWh, F("mWh"));
  Serial.println();
#endif  //---------------------------------------//
}

void resetWatchdog() {
  //Pets the WDT and keeps the program alive. If the WDT trips the chip will experience a hard reset.
  digitalWrite(SIMB3_WDT_RESET, HIGH);
  delay(20);
  digitalWrite(SIMB3_WDT_RESET, LOW);
}

void buoySleep() {
  rtc.setAlarmSeconds(0);           // RTC time to wake, currently seconds only
  rtc.enableAlarm(rtc.MATCH_SS);    // Match seconds only, wakes every minute
  rtc.attachInterrupt(alarmMatch);  // Attaches function to be called, currently blank
  Serial.print(F("Sleep until..."));
  delay(20);  // Brief delay prior to sleeping not really sure its required

  Serial.end();
  USBDevice.detach();
  rtc.standbyMode();  // Sleep until next alarm match
  postSleepRoutine();
}

void postSleepRoutine() {
  sleepCycleCount++;
  USBDevice.attach();
  resetWatchdog();  //Pet the watchdog
  Serial.println(F("Just woke up."));

  if (TRANSMISSION_INTERVAL == 1) {
    if (rtc.getMinutes() == 0 || sleepCycleCount > 65) {
      return;
    } else {
      Serial.println(F("Sleeping some more."));
      buoySleep();
    }
  } else if (TRANSMISSION_INTERVAL == 4) {
    if ((rtc.getMinutes() == 0 && (rtc.getHours() == 0 || rtc.getHours() == 4 || rtc.getHours() == 8 || rtc.getHours() == 12 || rtc.getHours() == 16 || rtc.getHours() == 20)) || sleepCycleCount > 260) {
      return;
    } else {
      Serial.println(F("Sleeping some more."));
      buoySleep();
    }
  }
}