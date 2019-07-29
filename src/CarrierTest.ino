/*
 * Project CarrierTest
 * Description: Tests the Electron Carrier
 * Author: Charles McClelland
 * Date: Started 1-4-2017 

 // Easy place to change global numbers
 //These defines let me change the memory map and configuration without hunting through the whole program
 #define VERSIONNUMBER 7             // Increment this number each time the memory map is changed
 #define WORDSIZE 8                  // For the Word size
 #define PAGESIZE 4096               // Memory size in bytes / word size - 256kb FRAM
 #define VERSIONADDR 0x0             // Memory Locations By Name not Number
 #define TESTPASSEDADDR 0x1          // To keep from running the test over and again

 // The SparkFun MMA8452 breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
#define SA0 1
#if SA0
#define MMA8452_ADDRESS 0x1D  // SA0 is high, 0x1C if low
#else
#define MMA8452_ADDRESS 0x1C
#endif

const char releaseNumber[6] = "0.21";               // Displays the release on the menu ****  this is not a production release ****


/*	MMA8452Q-Serial_Example.ino
	Jim Lindblom <jim@sparkfun.com>
	August 31, 2015
	https://github.com/sparkfun/SparkFun_MMA8452Q_Particle_Library

	This is a simple example sketch for the SparkFun MMA8452Q
	Particle library. It'll connect to an MMA8452Q and stream the
	values out the serial port as the become available.
	Development environment specifics:
	Particle Build environment (https://www.particle.io/build)
	Particle Photon
	Distributed as-is; no warranty is given.
*/
// Include the library:
#include "SparkFunMMA8452Q/SparkFunMMA8452Q.h"

// Create an MMA8452Q object, used throughout the rest of the sketch.
MMA8452Q accel; // Default constructor, SA0 pin is HIGH

 // Included Libraries
 #include "Adafruit_FRAM_I2C.h"                           // Library for FRAM functions
 #include "FRAM-Library-Extensions.h"                     // Extends the FRAM Library
 #include "electrondoc.h"                                 // Documents pinout
 #include "MMA8452-Functions.h"                           // Adds the accelerometer functions


 // Prototypes and System Mode calls
 SYSTEM_THREAD(ENABLED);         // Means my code will not be held up by Particle processes.
 FuelGauge batteryMonitor;       // Prototype for the fuel gauge (included in Particle core library)


 // Pin Constants for Electron

 #if PLATFORM_ID==10                              // Electron
 #define WIRING_int2Pin D2                       // Acclerometer interrupt pin
 #define WIRING_blueLED  D7                     // This LED is on the Electron itself
 #define WIRING_userSwitch  D5                  // User switch with a pull-up resistor
 #define WIRING_tmp36Pin  A0                    // Simple Analog temperature sensor
 #define WIRING_tmp36Shutdwn  B5                // Can turn off the TMP-36 to save energy
 #define WIRING_donePin  D6                     // Pin the Electron uses to "pet" the watchdog
 #define WIRING_wakeUpPin  A7                   // This is the Particle Electron WKP pin
 #define WIRING_hardResetPin  D4                // Power Cycles the Electron and the Carrier Board
 #endif 

 #if PLATFORM_ID==12 || PLATFORM_ID == 13         // Boron or Argon
 #define WIRING_int2Pin  D2                     // Acclerometer interrupt pin
 #define WIRING_blueLED  D7                     // This LED is on the Electron itself
 #define WIRING_userSwitch  D4                  // User switch with a pull-up resistor
 #define WIRING_tmp36Pin  A4                    // Simple Analog temperature sensor
 #define WIRING_tmp36Shutdwn  B5                // Can turn off the TMP-36 to save energy
 #define WIRING_donePin  A3                     // Pin the Electron uses to "pet" the watchdog
 #define WIRING_wakeUpPin  D8                   // This is the Particle Electron WKP pin
 #define WIRING_hardResetPin  D6                // Power Cycles the Electron and the Carrier Board
 #endif


 // Program Variables
 int temperatureF;                           // Global variable so we can monitor via cloud variable
 volatile bool watchdogInterrupt = false;               // variable used to see if the watchdogInterrupt had fired
 unsigned long updateInterval = 60000;
 unsigned long lastUpdate;


 // Accelerometer Variables
const byte accelFullScaleRange = 2;  // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
const byte dataRate = 3;             // output data rate - 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
byte Sensitivity = 0;                    // Hex variable for sensitivity - Initialized in Setup (0 - most to 128 - least sensitive)
volatile bool sensorDetect = false;       // This is the flag that an interrupt is triggered

// Battery monitor
int stateOfCharge = 0;            // stores battery charge level value



// setup() runs once, when the device is first turned on.
void setup() {
  pinMode(WIRING_int2Pin,INPUT);                                          // PIR Sensor Interrupt pin
  pinMode(WIRING_userSwitch,INPUT);                                      // Button for user input
  pinMode(WIRING_wakeUpPin,INPUT);                                       // This pin is active HIGH
  pinMode(WIRING_blueLED, OUTPUT);                                       // declare the Blue LED Pin as an output
  pinMode(WIRING_tmp36Shutdwn,OUTPUT);                                   // Supports shutting down the TMP-36 to save juice
  digitalWrite(WIRING_tmp36Shutdwn, HIGH);                               // Turns on the temp sensor
  pinMode(WIRING_donePin,OUTPUT);                                        // Allows us to pet the watchdog
  digitalWrite(WIRING_donePin,HIGH);
  digitalWrite(WIRING_donePin,LOW);                                      // Pet the watchdog
  pinMode(WIRING_hardResetPin,OUTPUT);                                   // For a hard reset active HIGH

  Particle.variable("Release",releaseNumber);
  Particle.variable("stateOfChg", stateOfCharge);
  Particle.function("HardReset",hardResetNow);

  attachInterrupt(WIRING_wakeUpPin, watchdogISR, RISING);   // The watchdog timer will signal us and we have to respond

  if (!Particle.connected()) {                                     // Only going to connect if we are in connectionMode
    Particle.connect();
    waitFor(Particle.connected,90000);                             // 60 seconds then we timeout  -- *** need to add disconnected option and test
    Particle.process();
  }

  stateOfCharge = int(batteryMonitor.getSoC());             // Percentage of full charge
}


void loop() {
  Particle.publish("Test Start", "Beginning Test Run",PRIVATE);
  delay(1000);
  Particle.process();

  if (!fram.begin()) {                                                // You can stick the new i2c addr in here, e.g. begin(0x51);
    Particle.publish("Test #1", "Failed - Missing FRAM", PRIVATE);    // Can't communicate with FRAM - failed test
  }
  else if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {                 // Check to see if the memory map in the sketch matches the data on the chip
    Particle.publish("Test #1", "In process - Erasing FRAM", PRIVATE);
    ResetFRAM();                                                      // Reset the FRAM to correct the issue
  }
  delay(1000);
  Particle.process();

  if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) Particle.publish("Test #1", "Failed - FRAM Read Error", PRIVATE);
  else Particle.publish("Test #1", "FRAM Test Passed", PRIVATE);
  delay(1000);
  Particle.process();

  int tempValue = getTemperature();
  char data[64];                                                     // Store the date in this character array - not global
  snprintf(data, sizeof(data), "Temperature is %iF", tempValue);
  Particle.publish("Test #2", data, PRIVATE);
  delay(1000);
  Particle.process();

  Particle.publish("Test #3", "Press User Switch",PRIVATE);
  delay(1000);
  while(digitalRead(WIRING_userSwitch)) Particle.process();
  Particle.publish("Test #3", "User Switch Press Detected",PRIVATE);
  delay(1000);
  Particle.process();

  Particle.publish("Test #4", "Started testing Accelerometer - pls wait", PRIVATE);
  unsigned long waitForAccel = millis();
  // Initialize the accelerometer with begin():
  // begin can take two parameters: full-scale range, and output data rate (ODR).
  // Full-scale range can be: SCALE_2G, SCALE_4G, or SCALE_8G (2, 4, or 8g)
  // ODR can be: ODR_800, ODR_400, ODR_200, ODR_100, ODR_50, ODR_12, ODR_6 or ODR_1
  accel.begin(SCALE_2G, ODR_1); // Set up accel with +/-2g range, and slowest (1Hz) ODR
  do {
    if (accel.available())
    {
    // To update acceleration values from the accelerometer, call accel.read();
        accel.read();

    // After reading, six class variables are updated: x, y, z, cx, cy, and cz.
    // Those are the raw, 12-bit values (x, y, and z) and the calculated
    // acceleration's in units of g (cx, cy, and cz).
    Particle.process();
    }

  } while (millis() <= waitForAccel + 30000);  // Give it 30 seconds.
  snprintf(data, sizeof(data), "Acceleration Data is X: %2.1f, Y: %2.1f, Z: %2.1f", accel.cx, accel.cy, accel.cz);
  Particle.publish("Test #4", data, PRIVATE);
  delay(1000);
  Particle.process();

  do {
    if (millis() >= updateInterval + lastUpdate) {
      stateOfCharge = int(batteryMonitor.getSoC());             // Percentage of full charge
      snprintf(data, sizeof(data), "Battery charge level = %i", stateOfCharge);
      Particle.publish("Test #5", data, PRIVATE);
      Particle.process();
      lastUpdate = millis();
    }
  }  while(stateOfCharge <= 65);
  Particle.publish("Test #6", "Battery charge test passed", PRIVATE);


  time_t beginTime = Time.now();
  watchdogISR();
  watchdogInterrupt = false;

  if (Particle.connected()) Particle.publish("Test #7 Started","Expect this test to take ~60 minutes",PRIVATE);
  delay(1000);
  Particle.process();

  while(!watchdogInterrupt) {
    Particle.process();
    delay(1000);
  }

  int elapsedMinutes = (Time.now() - beginTime)/60;
  snprintf(data, sizeof(data), "Elapsed time in minutes is %i", elapsedMinutes);
  if (Particle.connected()) Particle.publish("Test #7 Finished", data ,PRIVATE);
  delay(1000);
  Particle.process();

  Particle.publish("Test #7", "Final Test - Hard Reset in 1 second",PRIVATE);
  delay(1000);
  Particle.process();

  digitalWrite(WIRING_hardResetPin,HIGH);                    // Zero the count so only every three

  Particle.publish("Test #7", "If you see this message - hard reset test failed", PRIVATE);
  BlinkForever();
}

int getTemperature()
{
  int reading = analogRead(WIRING_tmp36Pin);   //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;        // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                    // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));  //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  temperatureF = int((temperatureC * 9.0 / 5.0) + 32.0);  // now convert to Fahrenheit
  return temperatureF;
}

void watchdogISR()
{
  watchdogInterrupt = true;
  digitalWrite(WIRING_donePin, HIGH);                              // Pet the watchdog
  digitalWrite(WIRING_donePin, LOW);
}

void BlinkForever() {
  delay(1000);
  Particle.publish("Test Failed" "Reset Device to Continue", PRIVATE);
  while(1) {
    digitalWrite(WIRING_blueLED,HIGH);
    delay(2000);
    digitalWrite(WIRING_blueLED,LOW);
    delay(2000);
    Particle.process();
  }
}


int hardResetNow(String command)                                      // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    digitalWrite(WIRING_hardResetPin,HIGH);                                  // This will cut all power to the Electron AND the carrir board
    return 1;                                                         // Unfortunately, this will never be sent
  }
  else return 0;
}
