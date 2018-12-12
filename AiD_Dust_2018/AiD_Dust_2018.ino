/*******************************************************************************
   Copyright (c) 2016 Thomas Telkamp, Matthijs Kooijman, Bas Peschier, Harmen Zijp

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   In order to compile the following libraries need to be installed:
   - SparkFunHTU21D: https://github.com/sparkfun/SparkFun_HTU21D_Breakout_Arduino_Library
   - Adafruit_SleepyDog: https://github.com/adafruit/Adafruit_SleepyDog
   - lmic (mjs-specific fork): https://github.com/meetjestad/arduino-lmic
   - SDS011 by Rajko Zschiegner 

  8-12-2018: Adapted for TTN-Apeldoorn Dust Measurement
 *******************************************************************************/

// include external libraries          
#include <SparkFunHTU21D.h>          
#include <Adafruit_SleepyDog.h>      
#include <avr/power.h>
#include <util/atomic.h>
#include "SDS011.h"                  

// define various pins
#define rxPin  A3
#define txPin  A2

// setup timing variables
uint32_t const UPDATE_INTERVAL = 2180000; // Approx 1 measurement per minute. (for testing AiD 900000 => ~30 seconds interval)

uint32_t lastUpdateTime = 0;

// set run mode
#define DEBUG true

#include "mjs_lmic.h"
#include "PE1MEW_lmicCheck.h" /// Check if lmic is configured right

#define MEASUREMENTS        5

// setup temperature and humidity sensor
HTU21D htu;                                 /// Object for Humidity and temperatures sensor type HTU
float temperature = { 0.0 };                /// Variable to hold temperature (globally)
float humidity = { 0.0 };                   /// Variable to hold humidity (globally)

#define MAXIMUM_TEMPERATURE 50
#define MINIMUM_TEMPERATURE -20
#define MAXIMUM_HUMIDITY    100.1

// setup particle sensor
float pm2_5, pm10;
SDS011 mysds;

// Lora Payload Buffer
unsigned char mydata[20];
unsigned char mydata_size;

// Function Prototypes
unsigned char AiD_add_float (unsigned char idx_in, unsigned char type, float value);

/// \brief Arduino function called once for setup.
void setup() 
{
  // when in debugging mode start serial connection
  Serial.begin(115200);
  Serial.println(F("Start"));

  // Setup LoRaWAN transceiver
  mjs_lmic_setup();

  // start communication to sensors
  htu.begin();                              // Temperature and Humidity sensor
  mysds.begin(rxPin, txPin);                             // Software serial port for particle sensor

  delay(5000);
}

/// \brief Arduino function called as loop.
void loop() 
{
  // We need to calculate how long we should sleep, so we need to know how long we were awake
  unsigned long startMillis = millis();

  // Use moderate datarate for normal transmissions.
  LMIC_setDrTxpow(DR_SF9, 14);

  // Activate and read our sensors
  temperature = getTemperature(temperature);
  humidity = getHumidity(humidity);

  getParticle();
  
  dumpData();


  // Work around a race condition in LMIC, that is greatly amplified
  // if we sleep without calling runloop and then queue data
  // See https://github.com/lmic-lib/lmic/issues/3
  os_runloop_once();

  // We can now send the data
  queueData();

  mjs_lmic_wait_for_txcomplete();

  // Schedule sleep
  unsigned long msPast = millis() - startMillis;
  unsigned long sleepDuration = UPDATE_INTERVAL;
  if (msPast < sleepDuration)
  {
    sleepDuration -= msPast;
  }
  else
  {
    sleepDuration = 0;
  }
  
  // tell the world that we will go to sleep and how long.

  Serial.print(F("Sleeping: "));
  Serial.print(sleepDuration);
  Serial.println(F(" ms..."));
  Serial.flush();

  // Goto sleep.
  doSleep(sleepDuration);
  
  // tell the world that we woke-up.
  Serial.println(F("Woke up."));
}

void doSleep(uint32_t time) 
{

  while (time > 0) 
  {
    uint16_t slept;
    if (time < 8000)
    {
      slept = Watchdog.sleep(time);
    }
    else
    {
      slept = Watchdog.sleep(8000);
    }

    // Update the millis() and micros() counters, so duty cycle
    // calculations remain correct. This is a hack, fiddling with
    // Arduino's internal variables, which is needed until
    // https://github.com/arduino/Arduino/issues/5087 is fixed.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
    {
      extern volatile unsigned long timer0_millis;
      extern volatile unsigned long timer0_overflow_count;
      timer0_millis += slept;
      // timer0 uses a /64 prescaler and overflows every 256 timer ticks
      timer0_overflow_count += microsecondsToClockCycles((uint32_t)slept * 1000) / (64 * 256);
    }

    if (slept >= time)
    {
      break;
    }
    time -= slept;
  }

}

void dumpData() 
{
  Serial.println("tmp/hum: " + String(temperature) + "/" + String(humidity) + " C/%" );
  Serial.println("particles 2.5/10: " + String(pm2_5) + "/" + String(pm10) + " ug/m3" );
  Serial.flush();
}


void queueData() 
{
  // Compose AiD message
  mydata_size = 0;              // init
  mydata[mydata_size++] = 0xA1; // Apeldoorn In data
  mydata_size = AiD_add_float(mydata_size, 0xD1, pm2_5);
  mydata_size = AiD_add_float(mydata_size, 0xD2, pm10);
  mydata_size = AiD_add_float(mydata_size, 0xD4, humidity);
  mydata_size = AiD_add_float(mydata_size, 0xD3, temperature);
   
  // Prepare upstream data transmission at the next possible time.
  LMIC_setTxData2(1, &mydata[0], mydata_size, 0);
  Serial.println(F("Packet queued"));

}

/// \brief read temperature from sensor 
/// This function prevents bogous readings from sensors.
/// \param oldTemp Last read temperature value.
/// \return new temperature from sensor.
float getTemperature(float oldTemp)
{
  float newTemp = oldTemp;
  float tempTemp = 0.0;
  int i = 0;

  while( i < MEASUREMENTS)
  {
    tempTemp = htu.readTemperature();
    
    if((tempTemp < MAXIMUM_TEMPERATURE) && (tempTemp > MINIMUM_TEMPERATURE))
    {
      i = MEASUREMENTS;
      newTemp = tempTemp;
    }
    else
    {
      i++;
    }
  }
  return newTemp;
}

/// \brief read humidity from sensor 
/// This function prevents bogous readings from sensors.
/// \param oldHumid Last read humidity value.
/// \return new humidity from sensor.
float getHumidity(float oldHumid)
{
  float newHumid = oldHumid;
  float tempHumid = 0.0;
  int i = 0;

  while( i < MEASUREMENTS)
  {
    tempHumid = htu.readHumidity();
    
    if((tempHumid < MAXIMUM_HUMIDITY) && (tempHumid > 0.0))
    {
      i = MEASUREMENTS;
      newHumid = tempHumid;
    }
    else
    {
      i++;
    }
  }
  return newHumid;
}

void getParticle(void)
{
  int error;
  error = mysds.read(&pm2_5,&pm10);

  if (error) {
    Serial.println("Error reading from Dust sensor");
    pm2_5 = -1.0;
    pm10 = -1;
   }
}

unsigned char AiD_add_float (unsigned char idx_in, unsigned char type, float value) { 
  union {
     uint32_t a_uint;
     float a_float;
  } convert;
  
   convert.a_float = value;
  
   // mydata[idx_in++] = type;
   mydata[idx_in++] = (convert.a_uint>>24) & 0xFF;
   mydata[idx_in++] = (convert.a_uint>>16) & 0xFF;
   mydata[idx_in++] = (convert.a_uint>>8 ) & 0xFF;
   mydata[idx_in++] = (convert.a_uint>>0 ) & 0xFF;
   return (idx_in);
 }


