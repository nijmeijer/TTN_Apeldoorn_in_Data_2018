/*******************************************************************************
   Copyright (c) 2016 Thomas Telkamp, Matthijs Kooijman, Bas Peschier, Harmen Zijp

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.
 *******************************************************************************/

// include external libraries
#include <avr/eeprom.h>

// set run mode
boolean const DEBUG = true;

/// Configure LoRaWAN personalisation parameters.

// This EUI must be in little-endian format, so least-significant-byte
// first (LSB..MSB). When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
// The same for all Dust-sensor-nodes
static const uint8_t APPEUI[8] =  { 0xA3, 0x42, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };

// This should also be in little endian format, see above (LSB..MSB)
static const uint8_t DEVEUI[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// This should also be in big endian format (MSB..LSB) // to be filled in on a per-node basis
static const uint8_t APPKEY[16] = { 0x.., 0x.., 0x.., 0x.., 0x.., 0x.., 0x50, 0x06, 0x71, 0xE0, 0xA3, 0x84, 0x30, 0x88, 0xAA, 0x96 }; // AID Dust Sensors 2018


#define EEPROM_LAYOUT_MAGIC_OLD 0x2a60af86 // Just a random number, stored little-endian
#define EEPROM_LAYOUT_MAGIC 0x2a60af87 // Just a random number, stored little-endian
#define EEPROM_LAYOUT_MAGIC_START 0x00 // 4 bytes
#define EEPROM_OSCCAL_START (EEPROM_LAYOUT_MAGIC_START + 4) // 1 byte
#define EEPROM_APP_EUI_START (EEPROM_OSCCAL_START + 1)
#define EEPROM_APP_EUI_LEN 8
#define EEPROM_DEV_EUI_START (EEPROM_APP_EUI_START + EEPROM_APP_EUI_LEN)
#define EEPROM_DEV_EUI_LEN 8
#define EEPROM_APP_KEY_START (EEPROM_DEV_EUI_START + EEPROM_DEV_EUI_LEN)
#define EEPROM_APP_KEY_LEN 16

typedef unsigned char byte;

void eeprom_getMagic (uint8_t* buf) {
  for (byte i = 0; i < 4; i++) {
    buf[i] = eeprom_read_byte((uint8_t*)i);
  }
}



void eeprom_getAppEui (uint8_t* buf) {
  for (byte i = 0; i < EEPROM_APP_EUI_LEN; i++) {
    buf[i] = eeprom_read_byte((uint8_t*)EEPROM_APP_EUI_START + EEPROM_APP_EUI_LEN - 1 - i);
  }
}

void eeprom_getDevEui (uint8_t* buf) {
  for (byte i = 0; i < EEPROM_DEV_EUI_LEN; i++) {
    buf[i] = eeprom_read_byte((uint8_t*)EEPROM_DEV_EUI_START + EEPROM_DEV_EUI_LEN - 1 - i);
  }
}

void eeprom_getAppKey (uint8_t* buf) {
  for (byte i = 0; i < EEPROM_APP_KEY_START; i++) {
    buf[i] = eeprom_read_byte((uint8_t*)EEPROM_APP_KEY_START + i);
  }
}

void eeprom_setAppEui (uint8_t* buf) {
  for (byte i = 0; i < EEPROM_APP_EUI_LEN; i++) {
    eeprom_write_byte((uint8_t*)EEPROM_APP_EUI_START + EEPROM_APP_EUI_LEN - 1 - i, buf[i]);
  }
}

void eeprom_setDevEui (uint8_t* buf) {
  for (byte i = 0; i < EEPROM_DEV_EUI_LEN; i++) {
    eeprom_write_byte((uint8_t*)EEPROM_DEV_EUI_START + EEPROM_DEV_EUI_LEN - 1 - i, buf[i]);
  }
}

void eeprom_setAppKey (uint8_t* buf) {
  for (byte i = 0; i < EEPROM_APP_KEY_START; i++) {
    eeprom_write_byte((uint8_t*)EEPROM_APP_KEY_START + i, buf[i]);
  }
}


void eeprom_setMagic (uint32_t buf) {
  uint32_t buf_i = buf;
  for (byte i = 0; i < 4; i++) {
      eeprom_write_byte((uint8_t*) i, buf_i & 0xFF);
      buf_i = buf_i >> 8;
  }
}



void printHex(const __FlashStringHelper *prefix, uint8_t *buf, size_t len) {
  Serial.print(prefix);
  for (size_t i = 0; i < len; ++i) {
    if (buf[i] < 0x10)
      Serial.write('0');
    Serial.print(buf[i], HEX);
    Serial.write(' ');
  }
  Serial.println();
}


void setup() 
{
  // start serial connection
  Serial.begin(9600);
  Serial.println(F("Start"));
}

void loop() 
{

   // Write magic
  eeprom_setMagic(EEPROM_LAYOUT_MAGIC_OLD);
  
  // Write AppEUI
  eeprom_setAppEui(APPEUI);
  // Write DevEUI
  eeprom_setDevEui(DEVEUI);
  // write AppKEY
  eeprom_setAppKey(APPKEY);
  
  uint32_t hash = eeprom_read_dword(0x00);
  if (hash != EEPROM_LAYOUT_MAGIC && hash != EEPROM_LAYOUT_MAGIC_OLD) 
  {
    Serial.println(F("EEPROM is not correctly configured"));

    while (true) /* nothing */;
 }
  
  uint8_t buf[EEPROM_APP_KEY_LEN];

  eeprom_getMagic(buf);
  printHex(F("Magic: "), buf, 4);
  eeprom_getAppEui(buf);
  printHex(F("App EUI (LSB-MSB): "), buf, EEPROM_APP_EUI_LEN);
  eeprom_getDevEui(buf);
  printHex(F("Dev EUI (LSB-MSB): "), buf, EEPROM_DEV_EUI_LEN);
  eeprom_getAppKey(buf);
  printHex(F("App Key (MSB-LSB): "), buf, EEPROM_APP_KEY_LEN);

  while(1)
  {

  }
}


