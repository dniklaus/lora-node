/*
 * wiring-skeleton.cpp
 *
 *  Created on: 15.03.2017
 *      Author: niklausd
 */

//#include <Arduino.h>
#include <wire.h>
#include <SPI.h>

// PlatformIO libraries
#include <SerialCommand.h>  // pio lib install 173, lib details see https://github.com/kroimon/Arduino-SerialCommand
#include <Timer.h>          // pio lib install 1699, lib details see https://github.com/dniklaus/wiring-timer

// public external libraries
#include <lmic.h>           // lib details see https://github.com/mikenz/LoRa-LMIC-1.51
#include <hal/hal.h>

// private libraries
#include <DbgCliNode.h>
#include <DbgCliTopic.h>
#include <DbgCliCommand.h>
#include <DbgTracePort.h>
#include <DbgTraceContext.h>
#include <DbgTraceOut.h>
#include <DbgPrintConsole.h>
#include <DbgTraceLevel.h>
#include <AppDebug.h>
#include <ProductDebug.h>
#include <RamUtils.h>

#ifndef BUILTIN_LED
#define BUILTIN_LED 13
#endif

SerialCommand* sCmd = 0;

//-----------------------------------------------------------------------------
// LMIC Wrapper stuff
//-----------------------------------------------------------------------------
// This EUI must be in little-endian format, so least-significant-byte
// ffirst. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0xC0, 0x69, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui(u1_t* buf)
{
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };
void os_getDevEui(u1_t* buf)
{
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0xBD, 0x3F, 0x4C, 0xED, 0xA4, 0x99, 0x75, 0x94, 0x57, 0xDA, 0x31, 0x06, 0x4B, 0x69, 0x70, 0x5D };
void os_getDevKey(u1_t* buf)
{
  memcpy_P(buf, APPKEY, 16);
}

// forward declaration
void do_send(osjob_t* j);

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins =
{
  .nss = 6,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,
  .dio = { 2, 3, 4 }
};

void onEvent(ev_t ev)
{
  Serial.print((s4_t)os_getTime());
  Serial.print(": ");
  switch (ev)
  {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen)
      {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL),
          do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

#if 0
// LoRaWan credentials (http://www.thethingsnetwork.org), Device EUI 1234567812345678
uint8_t NWKSKEY[] =
{ 0x62, 0x5B, 0xBE, 0x1A, 0x57, 0x09, 0x12, 0x25, 0xFA, 0x74, 0x5C, 0xB7, 0x1E, 0x62, 0x28, 0x5A};
uint8_t APPSKEY[] =
{ 0xAA, 0x22, 0x9D, 0x3D, 0x63, 0x98, 0xBE, 0xEE, 0xB4, 0xE2, 0x7F, 0xD6, 0x75, 0x1F, 0x5C, 0x83};
const long int DEVADDR = 0x26011BFE;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t* buf)
{}
void os_getDevEui(u1_t* buf)
{}
void os_getDevKey(u1_t* buf)
{}

// forward declaration
void do_send(osjob_t* j);

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 20;

// Pin mapping Dragino Shield
const lmic_pinmap lmic_pins =
{
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9, .dio =
  { 2, 6, 7}
};

void onEvent(ev_t ev)
{
  if (ev == EV_TXCOMPLETE)
  {
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
  }
}

void do_send(osjob_t* j)
{
  // Payload to send (uplink)
  static uint8_t message[] = "hi";

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, message, sizeof(message) - 1, 0);
    Serial.println(F("Sending uplink packet..."));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}
#endif

//-----------------------------------------------------------------------------

void setup()
{
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, 0);

  setupProdDebugEnv();

//  // LMIC init
//  os_init();
//
//  // Reset the MAC state. Session and pending data transfers will be discarded.
//  LMIC_reset();
//
//  // Set static session parameters.
//  LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
//
//  // Disable link check validation
//  LMIC_setLinkCheckMode(0);
//
//  // TTN uses SF9 for its RX2 window.
//  LMIC.dn2Dr = DR_SF9;
//
//  // Set data rate and transmit power for uplink
//  LMIC_setDrTxpow(DR_SF7, 14);

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Start job
  do_send(&sendjob);
}

void loop()
{
  if (0 != sCmd)
  {
    sCmd->readSerial();     // process serial commands
  }
  os_runloop_once();
  yield();                  // process Timers
}
