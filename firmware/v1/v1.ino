/*********************************************************************
 
 ROPLOT FIRMWARE V1
 
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "config.h"

// Enable Debug Mode - Device will wait for 10 seconds for a serial connection
// if not discovered will revert back to debug=false;
bool DEBUG = true;

/**************************************************************************/
/**************************************************************************/

// Bluefruit
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void setup(void) {

  log(F("Roplotter Firmware V1"));

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  showState(STATE_SETUP_STARTED);
  
  int start = millis();
  if (DEBUG) {
    while (!Serial) { 
      if (millis() - start > 10000) {
        DEBUG=false;
        break;  
      }
    }  
    Serial.begin(115200);
  }

  digitalWrite(LED_BUILTIN, LOW);

  // Initialise BLE module
  log(F("Initialising BLE..."));
  if (!ble.begin(DEBUG)) error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  
  // Reset BLE Hardware
  if ( FACTORYRESET_ENABLE ) {
    log(F("Performing a factory reset"));
    if (!ble.factoryReset()) error(F("Couldn't factory reset"));
  }

  // Set device name
  ble.sendCommandCheckOK("AT+GAPDEVNAME=Roplotter");
  
  // Disable command echo
  ble.echo(false);
  if (DEBUG) ble.info();
  ble.verbose(false);


}

/**************************************************************************/
/**************************************************************************/

int state = STATE_WAITING_BLE;
String cmdString = "";
int count_open = 0;
int count_close = 0;
    
void loop(void) {
  
  waitForBLE();
  
  while (ble.available()) {
    // Get char
    char c = (char) ble.read();
    Serial.print(c);
    // Count open braces
    if (c == '{') {
      count_open++;
    // Count closed braces
    } else if (c == '}') { 
      count_close++;
    } 
    
    cmdString += c;
    
    // If we have a matching number of open and clos braces then the json must be complete
    if (count_open == count_close) {
      processCMD(cmdString); 
      cmdString = "";
      count_open = 0;
      count_close = 0;
    } 
    
  }
  
}

int processCMD(String cmdString) {
  if (DEBUG) { Serial.print("Processing command: "); Serial.println(cmdString); }
  StaticJsonBuffer<200> cmdBuffer;
  JsonObject& cmd = cmdBuffer.parseObject(cmdString);
//  if (!cmd.success()) {
//     sendAck(false);
//     return STATE_RX_BLE;
//  } else {
//     sendAck(true);
//     return STATE_PROCESSING_CMD;
//  }
  // Handle commands
  if(cmd["type"]=="getconfig") cmdGetConfig();
  
}

void cmdGetConfig() {
  log("Configuration requested");
  StaticJsonBuffer<400> updBuffer;
  JsonObject& upd = updBuffer.createObject();
  JsonArray& pens = upd.createNestedArray("pens");
  pens.add("A");
  pens.add("B");
  upd["type"]       = "printerConfig";
  upd["boomRadius"] = boomRadius; 
  upd["boomWidth"]  = boomWidth; 
  upd["boomColor"]  = boomColor; 
  upd["drawStart"]  = drawStart; 
  upd["drawEnd"]    = drawEnd; 
  upd["carWidth"]   = carWidth; 
  upd["carHeight"]  = carHeight; 
  upd["boomStep"]   = boomRadius; 
  upd["carStep"]    = boomRadius; 
  
  if (DEBUG) upd.prettyPrintTo(Serial);
  char msgBuffer[400];
  upd.printTo(msgBuffer);
  ble.print(msgBuffer);
}


void sendAck(bool receivedSuccessfully) {
  StaticJsonBuffer<200> updBuffer;
  JsonObject& upd = updBuffer.createObject();
  upd["type"] = "ACK";
  upd["success"] = receivedSuccessfully;
  if (DEBUG) upd.prettyPrintTo(Serial);
  char msgBuffer[200];
  upd.printTo(msgBuffer);
  ble.print(msgBuffer);
}



// Check for user input
//  char n, inputs[BUFSIZE+1];
//  if (Serial.available()) {
//    n = Serial.readBytes(inputs, BUFSIZE);
//    inputs[n] = 0;
//    // Send characters to Bluefruit
//    log("Sending: ");
//    log(inputs);
//    showState(STATE_TX_BLE);
//    ble.print(inputs);
//  }
//
//  // Echo received data



/**************************************************************************/
/**************************************************************************/

void waitForBLE() {
  if (ble.isConnected()) return;
  while (!ble.isConnected()) { delay(500); }
  // LED Activity command is only supported from 0.6.6
  if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION)) {
      ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }
  // Set module to DATA mode
  log( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
}

/**************************************************************************/
/**************************************************************************/

// Error helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// Log helper
void log(String s) {
  if (DEBUG) Serial.println(s);
}

// Show system state
void showState(int state) {
//  switch (state) {
//    case STATE_SETUP_STARTED:
//      if (DEBUG) Serial.println("System started");
//      break;
//    case STATE_WAITING_BLE:
//      if (DEBUG) Serial.println("Waiting for BLE connection");
//      break;
//    case STATE_RX_BLE:
//      if (DEBUG) Serial.println("Received..");
//      break;
//    case STATE_TX_BLE:
//      if (DEBUG) Serial.println("Sending data");
//      break;
//    case STATE_PROCESSING_CMD:
//      if (DEBUG) Serial.println("Processing command");
//      break;
//    default:
//      if (DEBUG) Serial.println("Unknown state");
//  }
}
