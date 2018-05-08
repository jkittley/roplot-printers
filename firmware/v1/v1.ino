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

#if defined(HASOLED)
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #include <Adafruit_FeatherOLED.h>
  Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();
#endif

// Enable Debug Mode - Device will wait for 10 seconds for a serial connection
// if not discovered will revert back to debug=false;
bool DEBUG = true;

/**************************************************************************/
/**************************************************************************/

// Bluefruit
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void setup(void) {

  // Display
  #if defined(HASOLED) 
    oled.init();
    oled.setBatteryVisible(true);
    log(F("Roplotter Firmware V1"));
    delay(1000);
  #endif
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
   
  int start = millis();
  if (DEBUG) {
    while (!Serial) { 
      int remianing = 10000 - (millis() - start);
      log("Waiting for serial:" + String(remianing) );
      if (remianing < 1) {
        DEBUG=false;
        break;  
      }
      delay(100);
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
    log(F("Data available"));
    // Get char
    char c = (char) ble.read();
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
  log("Processing: " + cmdString); 
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
//    ble.print(inputs);
//  }
//
//  // Echo received data



/**************************************************************************/
/**************************************************************************/

void waitForBLE() {
  if (ble.isConnected()) return;
  log(F("Waiting for BLE connection"));
  while (!ble.isConnected()) { delay(500); }
  // LED Activity command is only supported from 0.6.6
  if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION)) {
      ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }
  // Set module to DATA mode
  ble.setMode(BLUEFRUIT_MODE_DATA);
  log(F("BLE connected"));
}

/**************************************************************************/
/**************************************************************************/

// Error helper
void error(String e) {
  if (DEBUG) Serial.println(e);
  #if defined(HASOLED)
    displayMessage("ERROR:" + String(e));
  #endif
  while (1);
}

// Log helper
void log(String s) {
  if (DEBUG) Serial.println(s);
  #if defined(HASOLED)
    displayMessage(s);
  #endif
}

// Display Message
void displayMessage(String line1, String line2, String line3) {
  oled.clearMsgArea();
  // update the battery icon
  displayBattery(); 
  oled.println(line1);
  oled.println(line2);
  oled.println(line3);
  oled.display();
}
void displayMessage(String line1) { return displayMessage(String(line1), String(""), String("")); }
void displayMessage(String line1, String line2) { return displayMessage(String(line1), String(line2), String("")); }

// Display Message
void displayBattery() {
  float battery = getBatteryVoltage();
  oled.setBattery(battery);
  oled.renderBattery();
}

/**************************************************************************/
/**************************************************************************/

#if defined(ARDUINO_ARCH_SAMD) || defined(__AVR_ATmega32U4__)
  // m0 & 32u4 feathers
  #define VBATPIN A7
  float getBatteryVoltage() {
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    return measuredvbat;
  }
#elif defined(ESP8266)
  // esp8266 feather
  #define VBATPIN A0
  float getBatteryVoltage() {
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    return measuredvbat;
 }
#elif defined(ARDUINO_STM32_FEATHER)
  // wiced feather
  #define VBATPIN PA1
  float getBatteryVoltage() {
    pinMode(VBATPIN, INPUT_ANALOG);
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;         // we divided by 2, so multiply back
    measuredvbat *= 0.80566F;  // multiply by mV per LSB
    measuredvbat /= 1000;      // convert to voltage
    return measuredvbat;
  }
#else
  // unknown platform
  float getBatteryVoltage() {
    error("warning: unknown feather. getting battery voltage failed.");
    return 0.0F;
  }
#endif


