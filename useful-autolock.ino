// SPDX-FileCopyrightText: 2021 Kattni Rembor for Adafruit Industries
//
// SPDX-License-Identifier: MIT

/**************************************************************************/
/*!
This is a demo for the Adafruit QT2040 Trinkey and the MCP9808 temperature
sensor.
QT2040 Trinkey - https://www.adafruit.com/product/5056
MCP9808 - https://www.adafruit.com/product/5027

*/
/**************************************************************************/
#include "Adafruit_TinyUSB.h"
#include <Adafruit_NeoPixel.h>
#include "I2CDriver.h"

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] =
{
  TUD_HID_REPORT_DESC_KEYBOARD()
};

uint8_t hidcode[] = { HID_KEY_Q, HID_KEY_H };
uint8_t modcode[] = { KEYBOARD_MODIFIER_LEFTCTRL, KEYBOARD_MODIFIER_LEFTGUI,  };
// USB HID object. For ESP32 these values cannot be changed after this declaration
// desc report, desc len, protocol, interval, use out endpoint
Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_KEYBOARD, 2, false);


// Create the neopixel strip with the built in definitions NUM_NEOPIXEL and
// PIN_NEOPIXEL
Adafruit_NeoPixel pixel =
    Adafruit_NeoPixel(NUM_NEOPIXEL, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);


long previousMillis = 0;
long intervalTemp = 2000;
I2CDriver i2c = I2CDriver();
static inference_results_t results;
uint8_t hide_confidence = 90;
uint8_t lock_confidence = 50;
int hide_reset_ms = 30000;
int lock_timeout_ms = 3000;



void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("init i2c");
  i2c.begin();
  i2c.setMode(i2c.MODE_CONTINUOUS);
  i2c.setIdModelEnabled(false);
  i2c.setDebugMode(true);
  i2c.setPersistentIds(false);
  
  Serial.println("finish init i2c");
#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  TinyUSB_Device_Init(0);
#endif
  // Notes: following commented-out functions has no affect on ESP32
  // usb_hid.setBootProtocol(HID_ITF_PROTOCOL_KEYBOARD);
  // usb_hid.setPollInterval(2);
  // usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  // usb_hid.setStringDescriptor("TinyUSB Keyboard");

  usb_hid.begin();
  pinMode(PIN_SWITCH, INPUT_PULLUP); // Setup the BOOT button

  pixel.begin();
  pixel.setBrightness(20);
  pixel.show(); // Initialize all pixels to 'off'
  
  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);

}

uint8_t j = 0;
unsigned long last_detection = millis();
unsigned long last_hidden = 0;
bool triggered_hide = false;
bool triggered_lock = false;

void loop() {
  bool curr_button = !digitalRead(PIN_SWITCH);
  uint8_t keycode[6] = { 0 };
  uint8_t modifier[6] = { 0 };
  pixel.setPixelColor(0, Wheel(j++));
  pixel.show();

  results = i2c.read();
  uint8_t best_confidence = 0;
  bool multiple_good_faces = false;

  // Finds out if there are multiple faces with a confid over 50%
  // Finds the highest confidence if there are multiple faces
  for (int i=0; i< results.num_faces; i++) {
    uint8_t confidence = results.boxes[i].confidence;
    if(confidence > 99) confidence = 99;
    if ((best_confidence > hide_confidence) && (confidence > hide_confidence)) {  // There must be atleast 2 good faces detected
      multiple_good_faces = true;
    }
    if (confidence > best_confidence){
      best_confidence = confidence;
    }
  }
  
  Serial.print(results.num_faces);
  Serial.print(" - ");
  Serial.println(best_confidence);
  if (best_confidence > lock_confidence)
  {
    last_detection = millis();
    triggered_lock = false;
  }
  unsigned long current_millis = millis();


  // we have detected multiple faces - hide the top window
  if (!triggered_hide && multiple_good_faces) {
    uint8_t const report_id = 0;
    triggered_hide = true;
    keycode[0] = hidcode[1];
    usb_hid.keyboardReport(report_id, modcode[1], keycode);
    delay(20);
    usb_hid.keyboardRelease(0);
    last_hidden = current_millis;
  }

  // reset triggered_hide if we have seen multiple faces in the past 30 seconds. We want to prevent it from cycling and hiding lots of windows
  if ((current_millis - last_hidden > hide_reset_ms) && (triggered_hide)) {
    triggered_hide = false;
    Serial.println("----------\n\tRESETTING triggered_hide to false\n----------");
  }
  

  // It has been over 5 seconds since we have seen a face and we haven't triggered a Lock yet
  if ((current_millis - last_detection > lock_timeout_ms) && (!triggered_lock)) {
    // Send report if there is key pressed
    uint8_t const report_id = 0;

    keycode[0] = hidcode[0];
    usb_hid.keyboardReport(report_id, modcode[0] | modcode[1], keycode);
    delay(20);
    usb_hid.keyboardRelease(0);
  
  }

  // skip if hid is not ready e.g still transferring previous report
  if ( !usb_hid.ready() ) return;

  delay(100);
}



// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if (WheelPos < 85) {
    return pixel.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return pixel.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return pixel.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
