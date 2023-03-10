/*
Bpod_USBhost
Copyright (C) 2023 Florian Rau

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include "USBHost_t36.h"
#include <ArCOM.h>

#define KEYBOARD_1BYTE 1
#define KEYBOARD_MODE_UNICODE_8BIT  2
#define KEYBOARD_MODE_UNICODE_16BIT 3
#define KEYBOARD_MODE_KEYCODE_8BIT  4

USBHost myusb;

USBHub hub[8](myusb);
KeyboardController keyboard[8](myusb);
MouseController mouse[8](myusb);
JoystickController joystick[8](myusb);
USBHIDParser hid[8](myusb);
RawHIDController rawhid[8](myusb);

// serial ports
ArCOM usbCOM(Serial);      // wrap Serial (USB on Teensy)
ArCOM Serial1COM(Serial1); // wrap Serial1 (UART on Teensy)
ArCOM *COM;                // pointer for serial port

// version information
uint32_t FirmwareVersion = 1;
uint8_t PCBrev = 0; // read from PCB in setup()

// variables
uint8_t mode = KEYBOARD_1BYTE;
extern const uint8_t vDriver; // version number of TMC stepper driver
uint8_t events[15] = {255};

// forward declarations
void ISRkeyPressRaw(uint8_t keycode);
void ISRkeyPress(int unicode);
void ISRkeyReleaseRaw(uint8_t keycode);
void ISRkeyRelease(int unicode);
void returnModuleInfo(void);
void OnHIDExtrasPress(uint32_t top, uint16_t key);
void OnHIDExtrasRelease(uint32_t top, uint16_t key);



inline __attribute__((always_inline)) void writeUint8(uint8_t data) {
  noInterrupts();
  Serial1COM.writeUint8(data);
  interrupts();
}

inline __attribute__((always_inline)) void writeUint16(uint16_t data) {
  noInterrupts();
  Serial1COM.writeUint16(data);
  interrupts();
}



void setup() {
  Serial.begin(9600);
  Serial1.begin(1312500); // Initialize serial communication

  myusb.begin();
  for (KeyboardController &i : keyboard) {
    // i.attachPress(ISRkeyPress);
    // i.attachRelease(ISRkeyRelease);
    i.attachRawPress(ISRkeyPressRaw);
    i.attachRawRelease(ISRkeyReleaseRaw);
    i.attachExtrasPress(OnHIDExtrasPress);
    i.attachExtrasRelease(OnHIDExtrasRelease);
  }

  // keyboard1.attachPress(ISRkeyPress);
  // keyboard1.attachRelease(ISRkeyRelease);
  // keyboard2.attachPress(ISRkeyPress);
  // keyboard2.attachRelease(ISRkeyRelease);

  // keyboard1.attachExtrasPress(OnHIDExtrasPress);
  // keyboard1.attachExtrasRelease(OnHIDExtrasRelease);
  // keyboard2.attachExtrasPress(OnHIDExtrasPress);
  // keyboard2.attachExtrasRelease(OnHIDExtrasRelease);
  // rawhid1.attachReceive(OnReceiveHidData);
  // rawhid2.attachReceive(OnReceiveHidData);

  // read PCB revision (GPIO 33-36)
  for (uint8_t i = 33; i <= 36; i++) {
    pinMode(i, INPUT_PULLUP);
    delay(5);
    bitWrite(PCBrev, i - 33, !digitalReadFast(i));
    pinMode(i, INPUT_DISABLE);
  }
  PCBrev *= 10;

  pinMode(LED_BUILTIN, OUTPUT);
}

void setEvent(uint8_t number, uint8_t keycode) {
  if (number < 1 || number > 15)
    return;
  events[number - 1] = keycode;
}

uint8_t getEvent(uint8_t number) {
  if (number < 1 || number > 15)
    return 255;
  return (events[number - 1]);
}

uint8_t findEvent(uint8_t keycode) {
  for (uint8_t i = 0; i < sizeof(events); i++) {
    if (events[i] == keycode)
      return i + 1;
  }
  return 255;
}

void loop() {

  if (usbCOM.available())          // byte available at usbCOM?
    COM = &usbCOM;                 //   point *COM to usbCOM
  else if (Serial1COM.available()) // byte available at Serial1COM?
    COM = &Serial1COM;             //   point *COM to Serial1COM
  else                             // otherwise
    COM = nullptr;                 //   make *COM a null pointer

  if (COM) {
    switch (COM->readUint8()) {
    case 'E':
      setEvent(COM->readUint8(), COM->readUint8());
      break;
    case 'G':
      COM->writeUint8(getEvent(COM->readUint8()));
      break;
    case 255:
      returnModuleInfo();
      break;
    }
  }
}

void ISRkeyPressRaw(uint8_t keycode)
{
  uint8_t eventNumber = findEvent(keycode);
  if (eventNumber < 255) {
    writeUint8(eventNumber);
    digitalWriteFast(LED_BUILTIN, HIGH);
  }
}

// void ISRkeyPress(int unicode)
// {
//   writeUint16(unicode);
//   digitalWriteFast(LED_BUILTIN, HIGH);
// }

void ISRkeyReleaseRaw(uint8_t keycode)
{
  digitalWriteFast(LED_BUILTIN, LOW);
}

// void ISRkeyRelease(int unicode)
// {
//   digitalWriteFast(LED_BUILTIN, LOW);
// }

void OnHIDExtrasPress(uint32_t top, uint16_t key)
{
  // only send if within first 256 keycodes
  // writeUint16(key);
  // digitalWriteFast(LED_BUILTIN, HIGH);
}

void OnHIDExtrasRelease(uint32_t top, uint16_t key)
{
  // digitalWriteFast(LED_BUILTIN, LOW);
}



void returnModuleInfo() {
  char moduleName[] = "USB"; // name of module
  const char *eventNames[] = {"key01", "key02", "key03", "key04", "key05", "key06", "key07", "key08", "key09", "key10", "key11", "key12", "key13", "key14", "key15"};
  uint8_t nEventNames  = sizeof(eventNames) / sizeof(char *);

  // FSM firmware v23 or newer sends a second info request byte to indicate that
  // it supports additional ops
  delayMicroseconds(100);
  boolean fsmSupportsHwInfo =
    (Serial1COM.available() && Serial1COM.readByte() == 255) ? true : false;

  Serial1COM.writeByte(65);                                       // Acknowledge
  Serial1COM.writeUint32(FirmwareVersion);                        // 4-byte firmware version
  Serial1COM.writeByte(sizeof(moduleName) - 1);
  Serial1COM.writeCharArray(moduleName, sizeof(moduleName) - 1);  // Module name
  Serial1COM.writeByte(1);                                        // 1 if more info follows, 0 if not
  Serial1COM.writeByte('#');                                      // Op code for: Number of behavior events this module can generate
  Serial1COM.writeByte(nEventNames);
  Serial1COM.writeByte(1);                                        // 1 if more info follows, 0 if not
  Serial1COM.writeByte('E');                                      // Op code for: Behavior event names
  Serial1COM.writeByte(nEventNames);
  for (unsigned int i = 0; i < nEventNames; i++) {                // Once for each event name
    Serial1COM.writeByte(strlen(eventNames[i]));                  // Send event name length
    for (unsigned int j = 0; j < strlen(eventNames[i]); j++) {    // Once for each character in this event name
      Serial1COM.writeByte(*(eventNames[i] + j));                 // Send the character
    }
  }
  if (fsmSupportsHwInfo) {
    Serial1COM.writeByte(1);                                      // 1 if more info follows, 0 if not
    Serial1COM.writeByte('V');                                    // Op code for: Hardware major version
    Serial1COM.writeByte(PCBrev/10);
    Serial1COM.writeByte(1);                                      // 1 if more info follows, 0 if not
    Serial1COM.writeByte('v');                                    // Op code for: Hardware minor version
    Serial1COM.writeByte(PCBrev%10);
  }
  Serial1COM.writeByte(0);                                        // 1 if more info follows, 0 if not
}
