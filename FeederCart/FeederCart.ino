/*
  Feeder Cart
  by Scott Mitchell at RMIT University

  Started 10/06/16

  A01
  - based on hello_torotise animal showers.

  Scott Mitchell <scott@openobject.org>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  version 2 as published by the Free Software Foundation.

  -------------------------------

  This program is part of the food delivery system for the Tasmainian Devils
  breading program at Heilsville Sancturary. The program controls the opperation
  of a cart that takes food along a zip line and delivers the food to the appropriate
  Devil's pen at a pre-programed time. Settings are uploaded to the cart via bluetooth
  from a Android phone app.

  The system uses a RTC to keep track of the time and enters low power sleep mode while
  waiting for the delivery time. An optical encoder on the main drive motor is used
  to track the cart movement, this is checked against readings from an IR reciever
  designed to pick up stripes attached to the zip line.

  The following components are included in the system:
  - main 12v DC drive motor, connected via NPN transistor to pin 8.
  - 12v DC latch motor, connected via NPN transistor to pin 9.
  - optical encoder on main drive motor, connected to pin 3.
  - magnetic hall effect sensor for latch position, connected to pin A0.
  - IR sensor, connected to pin 4.
  - RTC connected to I2C bus, pins A4 & A5.
  - H-06 bluetooth module, connected to pins 11 and 12.
  - RGB Indicator LED.

  The system enables a watchdog timer so that it will restart if problems
  are encountered. This limits the posibility of having the system stuck
  in the on position. This requires the opticode bootloader, it will not work on the
  Arduino Pro Mini without upgrading the bootloader to optiboot.

  Serial communication is possible via bluetooth. User Variables may be
  set by sending the following bytes.

  Each Data String begins with:
  startByte1 + startByte2 + startByte3 + startByte4 + data type
  data type is either:
  0 to request cart settings
  1 to set cart settings
  A data type of 1 is followed by 4 bytes: // this may have changed - see notes
  byte 0: the hour of delivery, usign 24 hour clock.
  byte 1: the minute of delivery.
  byte 2: the distance in cm for delivery
  byte 3: the pen number for delivery

*/

const boolean DEBUG = 1;

// Hardware Settings
const byte MOTOR_INT_PIN = 2;
const byte R_LED_PIN = 3;
const byte G_LED_PIN = 5;
const byte B_LED_PIN = 6;
const byte SERVO_PIN = 7;
// const byte IR_IN_PIN = 8;
// const byte IR_OUT_PIN = 4;
// const byte OP_IN_PIN = 7;
// const byte OP_OUT_PIN = 8;
const byte DRIVE_PIN = 10;
const byte BT_TX = 11;
const byte BT_RX = 12;
const byte LINE_PIN = A0;
const byte VOLTAGE_PIN = A1;
// I2c Bus is on A4 & A5

// Security Bytes. These should match the Processing code
const byte START_BYTES[4] = {'A', 'B', 'C', 'D'};

// Default User Settings - generally only used once
// These are overwritten by User Settings stored in EEPROM
// turn the system on or off. default off.
const boolean SYSTEM_ON_OFF = 0;
// Departure hour in 24 hour clock
const byte HOUR_DEPART = 0;
// Departure minute
const byte MINUTE_DEPART = 0;
// Distance in cm to drop
const unsigned int DROP_DIST = 0;
// Pen number for drop
const byte DROP_PEN = 0;
// distance to end of line
const byte END_DIST = 0;

// battery voltage multiplication factor.
// circuit is a 1:10 ratio voltage divider with 3.3v reference.
// multiplication factor was determined through experiemntation.
const byte BAT_X_FACTOR = 36;
// Time between bluetooth checks
const unsigned int BT_SLEEP = 2000;
// maximum drive speed
const byte MAX_SPEED = 255;
// accelleration rate, bigger numbers result in slower accelleration
const unsigned int ACCEL_RATE = 10;

// include the watchdog timer library
#include <avr/wdt.h>

// include Wire library for I2C bus
#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 RTC;
unsigned long lastClockRead = 0;
byte theHour = 0;
byte theMin = 0;
// time between clock readings.
// every minute (60000 millis)
const unsigned long CLOCK_READ_INTERVAL = 60000;

// Serial for bluetooth module
#include <SoftwareSerial.h>
SoftwareSerial BTserial(BT_TX, BT_RX);

// Servo library for latch
#include <Servo.h>
Servo latchServo;  // create servo object to control a servo
// Remember when Servo was moved
unsigned long ServoStartTime = 0;
// time before servo is reset
const byte SERVO_RESET_TIME = 2000;
const byte SERVO_HOME_POS = 10;
const byte SERVO_RELEASE_POS = 100;

#include <Narcoleptic.h>
// remember sleep time because Narcoleptic.millis() doesn't seem to work
unsigned long NarcMillis = 0;

// for storing user settings
#include <EEPROM.h>

// User settings
// turn the system on or off: 0 = off, 1 = on
boolean systemOnOff;
// hour of departure
byte hourDepart;
// minute of departure
byte minuteDepart;
// distance to drop
unsigned int dropDist;
// pen to drop into
byte dropPen;
// distance to end of line
unsigned int endDist;

// Machine State
// waiting to travel = 0, traveling to drop = 1, dropping = 2, traveling to end = 3, at end = 4
byte cartState = 0;

// Setup motor photoInterupt counter
boolean counterState = 0;
// there will be 4 triggers per revolution
// 1/4 wheel circumfrance = WHEEL_DIS
const unsigned int WHEEL_DIS = 10;
unsigned long distTravelled = 0;

// Remember start times
unsigned long travelStart = 0;
// Remember when LED was switched on
unsigned long LEDstartTime = 0;
// duration of LED on
const byte LED_ON = 500;

// remember the last time the motor was activated
unsigned long lastMotorSetting = 0;
// remember the motor speed
unsigned int motorSpeed = 0;

// remember reading time and check for Millsecond rollover
unsigned long lastMillis = 0;


void setup() {
  // start the watchdog timer to reset the Arduino if it becomes unresponsive
  wdt_enable(WDTO_8S);

  // setup serial for debugging
  if (DEBUG) Serial.begin(9600);
  if (DEBUG) Serial.println("Serial ON");

  // set default user settings
  setDefaultValues();

  // update user settings from EEPROM
  loadUserSettings();

  // setup Bluetooth
  BTserial.begin(9600);
  delay(10);

  // setup the Arduino pins
  pinMode(R_LED_PIN, OUTPUT); // setup LED
  digitalWrite(R_LED_PIN, HIGH); // turn LED on
  pinMode(G_LED_PIN, OUTPUT); // setup LED
  digitalWrite(G_LED_PIN, HIGH); // turn LED on
  pinMode(B_LED_PIN, OUTPUT); // setup LED
  digitalWrite(B_LED_PIN, HIGH); // turn LED on
  LEDstartTime = currentTime();

  pinMode(MOTOR_INT_PIN, INPUT_PULLUP); // motor speed counter
  pinMode(DRIVE_PIN, OUTPUT);   // setup drive motor
  digitalWrite(DRIVE_PIN, LOW); // turn drive motor off

  // attach the servo
  latchServo.attach(SERVO_PIN);
  // send servo to home position
  // closeLatch();

  // begin Wire I2C
  Wire.begin();

  // begin RTC
  RTC.begin();
  if (DEBUG) Serial.println("RTC ON");
  checkTime();

  // if the cart is enroute then there has been an error, the system has reset mid-journey
  if (cartState != 0 && cartState != 4) {
    // continue to end of line
    // to be completed! noted in GitHub

    if (DEBUG) Serial.println("ERROR: System has reset enroute");
  }

  if (DEBUG) Serial.println(" ...Setup End");
}


void loop() {
  // reset the watchdog timer
  wdt_reset();

  // check for bluetooth data
  if (BTserial.available()) getSerialData();

  // turn off LED
  if (currentTime() - LEDstartTime > LED_ON) {
    digitalWrite(R_LED_PIN, LOW);
    digitalWrite(G_LED_PIN, LOW);
    digitalWrite(B_LED_PIN, LOW);
  }

  // close latch servo
  if (currentTime() - ServoStartTime > SERVO_RESET_TIME) closeLatch();

  // check for on-off status - is this needed?
  if (systemOnOff) {

    // action based on state
    switch (cartState) {
      case 0:
        // waiting for departure
        // check RTC Time
        if ((currentTime() - lastClockRead) > CLOCK_READ_INTERVAL) {
          checkTime();
          lastClockRead = currentTime();
        }

        // if its departure time
        if (theHour == hourDepart && theMin >= minuteDepart || theHour > hourDepart) {
          // change cart state to enroute
          cartState = 1;
          // save the cart state
          EEPROM.write(8, cartState);
        }

        break;

      case 1:
        // travel to drop location
        accelDriveMotor();

        // look for location stripe with IR sensor

        // read the motor interupt
        boolean newCounterState = digitalRead(MOTOR_INT_PIN);
        if (newCounterState != counterState) {
          // the motor has moved
          distTravelled += WHEEL_DIS;
          counterState = newCounterState;
        }

        // check distanced travelled
        if (distTravelled >= dropDist) {
          // stop drive motor
          digitalWrite(DRIVE_PIN, LOW);
          // change cart state to dropping food
          cartState = 2;
          // save the cart state
          EEPROM.write(8, cartState);
        }

        break;

      case 2:
        // dropping food
        // ensure motor is off
        digitalWrite(DRIVE_PIN, LOW);
        // open latch
        openLatch();
        // accellerate motor to max speed
        if (accelDriveMotor()) {
          // when at max speed change cart state to travelling to end of line
          cartState = 3;
          // save the cart state
          EEPROM.write(8, cartState);
        }

        break;

      case 3:
        // travelling to end of line
        // ensure motor is on
        analogWrite(DRIVE_PIN, MAX_SPEED);

        // look for location stripe with IR sensor

        // check distanced travelled
        if (distTravelled >= endDist) {
          // stop drive motor
          digitalWrite(DRIVE_PIN, LOW);
          // change cart state to dropping food
          cartState = 4;
          // save the cart state
          EEPROM.write(8, cartState);
        }

        break;

      case 4:
        // cart at end of line
        // ensure motor is off
        digitalWrite(DRIVE_PIN, LOW);

        break;
    }

    if (DEBUG) {
      Serial.print(" T: ");
      Serial.print(currentTime());
      Serial.print(" Cart State: ");
      Serial.print(cartState);
    }

  }

  // sleep time
  sleepUnit();
}


// set default values
void setDefaultValues() {
  // turn the system on or off: 0 = off, 1 = on
  systemOnOff = SYSTEM_ON_OFF;
  // hour of departure
  hourDepart = HOUR_DEPART;
  // minute of departure
  minuteDepart = MINUTE_DEPART;
  // distance to drop
  dropDist = DROP_DIST;
  // Pen number for drop
  dropPen = DROP_PEN;
  // distance to end of line
  endDist = END_DIST;
}


// Load user settings from EEPROM
void loadUserSettings() {
  // check for valid data
  if (EEPROM.read(0) == START_BYTES[0] && EEPROM.read(1) == START_BYTES[1]) {

    if (DEBUG) Serial.println("Loading User Settings");

    // get the system state.
    // saved as 1 or 0
    systemOnOff = EEPROM.read(2);
    if (DEBUG) {
      Serial.print("ON/OFF: ");
      Serial.println(systemOnOff);
    }
    // get the hour of departure
    // saved as byte
    hourDepart = EEPROM.read(3);
    if (DEBUG) {
      Serial.print("Departure Time: ");
      Serial.print(hourDepart);
      Serial.print(":");
    }
    // get the minute of departure
    // saved as byte
    minuteDepart = EEPROM.read(4);
    if (DEBUG) Serial.println(minuteDepart);
    // get the distance to drop
    // saved as cm
    dropDist = combineBytes(EEPROM.read(5), EEPROM.read(6));
    if (DEBUG) {
      Serial.print("Drop: ");
      Serial.print(dropDist);
      Serial.println(" cm");
    }
    // get the pen to drop into
    // saved as byte
    dropPen = EEPROM.read(7);
    if (DEBUG) {
      Serial.print("Pen: ");
      Serial.println(dropPen);
    }
    // get the distance to end of line
    // saved as cm
    endDist = combineBytes(EEPROM.read(8), EEPROM.read(9));
    if (DEBUG) {
      Serial.print("Line: ");
      Serial.print(endDist);
      Serial.println(" cm");
    }
    // get the cart state
    // if it is traveling then the system may have reset enroute
    // this is checked in the setup routine
    cartState = EEPROM.read(10);
    if (DEBUG) {
      Serial.print("Cart State: ");
      Serial.println(cartState);
    }

    if (DEBUG) Serial.println("End Read.");

  } else {
    if (DEBUG) Serial.println("No User Settings found");
  }
}


// Save user settings to EEPROM
void saveUserSettings() {
  if (DEBUG) Serial.println("Save User Settings");

  // Sign the first two bytes
  EEPROM.write(0, START_BYTES[0]);
  EEPROM.write(1, START_BYTES[1]);
  // save the system state.
  EEPROM.write(2, systemOnOff);
  // save the depart time
  EEPROM.write(3, hourDepart);
  EEPROM.write(4, minuteDepart);
  // save the travel distance
  EEPROM.write(5, highByte(dropDist));
  EEPROM.write(6, lowByte(dropDist));
  // save the drop pen
  EEPROM.write(7, dropPen);
  // save the line distance
  EEPROM.write(8, highByte(endDist));
  EEPROM.write(9, lowByte(endDist));
  // save the cart state
  EEPROM.write(10, cartState);

  if (DEBUG) Serial.println("End EEPROM Write.");

}


// clear the user settings in EEPROM
void clearEEPROM() {
  for ( int i = 0 ; i < 50 ; i++ ) EEPROM.write(i, 0);
  if (DEBUG) Serial.println("EEPROM cleared");
}


// get data from bluetooth serial connection
boolean getSerialData() {
  if (DEBUG) Serial.println("Get Bluetooth");

  // declare byte array big enough for the largest package
  byte packageIn[20];
  byte packageSize;
  byte byte1, byte2, byte3, byte4;

  // check for security code
  if (BTserial.available()) byte1 = BTserial.read();
  if (DEBUG) Serial.println(byte1);
  if (BTserial.available()) byte2 = BTserial.read();
  if (DEBUG) Serial.println(byte2);
  if (BTserial.available()) byte3 = BTserial.read();
  if (DEBUG) Serial.println(byte3);
  if (BTserial.available()) byte4 = BTserial.read();
  if (DEBUG) Serial.println(byte4);

  while (BTserial.available()) {
    if (byte1 == START_BYTES[0] && byte2 == START_BYTES[1]
        && byte3 == START_BYTES[2] && byte4 == START_BYTES[3]) {
      if (DEBUG) Serial.println("valid access code");
      byte sum = 0;
      byte errorCode = 0; // 0 = no errors
      byte dataType = BTserial.read();
      sum ^= dataType;
      switch (dataType) {
        case 0:
          // transmit cart status battery status.
          packageSize = 3; // 2 bytes of data + checksum
          for (int i = 0; i < packageSize; i++) {
            if (BTserial.available()) {
              packageIn[i] = BTserial.read();
              sum ^= packageIn[i];
            } else errorCode = 101;
          }
          // checksum
          if (sum == 0 && errorCode == 0) postCartState();
          else errorCode = 102; // not used

          break;

        case 1:
          // set delivery details
          packageSize = 9; // 8 bytes of data + checksum
          for (int i = 0; i < packageSize; i++) {
            if (BTserial.available()) {
              packageIn[i] = BTserial.read();
              sum ^= packageIn[i];
            } else errorCode = 101;
          }
          // checksum
          if (sum == 0 && errorCode == 0) {
            // update user settings
            systemOnOff = boolean(packageIn[0]);
            hourDepart = packageIn[1];
            minuteDepart = packageIn[2];
            dropDist = combineBytes(packageIn[3], packageIn[4]);
            dropPen = packageIn[5];
            endDist = combineBytes(packageIn[6], packageIn[7]);

            // save new settings to EEPROM
            saveUserSettings();

            // reset last sensor reads
            lastClockRead = 0;

          } else errorCode = 102;

          // send confirmation reply to Android
          postReply(errorCode);
          break;
      }

    } else {
      // get the next byte for security code
      byte1 = byte2;
      byte2 = byte3;
      byte3 = byte4;
      byte4 = BTserial.read();
    }
  }

  if (DEBUG) Serial.println("End Get Data");
}


// Post Sensor Readings
void postCartState() {
  // get battery voltage as 10 x actual voltage.
  // circuit is a 1:10 ratio voltage divider with 5v reference.
  // multiplication factor was determined through experiemntation.
  byte batteryVoltage = (analogRead(VOLTAGE_PIN) * BAT_X_FACTOR) / 100;
  // get syestem state
  //  byte currentSystemState = 0;

  if (DEBUG) Serial.print("Analog Read V: ");
  if (DEBUG) Serial.println(analogRead(VOLTAGE_PIN));
  if (DEBUG) Serial.print("Voltage: ");
  if (DEBUG) Serial.println(batteryVoltage);

  // construct message
  const byte msgSize = 8;
  byte msgBytes[msgSize];
  // data type 0: send sensor settings
  msgBytes[0] = 0;
  // pack sensor settings
  msgBytes[1] = theHour;
  msgBytes[2] = theMin;
  msgBytes[3] = batteryVoltage;
  msgBytes[4] = systemOnOff;
  // calculate checksum
  byte sum = 0;
  for (int x = 0; x < 7; x++) sum ^= msgBytes[x];
  msgBytes[7] = sum;
  // send message
  // sending an additional start byte improves communication
  BTserial.write(START_BYTES[0]);
  for (int b = 0; b < 4; b++) BTserial.write(START_BYTES[b]);
  for (int i = 0; i < msgSize; i++) {
    if (DEBUG) Serial.println(msgBytes[i]);
    BTserial.write(msgBytes[i]);
  }

  if (DEBUG) Serial.println("Sending Sensor Readings");
}


// Post Confirmation
void postReply(byte _value) {
  const byte msgSize = 3;
  byte msgBytes[msgSize];
  // add data type
  msgBytes[0] = 100;
  // add data
  msgBytes[1] = _value;
  // calculate checksum
  byte sum = 0;
  for (int x = 0; x < 2; x++) sum ^= msgBytes[x];
  msgBytes[2] = sum;
  // send message
  // sending an additional start byte improves communication
  BTserial.write(START_BYTES[0]);
  for (int b = 0; b < 4; b++) BTserial.write(START_BYTES[b]);
  for (int i = 0; i < msgSize; i++) {
    BTserial.write(msgBytes[i]);
  }

  if (DEBUG) {
    Serial.print("Sending Reply :");
    Serial.println(_value, DEC);
  }
}


// combine bytes to form an int
int combineBytes( unsigned char highValue, unsigned char lowValue)
{
  int newValue = highValue;
  // bit shift the high byte into the left most 8 bits
  newValue = newValue << 8;
  // add with logical OR the low byte
  newValue |= lowValue;
  return newValue;
}


// get the current time including deep sleep time and rollover check
unsigned long currentTime() {
  unsigned long newTime = millis() + NarcMillis;
  if (newTime < lastMillis) {
    if (DEBUG) Serial.println("Millisec have rolled over");
    // reset all time values
    lastMillis = 0;
    lastClockRead = 0;
    NarcMillis = 0;
  }

  return newTime;
}


// get the current time
// if RTC is missing then now.hour() returns a large value
void checkTime() {
  // LIDAR must be on (I2C bus wont work if it isn't on)
  DateTime now = RTC.now();
  theHour = now.hour();
  theMin = now.minute();

  if (DEBUG) {
    Serial.print(" time: ");
    Serial.print(theHour);
    Serial.print(":");
    Serial.println(theMin);
  }
}


boolean accelDriveMotor() {
  if (motorSpeed < MAX_SPEED) {
    if ((currentTime() - lastMotorSetting) > ACCEL_RATE) {
      motorSpeed++;
      lastMotorSetting = currentTime();
    }
    analogWrite(DRIVE_PIN, motorSpeed);
    return false;
  } else {
    analogWrite(DRIVE_PIN, motorSpeed);
    return true;
  }
}


boolean openLatch() {
  // move to open position
  latchServo.write(SERVO_RELEASE_POS);
  // remember time
  ServoStartTime = currentTime();
}


boolean closeLatch() {
  // move to close position
  latchServo.write(SERVO_HOME_POS);
}


void sleepUnit() {
  unsigned long duration = BT_SLEEP;

  // remember sleep time
  NarcMillis += duration;

  if (DEBUG) Serial.print(" sleep: ");
  if (DEBUG)  Serial.println(duration);
  // wait for serial to send
  if (DEBUG)  delay(100);

  // break sleep into 6 sec intervals to keep the watchdog timer running
  while (duration >= 6000) {

    // reset the watchdog timer
    wdt_reset();

    // Low power sleep. During this time power consumption is minimised
    Narcoleptic.delay(6000);
    // wake up
    // wait for system to settle. min 100 millisec
    delay(100);


    // enable the watchdog timer
    wdt_enable(WDTO_8S);

    duration -= 6000;

    // check for serial communication. if true then exit while
    if (getSerialData()) duration = 0;
  }

  // reset the watchdog timer
  wdt_reset();

  // Low power sleep. During this time power consumption is minimised
  Narcoleptic.delay(duration);
  // wake up
  // wait for system to settle
  delay(100);

  // enable the watchdog timer
  wdt_enable(WDTO_8S);
}


