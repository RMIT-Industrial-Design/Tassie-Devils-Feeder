# Tassie-Devils-Feeder
An Arduino based device for the automated Tasmanian Devil feeder system.

This program is part of the food delivery system for the Tasmainian Devils breading program at Heilsville Sancturary. The program controls the opperation of a cart that takes food along a zip line and delivers the food to the appropriate Devil's pen at a pre-programed time. Settings are uploaded to the cart via bluetooth from a Android phone app.

The system uses a RTC to keep track of the time and enters low power sleep mode while waiting for the delivery time. An optical encoder on the main drive motor is used to track the cart movement, this is checked against readings from an IR reciever designed to pick up stripes attached to the zip line.

The following components are included in the system:
 - main 12v DC drive motor, connected via NPN transistor to pin 8.
 - 12v DC latch motor, connected via NPN transistor to pin 9.
 - optical encoder on main drive motor, connected to pin 3.
 - magnetic hall effect sensor for latch position, connected to pin A0.
 - IR sensor, connected to pin 4.
 - RTC connected to I2C bus, pins A4 & A5.
 - H-06 bluetooth module, connected to pins 11 and 12.
 - RGB Indicator LED.

The system enables a watchdog timer so that it will restart if problems are encountered. This limits the posibility of having the system stuck in the on position. This requires the opticode bootloader, it will not work on the Arduino Pro Mini without upgrading the bootloader to optiboot.

Serial communication is possible via bluetooth. User Variables may be set by sending the following bytes.

Each Data String begins with:
 - startByte1 + startByte2 + startByte3 + startByte4 + data type
 - data type is either:
 - 0 to request cart settings
 - 1 to set cart settings
 - A data type of 1 is followed by 4 bytes: // this may have changed - see notes
 - byte 0: the hour of delivery, usign 24 hour clock.
 - byte 1: the minute of delivery.
 - byte 2: the distance in cm for delivery
 - byte 3: the pen number for delivery
