# Shorebird-Tracking-Bracelet
* MCUXpresso IDE is used with the QN9080DK connected via usb.
* Main code is in /power_mode_switch/power_mode_switch.c
* It will loop. each time sleeping for 10 seconds and then waking, reading the peripheral EEPROM via SPI to get the current memory location (line 369),
* then reading the peripheral real-time clock via SPI (line 403)
* reading the analogue light sensor (line 425),
* the memory location, time and light sensor reading are then written to the peripheral EEPROM.
* the device is then put to sleep for 10 seconds and then the process is repeated.
