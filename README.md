# Shorebird-Tracking-Bracelet
* MCUXpresso IDE is used with the QN9080DK connected via usb.
* Main code is in /power_mode_switch/power_mode_switch.c

For the demo the code is written to loop every 10 seconds as follows:
1. Waking, reading the peripheral EEPROM via SPI to get the current memory location (line 369),
2. Reading the peripheral real-time clock via SPI (line 403)
3. Reading the analogue light sensor (line 425),
4. The memory location, time and light sensor reading are then written to the peripheral EEPROM.
5. The device is then put to sleep for 10 seconds and then the process is repeated.
