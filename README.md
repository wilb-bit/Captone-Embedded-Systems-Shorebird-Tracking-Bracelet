# Shorebird-Tracking-Bracelet
Please see attached report (CAPSTONE PROJECT FINAL REPORT.pdf) for images and more in depth explaination of the device.

This tracking bracelet uses a system-on-a-chip (NXP QN9080C) attached with an external light sensor, EEPROM, and real-time clock.

It is to work using light based geolocation. Using the light sensor the time of midday and the time of sunrise and sunset can be calculated. This will give an approximate location. It is useful for shorebirds which travel long distances such as from Australia, through China and to Alaska.

Due to the large distances these birds travel the device must be very small and lightweight (ours is 1.9g) as to not affect the flight of the bird. Therefore satelite based geolocation (GPS) is not used as these devices are too heavy.

The bird however must return to the site that it was first tagged in order to get the data off the device. A proposed solution to this is using bluetooth which the NXP QN9080C has. The bluetooth data transfer was not in the scope of this project due to time contraints. It can be further developed to include this transfer in the future.

For demoing the functionality the device the QN9080 development kit is connect to the computer via USB with the EEPROM, light sensor and real-time clock attached. 
* MCUXpresso IDE is used to run the code
* Main code is in /power_mode_switch/power_mode_switch.c

For the demo the code is written to loop every 10 seconds as follows:
1. Waking, reading the peripheral EEPROM via SPI to get the current memory location (line 369),
2. Reading the peripheral real-time clock via SPI (line 403)
3. Reading the analogue light sensor (line 425),
4. The memory location, time and light sensor reading are then written to the peripheral EEPROM.
5. The device is then put to sleep for 10 seconds and then the process is repeated.
