# Embedded software for the mech

This is a Platformio project directory.

Import this from Platformio home.

It should set things up well for you.

## Issues
If serial monitor does not work on an ESP-32 CAM with Platformio it is because DTR and RTS both are connected to the RESET pin and GPIO 0 of the esp 8266 chip.
By default in the terminal serial port in visual studio code DTR and RST pin of the the serial to usb adapter are active and prevent the chip to act as normal.
**Solution**:
In terminal serial port press CTRL + T and then CTRL + D to inactive DTR and then CTRL + T and then CTRL + R to inactive RST pin.
