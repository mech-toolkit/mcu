# MicroControler Unit source code

This repo contains source code for Micro Controler Units.  The little boards on the mech that input vision, and ultrasonics and control the servos that move the mech.
- [esp32-cam](https://github.com/mech-toolkit/mcu/tree/main/esp32-cam) -  A Platformio project directory containing source and depedencies for an autonomous mech based on the esp32-cam.
 
# Rest of mech-toolkit:
- [mechlib](https://github.com/mech-toolkit/mechlib) - a small python libary for autonomously controling mech's.  This integrates networking, vision, and motion control to control the mech.
- [pretrained](https://github.com/mech-toolkit/pretrained) - The releases of pre-trained AI models for mechs.
- [platforms](https://github.com/mech-toolkit/platforms) - The mechanical and electrical documentation for mechs.

## Issues
If serial monitor does not work on an ESP-32 CAM with Platformio it is because DTR and RTS both are connected to the RESET pin and GPIO 0 of the esp 8266 chip.
By default in the terminal serial port in visual studio code DTR and RST pin of the the serial to usb adapter are active and prevent the chip to act as normal.
**Solution**:
In terminal serial port press CTRL + T and then CTRL + D to inactive DTR and then CTRL + T and then CTRL + R to inactive RST pin.
