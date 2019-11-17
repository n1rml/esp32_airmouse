# ESP32 Bluetooth Air Mouse

This is my first project on [ESP-IDF Framework](https://github.com/espressif/esp-idf)
it's based on [esp32_mouse_keyboard by Benjamin Aigner](https://github.com/asterics/esp32_mouse_keyboard) and MPU6050 3-axis gyro and accelerometer.

### Credits and many thanks to:
- Benjamin Aigner for his work on implementing HID over BLE on the ESP32: https://github.com/asterics/esp32_mouse_keyboard
- Neil Kolban for his great contributions to the ESP32 SW (in particular the Bluetooth support): https://github.com/nkolban
- Jeff Rowberg for the MPU6050 library for esp-idf : https://github.com/jrowberg/i2cdevlib/tree/master/ESP32_ESP-IDF
- Natanael Rabello for I2Cbus driver for esp-idf : https://github.com/natanaeljr/esp32-I2Cbus

### Prerequisites
Install ESP-IDF : https://github.com/espressif/esp-idf

**Note**: I use [this](https://dl.espressif.com/dl/xtensa-esp32-elf-win32-1.22.0-80-g6c4433a-5.2.0.zip) toolchain under a msys2 environment on Windows 10.

Install the I2Cbus driver
```
git clone https://github.com/natanaeljr/esp32-I2Cbus.git I2Cbus
```
**Note**: Edit the I2Cbus driver location in Makefile, or clone the driver directly into your components directory.

### Installation
Clone the project
```
git clone https://github.com/n1rml/esp32_airmouse.git esp32_airmouse

cd esp32_airmouse
```
Open menuconfig to set port for your esp32
```
make menuconfig
```
Flash using
```
make flash monitor

or

make -jN flash monitor # replace N with the no. of threads in your computer. build will finish double quick

```
--------------------------------------------------------------------------
## Control via stdin (make monitor)

For basic mouse and keyboard testing, some Bluettooh HID reports can be triggered via the 
keyboard when the make monitor console is running (see Espressiv IDF: https://github.com/espressif/esp-idf).


|Key|Function   |Description|
|---|-----------|-----------|
|a  |Mouse left |Move mouse left by 30px |
|s  |Mouse down |Move mouse down by 30px |
|d  |Mouse right|Move mouse right by 30px |
|w  |Mouse up   |Move mouse up by 30px |
|l  |Click left |Mouse click right |
|r  |Click right|Mouse click left  |
|q  |Type 'y'   |just for testing keyboard reports|

## Progress
 - Able to connect to a windows pc and operate via stdin (make monitor).
 - ~~Able to move cursor using only the gyro values (in Degrees per Second, from the MPU driver), the movement is very random and not accurate at all.~~
 - Using MPU6050's integrated DMP(Digital Motion Processor) to obtain stable mouse movement data.
 - Able to control mouse cursor with acceptable accuracy.
 
 
Do report issues, i will gladly fix them.
