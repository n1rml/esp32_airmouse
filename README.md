# ESP32 Bluetooth AirMouse

This is my first project on [ESP-IDF Framework](https://github.com/espressif/esp-idf)
it's based on [esp32_mouse_keyboard by Benjamin Aigner](https://github.com/asterics/esp32_mouse_keyboard) and MPU6050 3-axis gyro and accelerometer.

### Credits and many thanks to:
- Benjamin Aigner for his work on implementing HID over BLE on the ESP32: https://github.com/asterics/esp32_mouse_keyboard
- Paul Stoffregen for the implementation of the keyboard layouts for his Teensyduino project: www.pjrc.com
- Neil Kolban for his great contributions to the ESP32 SW (in particular the Bluetooth support): https://github.com/nkolban
- Natanael Rabello for the MPU6050 driver for esp-idf : https://github.com/natanaeljr/esp32-MPU-driver

### Prerequisites

Install the I2Cbus and MPU Driver
```
I2Cbus:  git clone https://github.com/natanaeljr/esp32-I2Cbus.git I2Cbus
```
**Note**: Edit the I2Cbus driver location in the MAKEFILE.

Download the repository [here](https://github.com/natanaeljr/esp32-MPU-driver/archive/master.zip),
or clone it right into your project components directory with the following command.

```
git clone https://github.com/natanaeljr/esp32-MPU-driver.git MPUdriver
``` 
Follow [this](https://github.com/natanaeljr/esp32-MPU-driver/blob/master/README.md) for detailed instructions on installing the MPU driver (or for using SPIbus instead of I2C).

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
 - Able to move cursor using only the gyro values (in Degrees per Second, from the MPU driver), the movement is very random and not accurate at all.
 
 
any advice on how to correctly implement the MPU6050 values is highly appreciated.