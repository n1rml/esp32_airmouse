# ESP32 Bluetooth Air Mouse

This is my first project on [ESP-IDF Framework](https://github.com/espressif/esp-idf)
it's based on [esp32_mouse_keyboard by Benjamin Aigner](https://github.com/asterics/esp32_mouse_keyboard) and MPU6050 6-axis gyro and accelerometer.

### Credits and many thanks to:
- Benjamin Aigner for his work on implementing HID over BLE on the ESP32: https://github.com/asterics/esp32_mouse_keyboard
- Neil Kolban for his great contributions to the ESP32 SW (in particular the Bluetooth support): https://github.com/nkolban
- Jeff Rowberg for the MPU6050 library for esp-idf : https://github.com/jrowberg/i2cdevlib/tree/master/ESP32_ESP-IDF

### Prerequisites
Install ESP-IDF : https://github.com/espressif/esp-idf

### Installation
Clone the project
```
git clone https://github.com/n1rml/esp32_airmouse.git esp32_airmouse

cd esp32_airmouse
```
Build using
```
idf.py build
```
Flash using
```
idf.py -p (PORT) flash monitor

```
--------------------------------------------------------------------------
