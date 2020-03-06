idf_component_register(SRCS "main.cpp" "./main.cpp" "ble_hid/hid_device_le_prf.c" "ble_hid/hal_ble.c" "ble_hid/hid_dev.c" "MPU6050/MPU6050.cpp" "I2Cdev/I2Cdev.cpp"
                       INCLUDE_DIRS "." "ble_hid" "I2Cdev" "MPU6050"                     # Edit following two lines to set component requirements (see docs)
                       REQUIRES 
                       PRIV_REQUIRES )

                                          