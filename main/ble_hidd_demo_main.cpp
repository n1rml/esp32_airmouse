#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "sdkconfig.h"

#include "config.h"
#include "HID_kbdmousejoystick.h"

#include "I2Cbus.hpp"
#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"

#include "esp_gap_ble_api.h"
//#include "esp_hidd_prf_api.h"
/*#include "esp_bt_defs.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"*/
#include "driver/uart.h"
//#include "hid_dev.h"
#include "keyboard.h"

/** demo mouse speed */
#define MOUSE_SPEED 30
#define MAX_CMDLEN  100

#define EXT_UART_TAG "EXT_UART"
#define CONSOLE_UART_TAG "CONSOLE_UART"

static const char* TAG = "mpu";

static constexpr gpio_num_t SDA = GPIO_NUM_22;
static constexpr gpio_num_t SCL = GPIO_NUM_23;
static constexpr uint32_t CLOCK_SPEED = 400000;  // range from 100 KHz ~ 400Hz


static uint8_t keycode_modifier=0;
static uint8_t keycode_deadkey_first=0;
//static joystick_data_t joystick;//currently unused, no joystick implemented
static config_data_t config;
// joystick_command_t joystickCmd;
static MPU_t MPU;

float gx = 0.0,gy = 0.0,gz = 0.0;

void update_config()
{
    nvs_handle my_handle;
    esp_err_t err = nvs_open("config_c", NVS_READWRITE, &my_handle);
    if(err != ESP_OK) ESP_LOGE("MAIN","error opening NVS");
    err = nvs_set_str(my_handle, "btname", config.bt_device_name);
    if(err != ESP_OK) ESP_LOGE("MAIN","error saving NVS - bt name");
    err = nvs_set_u8(my_handle, "locale", config.locale);
    if(err != ESP_OK) ESP_LOGE("MAIN","error saving NVS - locale");
    printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
    nvs_close(my_handle);
}


void sendKeyCode(uint8_t c, keyboard_action type) {
	keyboard_command_t keyboardCmd;
	keyboardCmd.type = type;
	keyboardCmd.keycode = c + ((uint16_t)keycode_modifier << 8);
	xQueueSend(keyboard_q,(void *)&keyboardCmd, (TickType_t) 0);
}

void sendKey(uint8_t c, keyboard_action type) {
    uint8_t keycode;
	keycode = parse_for_keycode(c,config.locale,&keycode_modifier,&keycode_deadkey_first); //send current byte to parser
	if(keycode == 0) 
	{
		ESP_LOGI(EXT_UART_TAG,"keycode is 0 for 0x%X, skipping to next byte",c);
		return; //if no keycode is found,skip to next byte (might be a 16bit UTF8)
	}
	ESP_LOGI(EXT_UART_TAG,"keycode: %d, modifier: %d, deadkey: %d",keycode,keycode_modifier,keycode_deadkey_first);
	//TODO: do deadkey sequence...
		//if a keycode is found, add to keycodes for HID
	sendKeyCode(keycode, type);
}

#define CMDSTATE_IDLE 0
#define CMDSTATE_GET_RAW 1
#define CMDSTATE_GET_ASCII 2

struct cmdBuf {
    int state;
	int expectedBytes;
	int bufferLength;
	uint8_t buf[MAX_CMDLEN];
} ;

uint8_t uppercase(uint8_t c) 
{
	if ((c>='a') && (c<='z')) return (c-'a'+'A');
	return(c);
}

int get_int(const char * input, int index, int * value) 
{
  int sign=1, result=0, valid=0;

  while (input[index]==' ') index++;   // skip leading spaces
  if (input[index]=='-') { sign=-1; index++;}
  while ((input[index]>='0') && (input[index]<='9'))
  {
	  result= result*10+input[index]-'0';
	  valid=1;
	  index++;
  }
  while (input[index]==' ') index++;  // skip trailing spaces
  if (input[index]==',') index++;     // or a comma
  
  if (valid) { *value = result*sign; return (index);}
  return(0);  
}

void uart_console(void *pvParameters)
{
    char character;
	mouse_command_t mouseCmd;
	keyboard_command_t keyboardCmd;
    
    //Install UART driver, and get the queue.
    uart_driver_install(CONSOLE_UART_NUM, UART_FIFO_LEN * 2, UART_FIFO_LEN * 2, 0, NULL, 0);

    ESP_LOGI("UART","console UART processing task started");
    
    while(1)
    {
        // read single byte
        uart_read_bytes(CONSOLE_UART_NUM, (uint8_t*) &character, 1, portMAX_DELAY);
		// uart_parse_command(character, &cmdBuffer);	      	

		if(HID_kbdmousejoystick_isConnected() == 0) {
			ESP_LOGI(CONSOLE_UART_TAG,"Not connected, ignoring '%c'", character);
		} else {
			//Do not send anything if queues are uninitialized
			if(mouse_q == NULL || keyboard_q == NULL || joystick_q == NULL)
			{
				ESP_LOGE(CONSOLE_UART_TAG,"queues not initialized");
				continue;
			}
			switch (character){
				case 'a':
					mouseCmd.x = -MOUSE_SPEED;
					mouseCmd.y = 0;
					mouseCmd.buttons = 0;
					mouseCmd.wheel = 0;
					xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
					ESP_LOGI(CONSOLE_UART_TAG,"mouse: a");
					break;
				case 's':
					mouseCmd.x = 0;
					mouseCmd.y = MOUSE_SPEED;
					mouseCmd.buttons = 0;
					mouseCmd.wheel = 0;
					xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
					ESP_LOGI(CONSOLE_UART_TAG,"mouse: s");
					break;
				case 'd':
					mouseCmd.x = MOUSE_SPEED;
					mouseCmd.y = 0;
					mouseCmd.buttons = 0;
					mouseCmd.wheel = 0;
					xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
					ESP_LOGI(CONSOLE_UART_TAG,"mouse: d");
					break;
				case 'w':
					mouseCmd.x = 0;
					mouseCmd.y = -MOUSE_SPEED;
					mouseCmd.buttons = 0;
					mouseCmd.wheel = 0;
					xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
					ESP_LOGI(CONSOLE_UART_TAG,"mouse: w");
					break;
				case 'l':
					mouseCmd.x = 0;
					mouseCmd.y = 0;
					mouseCmd.buttons = 1;
					mouseCmd.wheel = 0;
					xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
					mouseCmd.x = 0;
					mouseCmd.y = 0;
					mouseCmd.buttons = 0;
					mouseCmd.wheel = 0;
					xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
					ESP_LOGI(CONSOLE_UART_TAG,"mouse: l");
					break;
				case 'r':
					mouseCmd.x = 0;
					mouseCmd.y = 0;
					mouseCmd.buttons = 2;
					mouseCmd.wheel = 0;
					xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
					mouseCmd.x = 0;
					mouseCmd.y = 0;
					mouseCmd.buttons = 0;
					mouseCmd.wheel = 0;
					xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
					ESP_LOGI(CONSOLE_UART_TAG,"mouse: r");
					break;
				case 'q':
					ESP_LOGI(CONSOLE_UART_TAG,"received q: sending key y for test purposes");
					keyboardCmd.keycode = 28;
					keyboardCmd.type = PRESS_RELEASE;
					xQueueSend(keyboard_q,(void *)&keyboardCmd, (TickType_t) 0);
					break;
				default:
					ESP_LOGI(CONSOLE_UART_TAG,"received: %d",character);
					break;
			}
		}
    }
}

void blink_task(void *pvParameter)
{
    // Initialize GPIO pins
    gpio_pad_select_gpio(INDICATOR_LED_PIN);
    gpio_set_direction(INDICATOR_LED_PIN, GPIO_MODE_OUTPUT);
    int blinkTime;
    
    while(1) {
		
		if (HID_kbdmousejoystick_isConnected()) 
			blinkTime=1000;
		else blinkTime=250;
		
		
        /* Blink off (output low) */
        gpio_set_level(INDICATOR_LED_PIN, 0);
        vTaskDelay(blinkTime / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(INDICATOR_LED_PIN, 1);
        vTaskDelay(blinkTime / portTICK_PERIOD_MS);
    }
}

void mpu_poll(void *pvParameter)
{
	mpud::raw_axes_t gyroRaw;    // x, y, z axes as int16
    mpud::float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
	mouse_command_t mouseCmd;
    while (true) {
        // Read
        MPU.rotation(&gyroRaw);       // fetch raw data from the registers
        // Convert
        gyroDPS = mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);

		if(HID_kbdmousejoystick_isConnected() > 0) {
			if ( (gx - gyroDPS[0]) <= -0.5 && (gz - gyroDPS[2]) <= -0.5  )
			{
				mouseCmd.x = -5;
				mouseCmd.y = -5;
				mouseCmd.buttons = 0;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
			}
			else if ( (gx - gyroDPS[0]) >= 0.5 && (gz - gyroDPS[2]) >= 0.5  )
			{
				mouseCmd.x = 5;
				mouseCmd.y = 5;
				mouseCmd.buttons = 0;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
			}
			else if ( (gx - gyroDPS[0]) >= 0.5 && (gz - gyroDPS[2]) <= -0.5  )
			{
				mouseCmd.x = -5;
				mouseCmd.y = 5;
				mouseCmd.buttons = 0;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
			}
			else if ( (gx - gyroDPS[0]) <= -0.5 && (gz - gyroDPS[2]) >= 0.5  )
			{
				mouseCmd.x = 5;
				mouseCmd.y = -5;
				mouseCmd.buttons = 0;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
			}
			else if ( (gx - gyroDPS[0]) >= 0.5 )
			{
				mouseCmd.x = 0;
				mouseCmd.y = 5;
				mouseCmd.buttons = 0;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
			}
			else if ( (gx - gyroDPS[0]) <= -0.5 )
			{
				mouseCmd.x = 0;
				mouseCmd.y = -5;
				mouseCmd.buttons = 0;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
			}
			else if ( (gz - gyroDPS[2]) >= 0.5 )
			{
				mouseCmd.x = 5;
				mouseCmd.y = 0;
				mouseCmd.buttons = 0;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
			}
			else if ( (gz - gyroDPS[2]) <= -0.5 )
			{
				mouseCmd.x = -5;
				mouseCmd.y = 0;
				mouseCmd.buttons = 0;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q,(void *)&mouseCmd, (TickType_t) 0);
			}
			
		}
        // Debug
        // printf("gyro: [%+7.2f %+7.2f %+7.2f ] (º/s)\n", gyroDPS[0], gyroDPS[1], gyroDPS[2]);
		gx = gyroDPS[0];
		gy = gyroDPS[1];
		gz = gyroDPS[2];
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main()
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // Read config
    nvs_handle my_handle;
	ESP_LOGI("MAIN","loading configuration from NVS");
    ret = nvs_open("config_c", NVS_READWRITE, &my_handle);
    if(ret != ESP_OK) ESP_LOGE("MAIN","error opening NVS");
    size_t available_size = MAX_BT_DEVICENAME_LENGTH;
    strcpy(config.bt_device_name, GATTS_TAG);
    nvs_get_str (my_handle, "btname", config.bt_device_name, &available_size);
    if(ret != ESP_OK) 
    {
        ESP_LOGE("MAIN","error reading NVS - bt name, setting to default");
        strcpy(config.bt_device_name, GATTS_TAG);
    } else ESP_LOGI("MAIN","bt device name is: %s",config.bt_device_name);

    ret = nvs_get_u8(my_handle, "locale", &config.locale);
    if(ret != ESP_OK || config.locale >= LAYOUT_MAX) 
    {
        ESP_LOGE("MAIN","error reading NVS - locale, setting to US_INTERNATIONAL");
        config.locale = LAYOUT_US_INTERNATIONAL;
    } else ESP_LOGI("MAIN","locale code is : %d",config.locale);
    nvs_close(my_handle);

    // TBD: apply country code
    // load HID country code for locale before initialising HID
    // hidd_set_countrycode(get_hid_country_code(config.locale));


    //activate mouse & keyboard BT stack (joystick is not working yet)
    HID_kbdmousejoystick_init(1,1,0,0,config.bt_device_name);
    ESP_LOGI("HIDD","MAIN finished...");
    
    esp_log_level_set("*", ESP_LOG_INFO); 

	fflush(stdout);
	i2c0.begin(SDA, SCL, CLOCK_SPEED);

	MPU.setBus(i2c0);  // set bus port, not really needed since default is i2c0
    MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);  // set address, default is AD0_LOW

    // Great! Let's verify the communication
    // (this also check if the connected MPU supports the implementation of chip selected in the component menu)
    while (esp_err_t err = MPU.testConnection()) {
        ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU connection successful!");

    // Initialize
    ESP_ERROR_CHECK(MPU.initialize());  // initialize the chip and set initial configurations
    // Setup with your configurations
    ESP_ERROR_CHECK(MPU.setSampleRate(250));  // set sample rate to 50 Hz
    ESP_ERROR_CHECK(MPU.setGyroFullScale(mpud::GYRO_FS_500DPS));
    ESP_ERROR_CHECK(MPU.setAccelFullScale(mpud::ACCEL_FS_4G));

	
    // now start the tasks for processing UART input and indicator LED  
 
    xTaskCreate(&uart_console,  "console", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(&blink_task, "blink", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(&mpu_poll, "mpu", 4096, NULL, configMAX_PRIORITIES, NULL);
}