#include <stdio.h>
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
#include "mpu6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "config.h"
#include "HID_kbdmousejoystick.h"

#include "esp_gap_ble_api.h"
#include "driver/uart.h"
#include "keyboard.h"

/** demo mouse speed */
#define MOUSE_SPEED 30
#define MAX_CMDLEN 100

#define EXT_UART_TAG "EXT_UART"
#define CONSOLE_UART_TAG "CONSOLE_UART"

//MPU
#define PIN_SDA 22
#define PIN_CLK 23

static uint8_t keycode_modifier = 0;
static uint8_t keycode_deadkey_first = 0;
//static joystick_data_t joystick;//currently unused, no joystick implemented
static config_data_t config;
// joystick_command_t joystickCmd;

// MPU vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
float yaw = 0.0, pitch = 0.0, roll = 0.0;
float vertZero = 0, horzZero = 0;
float vertValue, horzValue;
bool enable_air = true;

void sendKeyCode(uint8_t c, keyboard_action type)
{
	keyboard_command_t keyboardCmd;
	keyboardCmd.type = type;
	keyboardCmd.keycode = c + ((uint16_t)keycode_modifier << 8);
	xQueueSend(keyboard_q, (void *)&keyboardCmd, (TickType_t)0);
}

void sendKey(uint8_t c, keyboard_action type)
{
	uint8_t keycode;
	keycode = parse_for_keycode(c, config.locale, &keycode_modifier, &keycode_deadkey_first); //send current byte to parser
	if (keycode == 0)
	{
		ESP_LOGI(EXT_UART_TAG, "keycode is 0 for 0x%X, skipping to next byte", c);
		return; //if no keycode is found,skip to next byte (might be a 16bit UTF8)
	}
	ESP_LOGI(EXT_UART_TAG, "keycode: %d, modifier: %d, deadkey: %d", keycode, keycode_modifier, keycode_deadkey_first);
	//TODO: do deadkey sequence...
	//if a keycode is found, add to keycodes for HID
	sendKeyCode(keycode, type);
}

#define CMDSTATE_IDLE 0
#define CMDSTATE_GET_RAW 1
#define CMDSTATE_GET_ASCII 2

struct cmdBuf
{
	int state;
	int expectedBytes;
	int bufferLength;
	uint8_t buf[MAX_CMDLEN];
};

char character;
mouse_command_t mouseCmd;
keyboard_command_t keyboardCmd;

void uart_console(void *pvParameters)
{
	char character;
	mouse_command_t mouseCmd;
	keyboard_command_t keyboardCmd;
	//Install UART driver, and get the queue.
	uart_driver_install(CONSOLE_UART_NUM, UART_FIFO_LEN * 2, UART_FIFO_LEN * 2, 0, NULL, 0);

	ESP_LOGI("UART", "console UART processing task started");

	while (1)
	{
		// read single byte
		uart_read_bytes(CONSOLE_UART_NUM, (uint8_t *)&character, 1, portMAX_DELAY);
		// uart_parse_command(character, &cmdBuffer);

		if (HID_kbdmousejoystick_isConnected() == 0)
		{
			ESP_LOGI(CONSOLE_UART_TAG, "Not connected, ignoring '%c'", character);
		}
		else
		{
			//Do not send anything if queues are uninitialized
			if (mouse_q == NULL || keyboard_q == NULL || joystick_q == NULL)
			{
				ESP_LOGE(CONSOLE_UART_TAG, "queues not initialized");
				continue;
			}
			switch (character)
			{
			case 'a':
				mouseCmd.x = -MOUSE_SPEED;
				mouseCmd.y = 0;
				mouseCmd.buttons = 0;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q, (void *)&mouseCmd, (TickType_t)0);
				ESP_LOGI(CONSOLE_UART_TAG, "mouse: a");
				break;
			case 's':
				mouseCmd.x = 0;
				mouseCmd.y = MOUSE_SPEED;
				mouseCmd.buttons = 0;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q, (void *)&mouseCmd, (TickType_t)0);
				ESP_LOGI(CONSOLE_UART_TAG, "mouse: s");
				break;
			case 'd':
				mouseCmd.x = MOUSE_SPEED;
				mouseCmd.y = 0;
				mouseCmd.buttons = 0;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q, (void *)&mouseCmd, (TickType_t)0);
				ESP_LOGI(CONSOLE_UART_TAG, "mouse: d");
				break;
			case 'w':
				mouseCmd.x = 0;
				mouseCmd.y = -MOUSE_SPEED;
				mouseCmd.buttons = 0;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q, (void *)&mouseCmd, (TickType_t)0);
				ESP_LOGI(CONSOLE_UART_TAG, "mouse: w");
				break;
			case 'l':
				mouseCmd.x = 0;
				mouseCmd.y = 0;
				mouseCmd.buttons = 1;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q, (void *)&mouseCmd, (TickType_t)0);
				mouseCmd.x = 0;
				mouseCmd.y = 0;
				mouseCmd.buttons = 0;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q, (void *)&mouseCmd, (TickType_t)0);
				ESP_LOGI(CONSOLE_UART_TAG, "mouse: l");
				break;
			case 'r':
				mouseCmd.x = 0;
				mouseCmd.y = 0;
				mouseCmd.buttons = 2;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q, (void *)&mouseCmd, (TickType_t)0);
				mouseCmd.x = 0;
				mouseCmd.y = 0;
				mouseCmd.buttons = 0;
				mouseCmd.wheel = 0;
				xQueueSend(mouse_q, (void *)&mouseCmd, (TickType_t)0);
				ESP_LOGI(CONSOLE_UART_TAG, "mouse: r");
				break;
			case 'q':
				ESP_LOGI(CONSOLE_UART_TAG, "received q: sending key y for test purposes");
				keyboardCmd.keycode = 28;
				keyboardCmd.type = PRESS_RELEASE;
				xQueueSend(keyboard_q, (void *)&keyboardCmd, (TickType_t)0);
				break;
			case 'i':
				enable_air = true;
				ESP_LOGI(CONSOLE_UART_TAG, "air enable");
				break;
			case 'o':
				enable_air = false;
				ESP_LOGI(CONSOLE_UART_TAG, "air disable");
				break;
			case 'g':
				ESP_LOGI(CONSOLE_UART_TAG, "console uart disable");
				vTaskDelete(NULL);
				break;
			default:
				ESP_LOGI(CONSOLE_UART_TAG, "received: %d", character);
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

	while (1)
	{
		if (HID_kbdmousejoystick_isConnected())
			blinkTime = 1000;
		else
			blinkTime = 250;

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
	mouse_command_t mouseCmd;
	MPU6050 mpu = MPU6050();
	mpu.initialize();
	mpu.dmpInitialize();

	// This need to be setup individually
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788);

	mpu.setDMPEnabled(true);

	while (1)
	{
		mpuIntStatus = mpu.getIntStatus();
		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

		if ((mpuIntStatus & 0x10) || fifoCount == 1024)
		{
			// reset so we can continue cleanly
			mpu.resetFIFO();

			// otherwise, check for DMP data ready interrupt frequently)
		}
		else if (mpuIntStatus & 0x02)
		{
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize)
				fifoCount = mpu.getFIFOCount();

			// read a packet from FIFO

			mpu.getFIFOBytes(fifoBuffer, packetSize);
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			yaw = ypr[2] /M_PI * 180;
            pitch = ypr[1] /M_PI * 180;
            roll = ypr[0] /M_PI * 180;
			vertValue = yaw - vertZero;
			horzValue = roll - horzZero;
			vertZero = yaw;
			horzZero = roll;
			if (connectedForReal() && enable_air)
			{
				if (vertValue != 0 || horzValue != 0)
				{
					mouseCmd.x = horzValue * MOUSE_SPEED;
					mouseCmd.y = vertValue * MOUSE_SPEED;
					mouseCmd.buttons = 0;
					mouseCmd.wheel = 0;
					xQueueSend(mouse_q, (void *)&mouseCmd, (TickType_t)0);
				}
			}
		}

		//Best result is to match with DMP refresh rate
		// Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
		// Now its 0x13, which means DMP is refreshed with 10Hz rate
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

void task_initI2C(void *ignore) {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
	conf.scl_io_num = (gpio_num_t)PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
	vTaskDelete(NULL);
}

extern "C" void app_main()
{
	esp_err_t ret;

	// Initialize NVS.
	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	// Read config
	nvs_handle my_handle;
	ESP_LOGI("MAIN", "loading configuration from NVS");
	ret = nvs_open("config_c", NVS_READWRITE, &my_handle);
	if (ret != ESP_OK)
		ESP_LOGE("MAIN", "error opening NVS");
	size_t available_size = MAX_BT_DEVICENAME_LENGTH;
	strcpy(config.bt_device_name, GATTS_TAG);
	nvs_get_str(my_handle, "btname", config.bt_device_name, &available_size);
	if (ret != ESP_OK)
	{
		ESP_LOGE("MAIN", "error reading NVS - bt name, setting to default");
		strcpy(config.bt_device_name, GATTS_TAG);
	}
	else
		ESP_LOGI("MAIN", "bt device name is: %s", config.bt_device_name);

	ret = nvs_get_u8(my_handle, "locale", &config.locale);
	if (ret != ESP_OK || config.locale >= LAYOUT_MAX)
	{
		// ESP_LOGE("MAIN", "error reading NVS - locale, setting to US_INTERNATIONAL");
		config.locale = LAYOUT_US_INTERNATIONAL;
	}
	else
		ESP_LOGI("MAIN", "locale code is : %d", config.locale);
	nvs_close(my_handle);

	// TBD: apply country code
	// load HID country code for locale before initialising HID
	// hidd_set_countrycode(get_hid_country_code(config.locale));

	//activate mouse & keyboard BT stack (joystick is not working yet)
	HID_kbdmousejoystick_init(1, 1, 0, 0, config.bt_device_name);
	ESP_LOGI("HIDD", "MAIN finished...");

	esp_log_level_set("*", ESP_LOG_INFO);

	// now start the tasks for processing UART input and indicator LED
	xTaskCreate(&task_initI2C, "mpu_task", 2048, NULL, configMAX_PRIORITIES, NULL);
	xTaskCreatePinnedToCore(&uart_console, "console", 4096, NULL, configMAX_PRIORITIES, NULL,0);
	xTaskCreate(&blink_task, "blink", 4096, NULL, configMAX_PRIORITIES, NULL);
	vTaskDelay(1000/portTICK_PERIOD_MS);
	xTaskCreatePinnedToCore(&mpu_poll, "mpu_loop", 8192, NULL, configMAX_PRIORITIES, NULL,1);
	
}