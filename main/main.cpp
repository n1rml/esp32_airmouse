#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/gpio.h"
#include "config.h"
#include "ble_hid/hal_ble.h"
#include "esp_gap_ble_api.h"
#include "driver/gpio.h"
#include "driver/uart.h"
// #include "keyboard.h"
#include "mpu6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

/** demo mouse speed */
#define MOUSE_SPEED 20
#define SCROLL_SENS 0.5

#define CONSOLE_UART_TAG "CONSOLE_UART"

//MPU Pins
#define PIN_SDA 22
#define PIN_CLK 23

// BUTTON Pins
#define BUTTON_0 GPIO_NUM_13
#define BUTTON_1 GPIO_NUM_25
#define BUTTON_2 GPIO_NUM_34

// MPU vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
float yaw = 0.0, pitch = 0.0, roll = 0.0;
float vertZero = 0, horzZero = 0, scrlZero = 0;
float vertValue, horzValue, scrlValue;


static config_data_t config;
QueueHandle_t hid_ble;

void blink_task(void *pvParameter)
{
    // Initialize GPIO pins
    gpio_pad_select_gpio(INDICATOR_LED_PIN);
    gpio_set_direction(INDICATOR_LED_PIN, GPIO_MODE_OUTPUT);
    int blinkTime;
    
    while(1) {
		
		if (halBLEIsConnected()) 
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

void uart_console(void *pvParameters)
{
    char character;
	hid_cmd_t mouseCmd;
	hid_cmd_t keyboardCmd;
	static uint8_t absMouseReport[HID_ABSMOUSE_IN_RPT_LEN];
    
    //Install UART driver, and get the queue.
    uart_driver_install(CONSOLE_UART_NUM, UART_FIFO_LEN * 2, UART_FIFO_LEN * 2, 0, NULL, 0);

    ESP_LOGI("UART","console UART processing task started");
    
    while(1)
    {
        // read single byte
        uart_read_bytes(CONSOLE_UART_NUM, (uint8_t*) &character, 1, portMAX_DELAY);
		// uart_parse_command(character, &cmdBuffer);	      	

		if(halBLEIsConnected() == 0) {
			ESP_LOGI(CONSOLE_UART_TAG,"Not connected, ignoring '%c'", character);
		} else {
			//Do not send anything if queues are uninitialized
			if(hid_ble == NULL)
			{
				ESP_LOGE(CONSOLE_UART_TAG,"queues not initialized");
				continue;
			}
			switch (character){
				case 'a':
					mouseCmd.cmd[0] = 0x10;
					mouseCmd.cmd[1] = -MOUSE_SPEED;
					xQueueSend(hid_ble,(void *)&mouseCmd, (TickType_t) 0);
					ESP_LOGI(CONSOLE_UART_TAG,"mouse: a");
					break;
				case 's':
					mouseCmd.cmd[0] = 0x11;
					mouseCmd.cmd[1] = MOUSE_SPEED;
					xQueueSend(hid_ble,(void *)&mouseCmd, (TickType_t) 0);
					ESP_LOGI(CONSOLE_UART_TAG,"mouse: s");
					break;
				case 'd':
					mouseCmd.cmd[0] = 0x10;
					mouseCmd.cmd[1] = MOUSE_SPEED;
					xQueueSend(hid_ble,(void *)&mouseCmd, (TickType_t) 0);
					ESP_LOGI(CONSOLE_UART_TAG,"mouse: d");
					break;
				case 'w':
					mouseCmd.cmd[0] = 0x11;
					mouseCmd.cmd[1] = -MOUSE_SPEED;
					xQueueSend(hid_ble,(void *)&mouseCmd, (TickType_t) 0);
					ESP_LOGI(CONSOLE_UART_TAG,"mouse: w");
					break;
				case 'l':
					mouseCmd.cmd[0] = 0x13;
					xQueueSend(hid_ble,(void *)&mouseCmd, (TickType_t) 0);
					ESP_LOGI(CONSOLE_UART_TAG,"mouse: l");
					break;
				case 'r':
					mouseCmd.cmd[0] = 0x14;
					xQueueSend(hid_ble,(void *)&mouseCmd, (TickType_t) 0);
					ESP_LOGI(CONSOLE_UART_TAG,"mouse: r");
					break;
				case 'q':
					ESP_LOGI(CONSOLE_UART_TAG,"received q: sending key y for test purposes");
					keyboardCmd.cmd[0] = 0x20;
					keyboardCmd.cmd[1] = 28;
					xQueueSend(hid_ble,(void *)&keyboardCmd, (TickType_t) 0);
					break;
				case '0':
					ESP_LOGI(CONSOLE_UART_TAG,"abs mouse: click left");
					absMouseReport[0] = 0x01;
					hid_dev_send_report(hidd_le_env.gatt_if, halBLEGetConnID(),
						HID_RPT_ID_ABSMOUSE_IN, HID_REPORT_TYPE_INPUT, HID_ABSMOUSE_IN_RPT_LEN, absMouseReport);
					absMouseReport[0] = 0x00;
					hid_dev_send_report(hidd_le_env.gatt_if, halBLEGetConnID(),
						HID_RPT_ID_ABSMOUSE_IN, HID_REPORT_TYPE_INPUT, HID_ABSMOUSE_IN_RPT_LEN, absMouseReport);
					break;
				case '1':
					ESP_LOGI(CONSOLE_UART_TAG,"abs mouse: X left, Y bottom");
					absMouseReport[1] = absMouseReport[2] = 0x00;
					absMouseReport[3] = 0x7F;
					absMouseReport[4] = 0xFF;
					hid_dev_send_report(hidd_le_env.gatt_if, halBLEGetConnID(),
						HID_RPT_ID_ABSMOUSE_IN, HID_REPORT_TYPE_INPUT, HID_ABSMOUSE_IN_RPT_LEN, absMouseReport);
					break;
				case '2':
					ESP_LOGI(CONSOLE_UART_TAG,"abs mouse: X center, Y bottom");
					absMouseReport[1] = 0x40;
					absMouseReport[2] = 0x00;
					absMouseReport[3] = 0x7F;
					absMouseReport[4] = 0xFF;
					hid_dev_send_report(hidd_le_env.gatt_if, halBLEGetConnID(),
						HID_RPT_ID_ABSMOUSE_IN, HID_REPORT_TYPE_INPUT, HID_ABSMOUSE_IN_RPT_LEN, absMouseReport);
					break;
				case '3':
					ESP_LOGI(CONSOLE_UART_TAG,"abs mouse: X right, Y bottom");
					absMouseReport[1] = 0x7F;
					absMouseReport[2] = 0xFF;
					absMouseReport[3] = 0x7F;
					absMouseReport[4] = 0xFF;
					hid_dev_send_report(hidd_le_env.gatt_if, halBLEGetConnID(),
						HID_RPT_ID_ABSMOUSE_IN, HID_REPORT_TYPE_INPUT, HID_ABSMOUSE_IN_RPT_LEN, absMouseReport);
					break;
				case '4':
					ESP_LOGI(CONSOLE_UART_TAG,"abs mouse: X left, Y center");
					absMouseReport[1] = absMouseReport[2] = 0x00;
					absMouseReport[3] = 0x40;
					absMouseReport[4] = 0x00;
					hid_dev_send_report(hidd_le_env.gatt_if, halBLEGetConnID(),
						HID_RPT_ID_ABSMOUSE_IN, HID_REPORT_TYPE_INPUT, HID_ABSMOUSE_IN_RPT_LEN, absMouseReport);
					break;
				case '5':
					ESP_LOGI(CONSOLE_UART_TAG,"abs mouse: X center, Y center");
					absMouseReport[1] = 0x40;
					absMouseReport[2] = 0x00;
					absMouseReport[3] = 0x40;
					absMouseReport[4] = 0x00;
					hid_dev_send_report(hidd_le_env.gatt_if, halBLEGetConnID(),
						HID_RPT_ID_ABSMOUSE_IN, HID_REPORT_TYPE_INPUT, HID_ABSMOUSE_IN_RPT_LEN, absMouseReport);
					break;
				case '6':
					ESP_LOGI(CONSOLE_UART_TAG,"abs mouse: X right, Y center");
					absMouseReport[1] = 0x7F;
					absMouseReport[2] = 0xFF;
					absMouseReport[3] = 0x40;
					absMouseReport[4] = 0x00;
					hid_dev_send_report(hidd_le_env.gatt_if, halBLEGetConnID(),
						HID_RPT_ID_ABSMOUSE_IN, HID_REPORT_TYPE_INPUT, HID_ABSMOUSE_IN_RPT_LEN, absMouseReport);
					break;
				case '7':
					ESP_LOGI(CONSOLE_UART_TAG,"abs mouse: X left, Y top");
					absMouseReport[1] = absMouseReport[2] = 0x00;
					absMouseReport[3] = absMouseReport[4] = 0x00;
					hid_dev_send_report(hidd_le_env.gatt_if, halBLEGetConnID(),
						HID_RPT_ID_ABSMOUSE_IN, HID_REPORT_TYPE_INPUT, HID_ABSMOUSE_IN_RPT_LEN, absMouseReport);
					break;
				case '8':
					ESP_LOGI(CONSOLE_UART_TAG,"abs mouse: X center, Y top");
					absMouseReport[1] = 0x40;
					absMouseReport[2] = 0x00;
					absMouseReport[3] = absMouseReport[4] = 0x00;
					hid_dev_send_report(hidd_le_env.gatt_if, halBLEGetConnID(),
						HID_RPT_ID_ABSMOUSE_IN, HID_REPORT_TYPE_INPUT, HID_ABSMOUSE_IN_RPT_LEN, absMouseReport);
					break;
				case '9':
					ESP_LOGI(CONSOLE_UART_TAG,"abs mouse: X right, Y top");
					absMouseReport[1] = 0x7F;
					absMouseReport[2] = 0xFF;
					absMouseReport[3] = absMouseReport[4] = 0x00;
					hid_dev_send_report(hidd_le_env.gatt_if, halBLEGetConnID(),
						HID_RPT_ID_ABSMOUSE_IN, HID_REPORT_TYPE_INPUT, HID_ABSMOUSE_IN_RPT_LEN, absMouseReport);
					break;
				default:
					ESP_LOGI(CONSOLE_UART_TAG,"received: %d",character);
					break;
			}
		}
    }
}

void mpu_poll(void *pvParameter)
{
	hid_cmd_t mouseCmd;
	MPU6050 mpu = MPU6050();
	gpio_pad_select_gpio(BUTTON_2);
	gpio_set_direction(BUTTON_2, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BUTTON_2, GPIO_PULLUP_ONLY);

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
			scrlValue = pitch - scrlZero;
			vertZero = yaw;
			horzZero = roll;
			scrlValue = pitch;
			if (halBLEIsConnected())
			{
				if (vertValue != 0 || horzValue != 0)
				{
					mouseCmd.cmd[0] = 0x01;
					mouseCmd.cmd[1] = horzValue * MOUSE_SPEED;
					mouseCmd.cmd[2] = vertValue * MOUSE_SPEED;
					xQueueSend(hid_ble,(void *)&mouseCmd, (TickType_t) 0);
				}
				if (scrlValue != 0 && gpio_get_level(BUTTON_2) == 0)
				{
					mouseCmd.cmd[0] = 0x12;
					mouseCmd.cmd[1] = scrlValue * SCROLL_SENS;
					xQueueSend(hid_ble,(void *)&mouseCmd, (TickType_t) 0);
					// ESP_LOGI("SCRL","BOOST");
				}
			}
		}

		//Best result is to match with DMP refresh rate
		// Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
		// Now its 0x13, which means DMP is refreshed with 10Hz rate
		vTaskDelay(25 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

void button_poll(void *pvParameter)
{
	hid_cmd_t mouseCmd;
	gpio_pad_select_gpio(BUTTON_0);
	gpio_set_direction(BUTTON_0, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BUTTON_0, GPIO_PULLUP_ONLY);
	// gpio_pullup_dis(BUTTON_0);
	gpio_pad_select_gpio(BUTTON_1);
	gpio_set_direction(BUTTON_1, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BUTTON_1, GPIO_PULLUP_ONLY);
	// gpio_pullup_dis(BUTTON_1);

	bool mlb = false, mrb = false;

	while (1)
	{
		if (!gpio_get_level(BUTTON_0) && !mlb)
		{
			// ESP_LOGI("MLB","klik");
			mouseCmd.cmd[0] = 0x16;
			xQueueSend(hid_ble,(void *)&mouseCmd, (TickType_t) 0);
			mlb = true;
		}
		else if (gpio_get_level(BUTTON_0) == 1 && mlb)
		{
			// ESP_LOGI("MLB","release");
			mouseCmd.cmd[0] = 0x19;
			xQueueSend(hid_ble,(void *)&mouseCmd, (TickType_t) 0);
			mlb = false;
		}

		if (!gpio_get_level(BUTTON_1) && !mrb)
		{
			// ESP_LOGI("MRB","klik");
			mouseCmd.cmd[0] = 0x17;
			xQueueSend(hid_ble,(void *)&mouseCmd, (TickType_t) 0);
			mrb = true;
		}
		else if (gpio_get_level(BUTTON_1) == 1 && mrb)
		{
			// ESP_LOGI("MRB","release");
			mouseCmd.cmd[0] = 0x1A;
			xQueueSend(hid_ble,(void *)&mouseCmd, (TickType_t) 0);
			mrb = false;
		}
		vTaskDelay(25 / portTICK_PERIOD_MS);
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
    
    esp_log_level_set("*", ESP_LOG_VERBOSE);

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
    nvs_close(my_handle);
	
	halBLEInit(0,1,0,config.bt_device_name);
    ESP_LOGI("HIDD","MAIN finished...");
    hid_ble = xQueueCreate(32,sizeof(hid_cmd_t));
    
    esp_log_level_set("*", ESP_LOG_DEBUG); 

  
    // now start the tasks for processing UART input and indicator LED  
	xTaskCreate(&task_initI2C, "mpu_init", 2048, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(&uart_console,  "console", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(&blink_task, "blink", 4096, NULL, configMAX_PRIORITIES, NULL);
    vTaskDelay(1000/portTICK_PERIOD_MS);
	xTaskCreate(&mpu_poll, "mpu_loop", 8192, NULL, configMAX_PRIORITIES, NULL);
	xTaskCreate(&button_poll, "button_loop", 4096, NULL, configMAX_PRIORITIES, NULL);
}

