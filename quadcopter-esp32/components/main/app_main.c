#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "sensor_task.h"
#include "control_task.h"
#include "actuator_task.h"
#include "telemetry_task.h"
#include "safety_task.h"
#include "led_indicator.h"
#include "flight_config.h"
#include "version.h"


static const char *TAG="MAIN";

//Global system resource
QueueHandle_t sensor_data_queue;
QueueHandle_t control_cmd_queue;
QueueHandle_t telemetry_queue;
SemaphoreHandle_t i2c_mutex;
EventGroupHandle_t system_events;
TaskHandle_t safety_task_handle=NULL;


//System state
typedef struct{
	uint8_t armed:1;
	uint8_t calibrated:1;
	uint8_t failsafe:1;
	uint8_t flying:1;
	uint8_t error_code;
	float battery_voltage;
	uint32_t flight_time_ms;
}system_state_t;


static system_state_t sys_state={0};

//Event group bits
#define EVENT_ARMED (1 << 0)
#define EVENT_DISARMED (1 << 1) 
#define EVENT_IMU_READY (1 << 2)
#define EVENT_RC_LOST (1 << 3)
#define EVENT_LOW_BATTERY (1 << 4) 
#define EVENT_CRITICAL_ERROR (1 << 5) 
#define EVENT_CALIBRATED (1 << 6) 

void system_init(void) {
	ESP_LOG(TAG, "Quadcoptor Flight Controller v%s", FIRMWARE_VERSION);
	ESP_LOG(TAG, "CPU Frequency: %d MHz", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);


	//Initialize NVS
	esp_err_t ret=nvs_flas_init();
	if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	ESP_ERROR_CHECK(ret);

	//Create RTOS resources
	sensor_data_queue = xQueueCreate(SENSOR_QUEUE_LENGTH, sizeof(sensor_data_t));
	control_cmd_queue = xQueueCreate(CONTROL_QUEUE_LENGTH, sizeof(control_cmd_t));
	telemetry_queue = xQueueCreate(TELEMETRY_QUEUE_LENGTH, sizeof(telemetry_data_t));
	i2c_mutex = xSemaphoreCreateMutex();
	system_events = xEventGroupCreate();

	//Initialize LED indicators 
	led_init();
	led_set_pattern(LED_PATTERN_INIT);
}


void create_tasks(void){
	ESP_LOGI(TAG, "Creating RTOS tasks...");

	//Create tasks with stack sizes and priorities 
	xTaskCreatePinnedToCore(sensor_task, "SENSOR", 4096, NULL,
		       SENSOR_TASK_PRIORITY, NULL, 1);

	xTaskCreatePinnedToCore(control_task, "CONTROL", 8192, NULL, 
		CONTROL_TASK_PRIORITY, NULL, 1);

	xTaskCreatePinnedToCore(actuator_task, "ACTUATOR", 3072, NULL, 
		ACTUATOR_TASK_PRIORITY, NULL, 0);	






