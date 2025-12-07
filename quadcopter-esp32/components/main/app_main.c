// Include standard I/O functions (e.g. printf) if needed for debugging
#include <stdio.h>
// Include string manipulation functions (memcpy, memset, strcmp, etc.)
#include <string.h>

// Include FreeRTOS core definitions (types, base config)
#include "freertos/FreeRTOS.h"
// Include FreeRTOS task APIs (xTaskCreate, vTaskDelay, etc.)
#include "freertos/task.h"
// Include FreeRTOS queue APIs (xQueueCreate, xQueueSend, xQueueReceive, etc.)
#include "freertos/queue.h"
// Include FreeRTOS semaphore APIs (binary/mutex semaphores, xSemaphoreTake/Give)
#include "freertos/semphr.h"

// Include core ESP32 system APIs (chip info, restart, heap, etc.)
#include "esp_system.h"
// Include SPI flash related APIs (flash size, chip ID, etc.)
#include "esp_spi_flash.h"
// Include ESP-IDF logging facility (ESP_LOGX macros)
#include "esp_log.h"
// Include NVS (non-volatile storage) driver for storing, loading config
#include "nvs_flash.h"
// Include GPIO driver for controlling basic digital pins if needed
#include "driver/gpio.h"

// Include application-specific task headers so we can create those tasks
#include "sensor_task.h"
#include "control_task.h"
#include "actuator_task.h"
#include "telemetry_task.h"
#include "safety_task.h"

// Include LED indicator driver to show system state via LEDs
#include "led_indicator.h"
// Include flight configuration parameters (gains, thresholds, etc.)
#include "flight_config.h"
// Include firmware version string definition
#include "version.h"

// Define a tag string used by ESP_LOGx macros to identify this module in logs
static const char *TAG = "MAIN";

// Declare a queue for passing sensor data from sensor_task to other tasks
QueueHandle_t sensor_data_queue;
// Declare a queue for control commands (e.g. from RC or autopilot to actuator_task)
QueueHandle_t control_cmd_queue;
// Declare a queue for telemetry data to be sent off-board
QueueHandle_t telemetry_queue;
// Declare a mutex to protect shared I2C bus access from multiple tasks
SemaphoreHandle_t i2c_mutex;
// Declare an event group for broadcasting global system events (armed, errors, etc.)
EventGroupHandle_t system_events;
// Declare a handle so we can reference/control the safety task (e.g. suspend, notify)
TaskHandle_t safety_task_handle = NULL;

// Define a structure to represent the current high-level system state
typedef struct
{
    // 1-bit flag: system is armed (motors allowed to spin)
    uint8_t armed      : 1;
    // 1-bit flag: calibration completed successfully
    uint8_t calibrated : 1;
    // 1-bit flag: failsafe mode active (e.g. RC lost, critical error)
    uint8_t failsafe   : 1;
    // 1-bit flag: quadcopter is currently flying
    uint8_t flying     : 1;
    // 8-bit error code for diagnostics (0 = no error, others = various faults)
    uint8_t error_code;
    // Current measured battery voltage in volts
    float    battery_voltage;
    // Accumulated flight time in milliseconds
    uint32_t flight_time_ms;
} system_state_t;

// Create a single global instance of system_state_t initialized to all zeros
static system_state_t sys_state = {0};

// Define event bit for "system armed" in the event group
#define EVENT_ARMED           (1 << 0)
// Define event bit for "system disarmed" in the event group
#define EVENT_DISARMED        (1 << 1)
// Define event bit for "IMU ready" (calibration complete / sensor OK)
#define EVENT_IMU_READY       (1 << 2)
// Define event bit for "RC signal lost" (failsafe input)
#define EVENT_RC_LOST         (1 << 3)
// Define event bit for "battery low" warning
#define EVENT_LOW_BATTERY     (1 << 4)
// Define event bit for "critical error" condition
#define EVENT_CRITICAL_ERROR  (1 << 5)
// Define event bit for "calibration finished successfully"
#define EVENT_CALIBRATED      (1 << 6)

// Initialize hardware, NVS, RTOS primitives, and LED indicators
void system_init(void)
{
    // Log firmware version at startup for traceability
    ESP_LOGI(TAG, "Quadcopter Flight Controller v%s", FIRMWARE_VERSION);
    // Log CPU frequency so we know what performance configuration is running
    ESP_LOGI(TAG, "CPU Frequency: %d MHz", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);

    // Initialize Non-Volatile Storage (used for config, calibration data, etc.)
    esp_err_t ret = nvs_flash_init();
    // Handle common NVS init errors where flash needs to be erased and re-initialized
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // Erase NVS partition if it is full or from old version
        ESP_ERROR_CHECK(nvs_flash_erase());
        // Try to initialize NVS again after erasing
        ret = nvs_flash_init();
    }
    // Ensure NVS initialization result is OK and abort on failure
    ESP_ERROR_CHECK(ret);

    // Create a queue for sensor data with configured length and element type
    sensor_data_queue = xQueueCreate(SENSOR_QUEUE_LENGTH, sizeof(sensor_data_t));
    // Create a queue for control commands with configured length and element type
    control_cmd_queue = xQueueCreate(CONTROL_QUEUE_LENGTH, sizeof(control_cmd_t));
    // Create a queue for telemetry data with configured length and element type
    telemetry_queue   = xQueueCreate(TELEMETRY_QUEUE_LENGTH, sizeof(telemetry_data_t));
    // Create a mutex to serialize access to the shared I2C bus
    i2c_mutex         = xSemaphoreCreateMutex();
    // Create a FreeRTOS event group for publishing and subscribing to system events
    system_events     = xEventGroupCreate();

    // Initialize the LED indicator hardware (GPIOs, timers, etc.)
    led_init();
    // Set an LED pattern that indicates system is in initialization state
    led_set_pattern(LED_PATTERN_INIT);
}

// Create all the main application tasks with appropriate stack sizes and priorities
void create_tasks(void)
{
    // Log that we are about to create FreeRTOS tasks
    ESP_LOGI(TAG, "Creating RTOS tasks...");

    // Create sensor task on core 1 with medium stack and its defined priority
    xTaskCreatePinnedToCore(sensor_task,
                            "SENSOR",
                            4096,
                            NULL,
                            SENSOR_TASK_PRIORITY,
                            NULL,
                            1);
    // Log successful creation of the sensor task
    ESP_LOGI(TAG, "SENSOR task created successfully");

    // Create control task on core 1 with larger stack (more math/control logic)
    xTaskCreatePinnedToCore(control_task,
                            "CONTROL",
                            8192,
                            NULL,
                            CONTROL_TASK_PRIORITY,
                            NULL,
                            1);
    // Log successful creation of the control task
    ESP_LOGI(TAG, "CONTROL task created successfully");

    // Create actuator task on core 0 to drive motors/ESCs
    xTaskCreatePinnedToCore(actuator_task,
                            "ACTUATOR",
                            3072,
                            NULL,
                            ACTUATOR_TASK_PRIORITY,
                            NULL,
                            0);
    // Log successful creation of the actuator task
    ESP_LOGI(TAG, "ACTUATOR task created successfully");

    // Create telemetry task on core 0 to send status to ground station
    xTaskCreatePinnedToCore(telemetry_task,
                            "TELEMETRY",
                            4096,
                            NULL,
                            TELEMETRY_TASK_PRIORITY,
                            NULL,
                            0);
    // Log successful creation of the telemetry task
    ESP_LOGI(TAG, "TELEMETRY task created successfully");

    // Create safety task on core 0 with high priority for failsafe handling
    xTaskCreatePinnedToCore(safety_task,
                            "SAFETY",
                            3072,
                            NULL,
                            SAFETY_TASK_PRIORITY,
                            &safety_task_handle,
                            0);
    // Log that all tasks have been created
    ESP_LOGI(TAG, "All tasks created successfully");
}

// Periodic task that monitors system health (heap, battery, etc.)
void system_monitor_task(void *pvParameter)
{
    // Define the period between monitor updates (1 second)
    const TickType_t xDelay = pdMS_TO_TICKS(1000);

    // Capture the current tick count as a reference for vTaskDelayUntil (no drift)
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Run this monitor logic forever as a background health checker
    while (1)
    {
        // Read current free heap size in bytes to detect leaks or heavy usage
        uint32_t heap_free = esp_get_free_heap_size();
        // Read the minimum ever free heap size as a worst-case memory watermark
        uint32_t min_free  = esp_get_minimum_free_heap_size();

        // Log heap usage at debug level for profiling and troubleshooting
        ESP_LOGD(TAG, "Heap: Free=%u, Min=%u", (unsigned)heap_free, (unsigned)min_free);

        // Read current battery voltage from ADC (or other battery monitor hardware)
        float voltage = read_battery_voltage();
        // If voltage drops below configured low-battery threshold, raise an event
        if (voltage < LOW_BATTERY_THRESHOLD)
        {
            // Set the low-battery event bit so other tasks can react (e.g. land)
            xEventGroupSetBits(system_events, EVENT_LOW_BATTERY);
            // Log a warning to indicate low-battery condition
            ESP_LOGW(TAG, "Low battery: %.2fV", voltage);
        }

        // Update global system state with latest battery voltage reading
        sys_state.battery_voltage = voltage;

        // Delay until the next 1-second boundary (relative to xLastWakeTime) to avoid drift
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

// Main entry point for ESP-IDF applications
void app_main(void)
{
    // Initialize system hardware, NVS, queues, event groups, and LEDs
    system_init();

    // Inform the user we are about to start IMU calibration
    ESP_LOGI(TAG, "Place quadcopter on level surface for calibration...");
    // Wait 3 seconds to give user time to place the quadcopter level
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Create all application tasks (sensor, control, actuator, telemetry, safety)
    create_tasks();

    // Create the system monitor task with small stack and low priority
    xTaskCreate(system_monitor_task,
                "MONITOR",
                2048,
                NULL,
                1,
                NULL);

    // Block here until either calibration completes or a critical error occurs
    EventBits_t bits = xEventGroupWaitBits(
        system_events,                           // event group to wait on
        EVENT_CALIBRATED | EVENT_CRITICAL_ERROR, // bits of interest
        pdFALSE,                                 // do not clear bits on exit
        pdFALSE,                                 // wait for any bit, not all
        portMAX_DELAY                            // wait indefinitely
    );

    // If the calibration bit is set, system is ready to fly
    if (bits & EVENT_CALIBRATED)
    {
        // Log that system finished calibration successfully
        ESP_LOGI(TAG, "System calibrated and ready for flight!");
        // Show "ready" pattern on LED to indicate we can arm
        led_set_pattern(LED_PATTERN_READY);
        // Mark global state as calibrated
        sys_state.calibrated = 1;
    }
    // Otherwise, a critical error occurred before calibration completed
    else
    {
        // Log an error indicating calibration failed
        ESP_LOGE(TAG, "Calibration failed!");
        // Set LED to error pattern to alert user
        led_set_pattern(LED_PATTERN_ERROR);
        // Abort app_main to avoid running in unsafe condition
        return;
    }

    // Main loop for app_main: mostly idle, but tracks high-level state
    while (1)
    {
        // Read current event bits without clearing them
        bits = xEventGroupGetBits(system_events);

        // If the system is armed according to events
        if (bits & EVENT_ARMED)
        {
            // Only run this block on the transition from disarmed -> armed
            if (!sys_state.armed)
            {
                // Log state change to "armed"
                ESP_LOGI(TAG, "System ARMED");
                // Mark system as armed in global state
                sys_state.armed = 1;
                // Reset flight timer so we measure time from arming
                sys_state.flight_time_ms = 0;
                // Set LED to ARMED pattern to warn that motors may spin
                led_set_pattern(LED_PATTERN_ARMED);
            }
        }
        // If the system is disarmed according to events
        else if (bits & EVENT_DISARMED)
        {
            // Only run this block on the transition from armed -> disarmed
            if (sys_state.armed)
            {
                // Log state change to "disarmed"
                ESP_LOGI(TAG, "System DISARMED");
                // Mark system as disarmed
                sys_state.armed  = 0;
                // Mark system as not flying
                sys_state.flying = 0;
                // Set LED to DISARMED pattern (safe state)
                led_set_pattern(LED_PATTERN_DISARMED);
            }
        }

        // If currently armed, accumulate flight time based on loop period
        if (sys_state.armed)
        {
            // Increment flight time by 100 ms per loop iteration
            sys_state.flight_time_ms += 100;
        }

        // Sleep this loop for 100 ms to reduce CPU usage and define timing
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
