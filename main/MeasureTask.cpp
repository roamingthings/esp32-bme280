
#include "MeasureTask.h"

#include <esp_log.h>
#include <FreeRTOS.h>
#include <string>
#include <BME280.h>


void MeasureTask::run(void *data) {
    BME280 bme280 = BME280();
    bme280.setDebug(false);
    bme280.init();

    while (1) {
        bme280_reading_data sensor_data = bme280.readSensorData();

        printf("Temperature: %.2foC, Humidity: %.2f%%, Pressure: %.2fPa\n",
               (double) sensor_data.temperature,
               (double) sensor_data.humidity,
               (double) sensor_data.preassure
        );

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
} // End run
