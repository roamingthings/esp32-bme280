#include <esp_log.h>
#include <string>
#include <FreeRTOS.h>
#include "MeasureTask.h"

#include "sdkconfig.h"

static char LOG_TAG[] = "BME280_main";

extern "C" {
	void app_main(void);
}

static MeasureTask measureTask("Measure");

void app_main(void)
{
    ESP_LOGI(LOG_TAG, "ESP32 Measure starting up...");

    measureTask.start();
    FreeRTOS::deleteTask();
}
