
#ifndef ESP32_BME280_MEASURETASK_H
#define ESP32_BME280_MEASURETASK_H

#include <Task.h>

#include "sdkconfig.h"


class MeasureTask: public Task {
public:
    using Task::Task;

    void run(void *data) override;
};


#endif //ESP32_BME280_MEASURETASK_H
