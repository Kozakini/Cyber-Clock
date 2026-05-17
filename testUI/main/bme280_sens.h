#pragma once

#include "esp_err.h"

esp_err_t bme280_init_sensor(void);
esp_err_t bme280_read_values(float *temp, float *press, float *hum);