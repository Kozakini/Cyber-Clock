#pragma once

#include "esp_err.h"
#include "mqtt_client.h"

esp_err_t mqtt_init(void);
void mqtt_publish_data(float temp, float press, float hum);