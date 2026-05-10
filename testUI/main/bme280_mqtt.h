// bme280_mqtt.h
#pragma once
#include "mqtt_client.h"

void bme280_init(void);
void bme280_mqtt_task_start(esp_mqtt_client_handle_t client);
