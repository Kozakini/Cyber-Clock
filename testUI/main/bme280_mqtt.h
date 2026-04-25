#pragma once

#include "esp_err.h"

/**
 * Konfiguracja - dostosuj do swojego projektu
 */
#define BME280_I2C_ADDR     0x76        // lub 0x77
#define BME280_I2C_PORT     I2C_NUM_0
#define BME280_SDA_PIN      21
#define BME280_SCL_PIN      22

#define MQTT_BROKER_URI     "mqtt://192.168.1.100"
#define MQTT_BROKER_PORT    1883
#define MQTT_TOPIC          "home/epaper/data"
#define MQTT_CLIENT_ID      "epaper_esp32"

/**
 * Dane z czujnika
 */
typedef struct {
    float temperature;  // °C
    float humidity;     // %RH
    float pressure;     // hPa
} bme280_data_t;

/**
 * Inicjalizuje I2C i czujnik BME280.
 * Wywołaj raz w app_main() przed pętlą główną.
 *
 * @return ESP_OK lub kod błędu
 */
esp_err_t bme280_mqtt_init(void);

/**
 * Odczytuje dane z BME280, buduje JSON i publikuje na MQTT.
 * Wywołuj cyklicznie (zalecane min. co 180s ze względu na e-Paper).
 *
 * @param data  opcjonalnie - jeśli != NULL, wypełni strukturę odczytanymi danymi
 * @return ESP_OK lub kod błędu
 */
esp_err_t bme280_mqtt_publish(bme280_data_t *data);

/**
 * Zwalnia zasoby MQTT (wywołaj przy shutdown).
 */
void bme280_mqtt_deinit(void);
