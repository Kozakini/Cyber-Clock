// bme280_mqtt.c
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cJSON.h"
#include "mqtt_client.h"
#include "i2c_bus.h"
#include "bme280.h"
#include "bme280_mqtt.h"

#define I2C_MASTER_SCL_IO       18
#define I2C_MASTER_SDA_IO       46
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_PORT                I2C_NUM_0
#define BME280_I2C_ADDR         0x77        // twój czujnik na 0x77

#define MQTT_TOPIC              "sensors/bme280"
#define MQTT_PUBLISH_INTERVAL_MS 5000

static const char *TAG = "BME280_MQTT";

static i2c_bus_handle_t  i2c_bus    = NULL;
static bme280_handle_t   bme280     = NULL;
static esp_mqtt_client_handle_t s_mqtt_client = NULL;

void bme280_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_bus = i2c_bus_create(I2C_PORT, &conf);
    if (!i2c_bus) {
        ESP_LOGE(TAG, "Błąd inicjalizacji magistrali I2C");
        return;
    }

    bme280 = bme280_create(i2c_bus, BME280_I2C_ADDR);
    if (!bme280) {
        ESP_LOGE(TAG, "Błąd tworzenia uchwytu BME280");
        return;
    }

    ESP_ERROR_CHECK(bme280_default_init(bme280));
    ESP_LOGI(TAG, "BME280 zainicjalizowany ✓");
}

static void bme280_mqtt_task(void *pvParameters)
{
    float temperature, pressure, humidity;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(MQTT_PUBLISH_INTERVAL_MS));

        esp_err_t ret = ESP_OK;
        ret |= bme280_read_temperature(bme280, &temperature);
        ret |= bme280_read_pressure(bme280, &pressure);
        ret |= bme280_read_humidity(bme280, &humidity);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Błąd odczytu BME280");
            continue;
        }

        ESP_LOGI(TAG, "T=%.2f°C  H=%.2f%%  P=%.2fhPa",
                 temperature, humidity, pressure / 100.0f);

        if (!s_mqtt_client) continue;

        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "temperature", temperature);
        cJSON_AddNumberToObject(root, "humidity",    humidity);
        cJSON_AddNumberToObject(root, "pressure",    pressure / 100.0f);
        cJSON_AddStringToObject(root, "device",      "esp32_s3_01");
        cJSON_AddNumberToObject(root, "timestamp",   (double)esp_timer_get_time() / 1e6);

        char *json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC, json_str, 0, 1, 0);
            ESP_LOGI(TAG, "Opublikowano → %s", MQTT_TOPIC);
            cJSON_free(json_str);
        }
        cJSON_Delete(root);
    }
}

void bme280_mqtt_task_start(esp_mqtt_client_handle_t client)
{
    s_mqtt_client = client;
    static bool started = false;
    if (!started) {
        xTaskCreate(bme280_mqtt_task, "bme280_mqtt", 4096, NULL, 5, NULL);
        started = true;
    }
}
