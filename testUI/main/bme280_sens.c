#include "bme280_sens.h"
#include "bmp280.h"
#include "i2cdev.h"
#include "esp_log.h"
#include <string.h>

#define SDA_PIN  GPIO_NUM_20
#define SCL_PIN  GPIO_NUM_21

static const char *TAG = "BME280";
static bmp280_t dev = {0};

esp_err_t bme280_init_sensor(void)
{
    ESP_ERROR_CHECK(i2cdev_init());

    bmp280_params_t params;
    bmp280_init_default_params(&params);

    esp_err_t err;

    // 0x76
    err = bmp280_init_desc(&dev,
                           BMP280_I2C_ADDRESS_0,
                           I2C_NUM_0,
                           SDA_PIN,
                           SCL_PIN);

    if (err == ESP_OK)
        err = bmp280_init(&dev, &params);

    // 0x77
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Próba adresu 0x77...");

        memset(&dev, 0, sizeof(dev));

        err = bmp280_init_desc(&dev,
                               BMP280_I2C_ADDRESS_1,
                               I2C_NUM_0,
                               SDA_PIN,
                               SCL_PIN);

        if (err == ESP_OK)
            err = bmp280_init(&dev, &params);
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Nie udało się zainicjować BME280!");
        return err;
    }

    ESP_LOGI(TAG, "BME280 initialized successfully!");
    return ESP_OK;
}
esp_err_t bme280_read_values(float *temp, float *press, float *hum)
{
    return bmp280_read_float(&dev, temp, press, hum);
}