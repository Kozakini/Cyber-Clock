#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO       46
#define I2C_MASTER_SDA_IO       18
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_TX_BUF       0
#define I2C_MASTER_RX_BUF       0
#define BME280_ADDR             0x77

static const char *TAG = "bme280";

#define BME280_REG_CHIP_ID      0xD0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_PRESS_MSB    0xF7

static esp_err_t bme280_read(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t bme280_write(uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1, dig_H3;
    int16_t  dig_H2, dig_H4, dig_H5;
    int8_t   dig_H6;
} bme280_calib_t;

static bme280_calib_t calib;
static int32_t t_fine;

static void bme280_read_calib(void)
{
    uint8_t buf[26];
    bme280_read(0x88, buf, 26);
    calib.dig_T1 = (buf[1] << 8) | buf[0];
    calib.dig_T2 = (buf[3] << 8) | buf[2];
    calib.dig_T3 = (buf[5] << 8) | buf[4];
    calib.dig_P1 = (buf[7] << 8) | buf[6];
    calib.dig_P2 = (buf[9] << 8) | buf[8];
    calib.dig_P3 = (buf[11] << 8) | buf[10];
    calib.dig_P4 = (buf[13] << 8) | buf[12];
    calib.dig_P5 = (buf[15] << 8) | buf[14];
    calib.dig_P6 = (buf[17] << 8) | buf[16];
    calib.dig_P7 = (buf[19] << 8) | buf[18];
    calib.dig_P8 = (buf[21] << 8) | buf[20];
    calib.dig_P9 = (buf[23] << 8) | buf[22];
    calib.dig_H1 = buf[25];

    uint8_t hbuf[7];
    bme280_read(0xE1, hbuf, 7);
    calib.dig_H2 = (hbuf[1] << 8) | hbuf[0];
    calib.dig_H3 = hbuf[2];
    calib.dig_H4 = (hbuf[3] << 4) | (hbuf[4] & 0x0F);
    calib.dig_H5 = (hbuf[5] << 4) | (hbuf[4] >> 4);
    calib.dig_H6 = hbuf[6];
}

static float bme280_compensate_temp(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * calib.dig_T2) >> 11;
    int32_t var2 = (((((adc_T >> 4) - (int32_t)calib.dig_T1) *
                      ((adc_T >> 4) - (int32_t)calib.dig_T1)) >> 12) * calib.dig_T3) >> 14;
    t_fine = var1 + var2;
    return ((t_fine * 5 + 128) >> 8) / 100.0f;
}

static float bme280_compensate_press(int32_t adc_P)
{
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * calib.dig_P6;
    var2 += (var1 * calib.dig_P5) << 17;
    var2 += (int64_t)calib.dig_P4 << 35;
    var1 = ((var1 * var1 * calib.dig_P3) >> 8) + ((var1 * calib.dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * calib.dig_P1 >> 33;
    if (var1 == 0) return 0;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = (calib.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)calib.dig_P7 << 4);
    return p / 25600.0f;
}

static float bme280_compensate_hum(int32_t adc_H)
{
    int32_t v = t_fine - 76800;
    v = (((adc_H << 14) - (calib.dig_H4 << 20) - (calib.dig_H5 * v)) + 16384) >> 15;
    v = v * (((((((v * calib.dig_H6) >> 10) *
                 (((v * calib.dig_H3) >> 11) + 32768)) >> 10) + 2097152) *
               calib.dig_H2 + 8192) >> 14);
    v -= ((((v >> 15) * (v >> 15)) >> 7) * calib.dig_H1) >> 4;
    if (v < 0) v = 0;
    if (v > 419430400) v = 419430400;
    return (v >> 12) / 1024.0f;
}

void app_main(void)
{
    // Inicjalizacja I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER,
                                       I2C_MASTER_TX_BUF, I2C_MASTER_RX_BUF, 0));

    // Sprawdź chip ID
    uint8_t chip_id;
    bme280_read(BME280_REG_CHIP_ID, &chip_id, 1);
    ESP_LOGI(TAG, "Chip ID: 0x%02X (oczekiwano 0x60)", chip_id);

    bme280_read_calib();
    bme280_write(BME280_REG_CTRL_HUM, 0x01);
    bme280_write(BME280_REG_CTRL_MEAS, 0x27);
    bme280_write(BME280_REG_CONFIG, 0xA0);

    while (1) {
        uint8_t buf[8];
        bme280_read(BME280_REG_PRESS_MSB, buf, 8);

        int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
        int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);
        int32_t adc_H = ((int32_t)buf[6] << 8)  |  buf[7];

        float temp  = bme280_compensate_temp(adc_T);
        float press = bme280_compensate_press(adc_P);
        float hum   = bme280_compensate_hum(adc_H);

        ESP_LOGI(TAG, "Temperatura: %.2f C", temp);
        ESP_LOGI(TAG, "Cisnienie:   %.2f hPa", press);
        ESP_LOGI(TAG, "Wilgotnosc:  %.2f %%", hum);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
