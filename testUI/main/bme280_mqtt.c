#include "bme280_mqtt.h"

#include <string.h>
#include <stdio.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/* ── BME280 rejestry ────────────────────────────────────────────────────── */
#define BME280_REG_CALIB00      0x88
#define BME280_REG_CALIB26      0xE1
#define BME280_REG_CHIP_ID      0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_PRESS_MSB    0xF7

#define BME280_CHIP_ID          0x60
#define BME280_RESET_VALUE      0xB6

#define I2C_TIMEOUT_MS          100
#define I2C_FREQ_HZ             400000

static const char *TAG = "bme280_mqtt";

/* ── Dane kalibracyjne ──────────────────────────────────────────────────── */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4, dig_H5;
    int8_t   dig_H6;
} bme280_calib_t;

static bme280_calib_t s_calib;
static int32_t        s_t_fine;  // współdzielony między kompensacjami

/* ── MQTT ───────────────────────────────────────────────────────────────── */
static esp_mqtt_client_handle_t s_mqtt_client = NULL;

#define MQTT_CONNECTED_BIT  BIT0
static EventGroupHandle_t s_mqtt_events = NULL;

/* ═══════════════════════════════════════════════════════════════════════════
   Pomocnicze funkcje I2C
   ═══════════════════════════════════════════════════════════════════════════ */

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(BME280_I2C_PORT, BME280_I2C_ADDR,
                                      buf, sizeof(buf),
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t i2c_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(BME280_I2C_PORT, BME280_I2C_ADDR,
                                        &reg, 1, data, len,
                                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/* ═══════════════════════════════════════════════════════════════════════════
   Inicjalizacja I2C
   ═══════════════════════════════════════════════════════════════════════════ */

static esp_err_t i2c_init(void)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = BME280_SDA_PIN,
        .scl_io_num       = BME280_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(BME280_I2C_PORT, &cfg);
    if (ret != ESP_OK) return ret;
    return i2c_driver_install(BME280_I2C_PORT, cfg.mode, 0, 0, 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
   Kalibracja BME280
   ═══════════════════════════════════════════════════════════════════════════ */

static esp_err_t bme280_read_calib(void)
{
    uint8_t buf[26];

    /* blok 0x88..0xA1 - temperatura i ciśnienie */
    esp_err_t ret = i2c_read_regs(BME280_REG_CALIB00, buf, 26);
    if (ret != ESP_OK) return ret;

    s_calib.dig_T1 = (uint16_t)(buf[1]  << 8 | buf[0]);
    s_calib.dig_T2 = (int16_t) (buf[3]  << 8 | buf[2]);
    s_calib.dig_T3 = (int16_t) (buf[5]  << 8 | buf[4]);
    s_calib.dig_P1 = (uint16_t)(buf[7]  << 8 | buf[6]);
    s_calib.dig_P2 = (int16_t) (buf[9]  << 8 | buf[8]);
    s_calib.dig_P3 = (int16_t) (buf[11] << 8 | buf[10]);
    s_calib.dig_P4 = (int16_t) (buf[13] << 8 | buf[12]);
    s_calib.dig_P5 = (int16_t) (buf[15] << 8 | buf[14]);
    s_calib.dig_P6 = (int16_t) (buf[17] << 8 | buf[16]);
    s_calib.dig_P7 = (int16_t) (buf[19] << 8 | buf[18]);
    s_calib.dig_P8 = (int16_t) (buf[21] << 8 | buf[20]);
    s_calib.dig_P9 = (int16_t) (buf[23] << 8 | buf[22]);
    s_calib.dig_H1 = buf[25];

    /* blok 0xE1..0xE7 - wilgotność */
    uint8_t hbuf[7];
    ret = i2c_read_regs(BME280_REG_CALIB26, hbuf, 7);
    if (ret != ESP_OK) return ret;

    s_calib.dig_H2 = (int16_t)(hbuf[1] << 8 | hbuf[0]);
    s_calib.dig_H3 = hbuf[2];
    s_calib.dig_H4 = (int16_t)(hbuf[3] << 4 | (hbuf[4] & 0x0F));
    s_calib.dig_H5 = (int16_t)(hbuf[5] << 4 | (hbuf[4] >> 4));
    s_calib.dig_H6 = (int8_t)hbuf[6];

    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
   Kompensacja (wzory z datasheet Bosch)
   ═══════════════════════════════════════════════════════════════════════════ */

static float compensate_temperature(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)s_calib.dig_T1 << 1)))
                    * (int32_t)s_calib.dig_T2) >> 11;
    int32_t var2 = (((((adc_T >> 4) - (int32_t)s_calib.dig_T1)
                    * ((adc_T >> 4) - (int32_t)s_calib.dig_T1)) >> 12)
                    * (int32_t)s_calib.dig_T3) >> 14;
    s_t_fine = var1 + var2;
    return (float)((s_t_fine * 5 + 128) >> 8) / 100.0f;
}

static float compensate_pressure(int32_t adc_P)
{
    int64_t var1 = (int64_t)s_t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
    var2 += (var1 * (int64_t)s_calib.dig_P5) << 17;
    var2 += ((int64_t)s_calib.dig_P4) << 35;
    var1  = ((var1 * var1 * (int64_t)s_calib.dig_P3) >> 8)
          + ((var1 * (int64_t)s_calib.dig_P2) << 12);
    var1  = (((int64_t)1 << 47) + var1) * (int64_t)s_calib.dig_P1 >> 33;
    if (var1 == 0) return 0.0f;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)s_calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)s_calib.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)s_calib.dig_P7 << 4);
    return (float)p / 25600.0f;  /* Pa → hPa */
}

static float compensate_humidity(int32_t adc_H)
{
    int32_t v = s_t_fine - 76800;
    v = (((((adc_H << 14) - ((int32_t)s_calib.dig_H4 << 20)
          - ((int32_t)s_calib.dig_H5 * v)) + 16384) >> 15)
        * (((((((v * (int32_t)s_calib.dig_H6) >> 10)
             * (((v * (int32_t)s_calib.dig_H3) >> 11) + 32768)) >> 10)
            + 2097152) * (int32_t)s_calib.dig_H2 + 8192) >> 14));
    v -= ((((v >> 15) * (v >> 15)) >> 7) * (int32_t)s_calib.dig_H1) >> 4;
    v  = v < 0 ? 0 : (v > 419430400 ? 419430400 : v);
    return (float)(v >> 12) / 1024.0f;
}

/* ═══════════════════════════════════════════════════════════════════════════
   Odczyt surowych danych
   ═══════════════════════════════════════════════════════════════════════════ */

static esp_err_t bme280_read_raw(int32_t *adc_T, int32_t *adc_P, int32_t *adc_H)
{
    /* wymuś tryb forced - jeden pomiar, potem sensor idle */
    esp_err_t ret = i2c_write_reg(BME280_REG_CTRL_HUM,  0x01); /* x1 oversampling */
    if (ret != ESP_OK) return ret;
    ret = i2c_write_reg(BME280_REG_CTRL_MEAS, 0x25); /* T x1, P x1, forced mode */
    if (ret != ESP_OK) return ret;

    /* czekaj na koniec pomiaru (status bit measuring) */
    uint8_t status;
    do {
        vTaskDelay(pdMS_TO_TICKS(10));
        ret = i2c_read_regs(BME280_REG_STATUS, &status, 1);
        if (ret != ESP_OK) return ret;
    } while (status & 0x08);

    /* odczyt 8 bajtów od 0xF7 */
    uint8_t raw[8];
    ret = i2c_read_regs(BME280_REG_PRESS_MSB, raw, 8);
    if (ret != ESP_OK) return ret;

    *adc_P = (int32_t)((raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4));
    *adc_T = (int32_t)((raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4));
    *adc_H = (int32_t)((raw[6] << 8)  |  raw[7]);

    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
   MQTT - event handler
   ═══════════════════════════════════════════════════════════════════════════ */

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT połączony");
            xEventGroupSetBits(s_mqtt_events, MQTT_CONNECTED_BIT);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT rozłączony");
            xEventGroupClearBits(s_mqtt_events, MQTT_CONNECTED_BIT);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "MQTT opublikowano msg_id=%d",
                     ((esp_mqtt_event_handle_t)event_data)->msg_id);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT błąd");
            break;
        default:
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   API publiczne
   ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t bme280_mqtt_init(void)
{
    /* --- I2C --- */
    esp_err_t ret = i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Błąd inicjalizacji I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    /* --- sprawdź chip ID --- */
    uint8_t chip_id;
    ret = i2c_read_regs(BME280_REG_CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK || chip_id != BME280_CHIP_ID) {
        ESP_LOGE(TAG, "Nie znaleziono BME280 (id=0x%02X)", chip_id);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BME280 wykryty (chip_id=0x%02X)", chip_id);

    /* --- reset i kalibracja --- */
    i2c_write_reg(BME280_REG_RESET, BME280_RESET_VALUE);
    vTaskDelay(pdMS_TO_TICKS(10));

    ret = bme280_read_calib();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Błąd odczytu kalibracji: %s", esp_err_to_name(ret));
        return ret;
    }

    /* --- MQTT --- */
    s_mqtt_events = xEventGroupCreate();

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri  = MQTT_BROKER_URI,
        .broker.address.port = MQTT_BROKER_PORT,
        .credentials.client_id = MQTT_CLIENT_ID,
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_mqtt_client) {
        ESP_LOGE(TAG, "Błąd inicjalizacji klienta MQTT");
        return ESP_FAIL;
    }

    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);

    ret = esp_mqtt_client_start(s_mqtt_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Błąd startu MQTT: %s", esp_err_to_name(ret));
        return ret;
    }

    /* czekaj na połączenie max 10s */
    EventBits_t bits = xEventGroupWaitBits(s_mqtt_events, MQTT_CONNECTED_BIT,
                                           pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(10000));
    if (!(bits & MQTT_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "Timeout połączenia MQTT");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t bme280_mqtt_publish(bme280_data_t *out_data)
{
    /* odczyt surowych danych */
    int32_t adc_T, adc_P, adc_H;
    esp_err_t ret = bme280_read_raw(&adc_T, &adc_P, &adc_H);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Błąd odczytu BME280: %s", esp_err_to_name(ret));
        return ret;
    }

    /* kompensacja */
    float temp  = compensate_temperature(adc_T);
    float press = compensate_pressure(adc_P);
    float hum   = compensate_humidity(adc_H);

    ESP_LOGI(TAG, "T=%.2f°C  H=%.2f%%  P=%.2fhPa", temp, hum, press);

    /* opcjonalnie zwróć dane do wywołującego */
    if (out_data) {
        out_data->temperature = temp;
        out_data->humidity    = hum;
        out_data->pressure    = press;
    }

    /* zbuduj JSON */
    /* sprawdź czy połączony */
        EventBits_t bits = xEventGroupGetBits(s_mqtt_events);
        if (!(bits & MQTT_CONNECTED_BIT)) {
            ESP_LOGW(TAG, "MQTT niepołączony, pomijam publikację");
            return ESP_ERR_INVALID_STATE;
        }

        /* każdy pomiar na osobny subtopic, payload = sama wartość */
        static const struct {
            const char *subtopic;
            float       value;
        } measurements[] = {
            { MQTT_TOPIC "/temperature", temp  },
            { MQTT_TOPIC "/humidity",    hum   },
            { MQTT_TOPIC "/pressure",    press },
        };

        char payload[16];
        for (int i = 0; i < 3; i++) {
            snprintf(payload, sizeof(payload), "{\"value\":%.2f}", measurements[i].value);

            int msg_id = esp_mqtt_client_publish(s_mqtt_client,
                                                 measurements[i].subtopic,
                                                 payload, 0,
                                                 1,   /* QoS 1 */
                                                 0);  /* retain 0 */
            if (msg_id < 0) {
                ESP_LOGE(TAG, "Błąd publikacji MQTT (%s)", measurements[i].subtopic);
                return ESP_FAIL;
            }
            ESP_LOGI(TAG, "Opublikowano [%s]: %s", measurements[i].subtopic, payload);
        }

        return ESP_OK;
}

void bme280_mqtt_deinit(void)
{
    if (s_mqtt_client) {
        esp_mqtt_client_stop(s_mqtt_client);
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
    }
    if (s_mqtt_events) {
        vEventGroupDelete(s_mqtt_events);
        s_mqtt_events = NULL;
    }
    i2c_driver_delete(BME280_I2C_PORT);
}
