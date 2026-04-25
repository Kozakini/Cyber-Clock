/*
 * main.c — e-Paper 5.79" Dashboard
 *  - NTP  → godzina + data
 *  - OpenWeatherMap API → temperatura + opis pogody
 *  - Wyświetlanie UI co 5 minut (full refresh)
 *
 * Wymagane komponenty ESP-IDF:
 *   esp_wifi, esp_http_client, esp_sntp, json (cJSON),
 *   driver/spi_master, driver/gpio, nvs_flash
 *
 * W menuconfig:
 *   WIFI_SSID, WIFI_PASSWORD, OWM_API_KEY, OWM_CITY_ID
 *   (albo wpisz bezpośrednio poniżej)
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "bme280_mqtt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "nvs_flash.h"

#include "esp_http_client.h"
#include "cJSON.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "epd_ui.h"

// ============================================================
//  KONFIGURACJA — uzupełnij!
// ============================================================
#define WIFI_SSID       ""
#define WIFI_PASSWORD   ""

// Klucz API z openweathermap.org (darmowy plan)
#define OWM_API_KEY     ""
// ID miasta — szukaj na openweathermap.org/find
// Białystok = 776069, Warszawa = 756135, Kraków = 3094802
#define OWM_CITY_ID     "776069"
// Timezone offset w sekundach (np. CET+1 = 3600, CEST+2 = 7200)
#define TZ_OFFSET_SEC   3600

// Interwał odświeżania (sekundy) — minimum 180s zgodnie z docs!
#define REFRESH_INTERVAL_SEC  180

// ============================================================
//  Piny SPI — takie same jak w poprzednim pliku
// ============================================================
#define PIN_MOSI   14
#define PIN_CLK    41
#define PIN_CS     13
#define PIN_DC     12
#define PIN_RST    11
#define PIN_BUSY   10
#define PIN_PWR     9

// ============================================================
//  Stałe EPD (wewnętrzne — ten sam kontroler)
// ============================================================
#define EPD_W_HALF   50
#define EPD_BYTES    (EPD_W_HALF * EPD_H)

static const char *TAG = "EPD_DASH";

// ============================================================
//  SPI handle
// ============================================================
static spi_device_handle_t spi;

// ============================================================
//  WiFi event group
// ============================================================
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// ============================================================
//  Bufor HTTP response
// ============================================================
#define HTTP_BUF_SIZE 2048
static char http_buf[HTTP_BUF_SIZE];
static int  http_buf_len;

// ============================================================
//  Globalne dane pogody
// ============================================================
static ui_weather_t g_weather = {
    .temp_c      = 0.0f,
    .icon        = WEATHER_UNKNOWN,
    .description = "brak danych",
    .city        = OWM_CITY_ID,
};

// ============================================================
//  SPI / EPD helpers (identyczne jak w teście)
// ============================================================
static void epd_cmd(uint8_t c) {
    gpio_set_level(PIN_DC, 0);
    spi_transaction_t t = { .length = 8, .tx_buffer = &c };
    spi_device_polling_transmit(spi, &t);
}
static void epd_dat(uint8_t d) {
    gpio_set_level(PIN_DC, 1);
    spi_transaction_t t = { .length = 8, .tx_buffer = &d };
    spi_device_polling_transmit(spi, &t);
}
static void epd_wait_busy(void) {
    while (gpio_get_level(PIN_BUSY) == 1)
        vTaskDelay(pdMS_TO_TICKS(10));
}
static void epd_reset(void) {
    gpio_set_level(PIN_RST, 0); vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(PIN_RST, 1); vTaskDelay(pdMS_TO_TICKS(200));
}

// ============================================================
//  Hardware init
// ============================================================
static void hw_init(void) {
    gpio_config_t pwr = {
        .pin_bit_mask = 1ULL<<PIN_PWR,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&pwr);
    gpio_set_level(PIN_PWR, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    gpio_config_t o = {
        .pin_bit_mask = (1ULL<<PIN_DC)|(1ULL<<PIN_RST)|(1ULL<<PIN_CS),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&o);

    gpio_config_t i = {
        .pin_bit_mask = 1ULL<<PIN_BUSY,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&i);

    gpio_set_level(PIN_RST, 1);
    gpio_set_level(PIN_DC,  1);
    gpio_set_level(PIN_CS,  1);

    spi_bus_config_t bus = {
        .mosi_io_num   = PIN_MOSI,
        .miso_io_num   = -1,
        .sclk_io_num   = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 4000000,
        .mode           = 0,
        .spics_io_num   = PIN_CS,
        .queue_size     = 1,
    };
    spi_bus_add_device(SPI2_HOST, &dev, &spi);
    ESP_LOGI(TAG, "HW init OK");
}

// ============================================================
//  EPD init (po każdym sleep!)
// ============================================================
static void epd_init_display(void) {
    gpio_set_level(PIN_PWR, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    epd_reset();
    epd_wait_busy();

    epd_cmd(0x12); epd_wait_busy(); // SW reset

    epd_cmd(0x11); epd_dat(0x01);   // data entry mode

    // M part okno RAM
    epd_cmd(0x44); epd_dat(0x00); epd_dat(0x31);
    epd_cmd(0x45); epd_dat(0x0F); epd_dat(0x01); epd_dat(0x00); epd_dat(0x00);
    epd_cmd(0x4E); epd_dat(0x00);
    epd_cmd(0x4F); epd_dat(0x0F); epd_dat(0x01);
    epd_wait_busy();

    // S part okno RAM
    epd_cmd(0x91); epd_dat(0x00);
    epd_cmd(0xC4); epd_dat(0x31); epd_dat(0x00);
    epd_cmd(0xC5); epd_dat(0x0F); epd_dat(0x01); epd_dat(0x00); epd_dat(0x00);
    epd_cmd(0xCE); epd_dat(0x31);
    epd_cmd(0xCF); epd_dat(0x0F); epd_dat(0x01);
    epd_wait_busy();
}

// ============================================================
//  Wyślij canvas na ekran
// ============================================================
static uint8_t tmp_m[EPD_BYTES];
static uint8_t tmp_s[EPD_BYTES];

static void epd_show_canvas(void) {
    // M part: pierwsze 50 bajtów każdego wiersza
    // M part: pierwsze 50 bajtów
    for (int y = 0; y < EPD_H; y++)
        memcpy(tmp_m + y * EPD_W_HALF,
               canvas + y * FULL_W_BYTES, EPD_W_HALF);

    // S part: bajty 50..99 (nie 49!)
    for (int y = 0; y < EPD_H; y++)
        memcpy(tmp_s + y * EPD_W_HALF,
               canvas + y * FULL_W_BYTES + EPD_W_HALF, EPD_W_HALF);
    epd_cmd(0x24);
    for (int i = 0; i < EPD_BYTES; i++) epd_dat(tmp_m[i]);
    epd_cmd(0x26);
    for (int i = 0; i < EPD_BYTES; i++) epd_dat(0x00);
    epd_cmd(0xA4);
    for (int i = 0; i < EPD_BYTES; i++) epd_dat(tmp_s[i]);
    epd_cmd(0xA6);
    for (int i = 0; i < EPD_BYTES; i++) epd_dat(0x00);

    epd_cmd(0x22); epd_dat(0xF7);
    epd_cmd(0x20);
    epd_wait_busy();
    ESP_LOGI(TAG, "ekran odswiezony");
}

static void epd_sleep_display(void) {
    epd_cmd(0x10); epd_dat(0x01);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_PWR, 0);
}

// ============================================================
//  WiFi
// ============================================================
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi rozlaczone, ponawiam...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void) {
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    wifi_config_t wcfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Czekaj na połączenie (max 15s)
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, pdMS_TO_TICKS(15000));
}

// ============================================================
//  NTP — synchronizacja czasu
// ============================================================
static void ntp_sync(void) {
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");
    esp_sntp_init();

    int retry = 0;
    while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED && retry < 40) {
        vTaskDelay(pdMS_TO_TICKS(500));
        retry++;
    }

    if (sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED)
        ESP_LOGI(TAG, "NTP zsynchronizowany");
    else
        ESP_LOGW(TAG, "NTP timeout, uzywam czasu systemowego");
}
// ============================================================
//  Pobranie czasu z systemu
// ============================================================
static void get_time(ui_time_t *out) {
    time_t now = time(NULL);
    now += TZ_OFFSET_SEC;           // ręczne przesunięcie strefy
    struct tm *tm_info = gmtime(&now);

    out->hour    = tm_info->tm_hour;
    out->minute  = tm_info->tm_min;
    out->second  = tm_info->tm_sec;
    out->day     = tm_info->tm_mday;
    out->month   = tm_info->tm_mon + 1;
    out->year    = tm_info->tm_year + 1900;
    out->weekday = tm_info->tm_wday; // 0=Nd
}

// ============================================================
//  OpenWeatherMap — parsowanie kodu pogody na ikonę
// ============================================================
static weather_icon_t owm_id_to_icon(int id) {
    // https://openweathermap.org/weather-conditions
    if (id == 800)                  return WEATHER_SUNNY;
    if (id >= 801 && id <= 804)     return WEATHER_CLOUDY;
    if (id >= 300 && id < 600)      return WEATHER_RAIN;
    if (id >= 600 && id < 700)      return WEATHER_SNOW;
    if (id >= 200 && id < 300)      return WEATHER_STORM;
    if (id >= 700 && id < 800)      return WEATHER_FOG;
    return WEATHER_UNKNOWN;
}

// ============================================================
//  HTTP event handler (zbiera odpowiedź do bufora)
// ============================================================
static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA:
        if (http_buf_len + evt->data_len < HTTP_BUF_SIZE - 1) {
            memcpy(http_buf + http_buf_len, evt->data, evt->data_len);
            http_buf_len += evt->data_len;
        }
        break;
    default:
        break;
    }
    return ESP_OK;
}

// ============================================================
//  Pobranie pogody z OpenWeatherMap
// ============================================================
static void fetch_weather(void) {
    char url[256];
    snprintf(url, sizeof(url),
        "http://api.openweathermap.org/data/2.5/weather"
        "?id=%s&appid=%s&units=metric&lang=pl",
        OWM_CITY_ID, OWM_API_KEY);

    memset(http_buf, 0, sizeof(http_buf));
    http_buf_len = 0;

    esp_http_client_config_t cfg = {
        .url            = url,
        .event_handler  = http_event_handler,
        .timeout_ms     = 8000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_err_t err = esp_http_client_perform(client);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HTTP blad: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return;
    }

    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (status != 200) {
        ESP_LOGW(TAG, "OWM HTTP %d", status);
        return;
    }

    // Parsowanie JSON
    cJSON *root = cJSON_Parse(http_buf);
    if (!root) {
        ESP_LOGW(TAG, "JSON parse error");
        return;
    }

    // Temperatura
    cJSON *main_obj = cJSON_GetObjectItem(root, "main");
    if (main_obj) {
        cJSON *temp = cJSON_GetObjectItem(main_obj, "temp");
        if (cJSON_IsNumber(temp))
            g_weather.temp_c = (float)temp->valuedouble;
    }

    // Opis i kod pogody
    cJSON *weather_arr = cJSON_GetObjectItem(root, "weather");
    if (cJSON_IsArray(weather_arr)) {
        cJSON *first = cJSON_GetArrayItem(weather_arr, 0);
        if (first) {
            cJSON *id_item  = cJSON_GetObjectItem(first, "id");
            cJSON *desc     = cJSON_GetObjectItem(first, "description");
            if (cJSON_IsNumber(id_item))
                g_weather.icon = owm_id_to_icon(id_item->valueint);
            if (cJSON_IsString(desc)) {
                strncpy(g_weather.description, desc->valuestring,
                        sizeof(g_weather.description) - 1);
                // Pierwsza litera duża
                if (g_weather.description[0] >= 'a' && g_weather.description[0] <= 'z')
                    g_weather.description[0] -= 32;
            }
        }
    }

    // Nazwa miasta
    cJSON *name = cJSON_GetObjectItem(root, "name");
    if (cJSON_IsString(name))
        strncpy(g_weather.city, name->valuestring, sizeof(g_weather.city) - 1);

    cJSON_Delete(root);
    ESP_LOGI(TAG, "Pogoda: %.1f°C %s (%s)",
             g_weather.temp_c, g_weather.description, g_weather.city);
}

// ============================================================
//  Przykładowy obrazek BMP (128×200 px, 1-bit)
//  ZAMIEŃ na swój obrazek — np. logo, ikonę, awatar
//  Dane: 1 bit/piksel, bit7=lewy piksel, 1=biały 0=czarny
//
//  Konwersja z pliku:
//    python3 bmp_to_c.py twoj_obrazek.bmp > bmp_data.h
//    (skrypt w bmp_to_c.py poniżej — patrz komentarz na końcu)
// ============================================================
#include "bmp_data.h"
  // <- wstaw tutaj dane swojego BMP
//  Jeśli bmp_data = {0xFF} (jeden bajt 0xFF), ui_bmp() narysuje
//  tylko jeden piksel biały — w praktyce wstaw prawdziwe dane

// ============================================================
//  Główna pętla
// ============================================================
void app_main(void) {
    ESP_LOGI(TAG, "Start EPD Dashboard");

    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    bme280_mqtt_init();  // raz na starcie



    // Sprzęt
    hw_init();
    epd_init_display();

    // Pierwsze czyszczenie ekranu
    uint8_t *m = tmp_m, *s = tmp_s;
    memset(m, 0xFF, EPD_BYTES); memset(s, 0xFF, EPD_BYTES);
    epd_cmd(0x24); for (int i=0;i<EPD_BYTES;i++) epd_dat(m[i]);
    epd_cmd(0x26); for (int i=0;i<EPD_BYTES;i++) epd_dat(0x00);
    epd_cmd(0xA4); for (int i=0;i<EPD_BYTES;i++) epd_dat(s[i]);
    epd_cmd(0xA6); for (int i=0;i<EPD_BYTES;i++) epd_dat(0x00);
    epd_cmd(0x22); epd_dat(0xF7); epd_cmd(0x20); epd_wait_busy();
    epd_sleep_display();
    vTaskDelay(pdMS_TO_TICKS(2000));

    // WiFi + NTP
    wifi_init();
    ntp_sync();

    // Pierwsze pobranie pogody
    fetch_weather();

    // ---- Pętla główna ----
    int  cycle         = 0;
    // Pogodę odświeżamy co 10 cykli = co 50 min (ograniczenie API: 60 req/h)


    if (sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED)
        ESP_LOGI(TAG, "NTP OK");
    else
        ESP_LOGW(TAG, "NTP niesync");
    while (1) {
        // Czas
        ui_time_t now;
        get_time(&now);
        bme280_data_t data;
        bme280_mqtt_publish(&data);  // odczyt + publikacja JSON

        // data.temperature, data.humidity, data.pressure
        // możesz przekazać do swojego kodu e-Paper

        vTaskDelay(pdMS_TO_TICKS(180000));  // 180s
        // Rysuj UI
        ui_draw(&now, &g_weather, bmp_data, BMP_W, BMP_H);

        // Wyślij na ekran
        epd_init_display();
        epd_show_canvas();
        epd_sleep_display();

        cycle++;

        // Odświeżaj pogodę rzadziej
        if (cycle == 10) {
            cycle = 0;
            // Sprawdź czy WiFi nadal działa
            EventBits_t bits = xEventGroupGetBits(wifi_event_group);
            if (bits & WIFI_CONNECTED_BIT)
                fetch_weather();
            else
                ESP_LOGW(TAG, "WiFi niedostepne — pomijam pogode");
        }

        ESP_LOGI(TAG, "Czekam %d sekund...", REFRESH_INTERVAL_SEC);
        vTaskDelay(pdMS_TO_TICKS((uint32_t)REFRESH_INTERVAL_SEC * 1000));
    }
}

/*
 * ============================================================
 *  bmp_to_c.py — konwerter BMP/PNG → tablica C
 * ============================================================
 *
 * #!/usr/bin/env python3
 * import sys
 * from PIL import Image
 *
 * img = Image.open(sys.argv[1]).convert("1")  # 1-bit
 * w, h = img.size
 * print(f"// {w}x{h} px")
 * print(f"#define BMP_W {w}")
 * print(f"#define BMP_H {h}")
 * print("static const uint8_t bmp_data[] = {")
 * pix = img.load()
 * row_bytes = (w + 7) // 8
 * for y in range(h):
 *     row = []
 *     for bx in range(row_bytes):
 *         byte = 0
 *         for bit in range(8):
 *             x = bx * 8 + bit
 *             if x < w:
 *                 # PIL: 0=czarny 255=biały → e-Paper: 0=czarny 1=biały (bit=1)
 *                 if pix[x, y] != 0:
 *                     byte |= (0x80 >> bit)
 *             else:
 *                 byte |= (0x80 >> bit)  # padding = biały
 *         row.append(f"0x{byte:02X}")
 *     print("    " + ",".join(row) + ",")
 * print("};")
 *
 * ============================================================
 */
