#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "EPD_TEST";

// =============================================================
//  PINY — ZMIEŃ NA SWOJE
// =============================================================
#define PIN_MOSI   14
#define PIN_CLK    41
#define PIN_CS     13
#define PIN_DC      12
#define PIN_RST     11
#define PIN_BUSY    10
#define PIN_PWR     9

// =============================================================
//  Wymiary
// =============================================================
#define EPD_W      792
#define EPD_H      272
#define EPD_W_HALF  50   // ceil(396/8) zaokr. do parzystej 16px = 50 bajtów
#define EPD_BYTES  (EPD_W_HALF * EPD_H)  // 13600 na każdą połowę

static spi_device_handle_t spi;

// ---------------------------------------------------------------
//  SPI
// ---------------------------------------------------------------
static void cmd(uint8_t c) {
    gpio_set_level(PIN_DC, 0);
    spi_transaction_t t = { .length = 8, .tx_buffer = &c };
    spi_device_polling_transmit(spi, &t);
}

static void dat(uint8_t d) {
    gpio_set_level(PIN_DC, 1);
    spi_transaction_t t = { .length = 8, .tx_buffer = &d };
    spi_device_polling_transmit(spi, &t);
}

static void wait_busy(void) {
    // BUSY: 0 = zajęty, 1 = gotowy
    while (gpio_get_level(PIN_BUSY) == 1)
        vTaskDelay(pdMS_TO_TICKS(10));
}

static void reset(void) {
    gpio_set_level(PIN_RST, 0); vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(PIN_RST, 1); vTaskDelay(pdMS_TO_TICKS(200));
}

// ---------------------------------------------------------------
//  Init sprzętu
// ---------------------------------------------------------------
static void hw_init(void) {

    gpio_config_t pwr_cfg = {
        .pin_bit_mask = (1ULL << PIN_PWR),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&pwr_cfg);
    gpio_set_level(PIN_PWR, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    // GPIO
    gpio_config_t o = {
        .pin_bit_mask = (1ULL<<PIN_DC)|(1ULL<<PIN_RST)|(1ULL<<PIN_CS),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&o);
    gpio_config_t i = {
        .pin_bit_mask = (1ULL<<PIN_BUSY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&i);

    gpio_set_level(PIN_RST, 1);
    gpio_set_level(PIN_DC,  1);
    gpio_set_level(PIN_CS,  1);

    // SPI
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
        .clock_speed_hz = 4 * 1000 * 1000,  // 4 MHz
        .mode           = 0,
        .spics_io_num   = PIN_CS,
        .queue_size     = 1,
    };
    spi_bus_add_device(SPI2_HOST, &dev, &spi);

    ESP_LOGI(TAG, "HW init OK, SPI 4MHz");
}

// ---------------------------------------------------------------
//  Init wyświetlacza
// ---------------------------------------------------------------
static void epd_init(void) {
    gpio_set_level(PIN_PWR, 1);  // wlacz zasilanie
    vTaskDelay(pdMS_TO_TICKS(100));

    reset();
    wait_busy();

    cmd(0x12);       // software reset
    wait_busy();

    cmd(0x11);       // data entry mode
    dat(0x01);       // X increment, Y increment

    // M part: okno RAM X (0x00 .. 0x31 = 50 bajtów = 400px)
    cmd(0x44); dat(0x00); dat(0x31);
    // M part: okno RAM Y (0x010F .. 0x0000 = 272 linii)
    cmd(0x45); dat(0x0F); dat(0x01); dat(0x00); dat(0x00);
    // Kursor startowy M
    cmd(0x4E); dat(0x00);
    cmd(0x4F); dat(0x0F); dat(0x01);
    wait_busy();

    // Przełącz na S part (prawa połowa)
    cmd(0x91); dat(0x00);
    // S part: okno RAM X (odwrócone: 0x31 .. 0x00)
    cmd(0xC4); dat(0x31); dat(0x00);
    // S part: okno RAM Y
    cmd(0xC5); dat(0x0F); dat(0x01); dat(0x00); dat(0x00);
    // Kursor startowy S
    cmd(0xCE); dat(0x31);
    cmd(0xCF); dat(0x0F); dat(0x01);
    wait_busy();

    ESP_LOGI(TAG, "EPD init OK");
}

// ---------------------------------------------------------------
//  Wyślij obraz + odśwież
//  buf_m: 13600 bajtów — lewa połowa (M part)
//  buf_s: 13600 bajtów — prawa połowa (S part)
//  Jeśli buf_s == NULL, używa buf_m dla obu stron
// ---------------------------------------------------------------
static void epd_show(const uint8_t *buf_m, const uint8_t *buf_s) {
    const uint8_t *s = buf_s ? buf_s : buf_m;

    cmd(0x24);  // zapis do RAM (bieżąca klatka, M part)
    for (int i = 0; i < EPD_BYTES; i++) dat(buf_m[i]);

    cmd(0x26);  // poprzednia klatka M (nieużywana — zera)
    for (int i = 0; i < EPD_BYTES; i++) dat(0x00);

    cmd(0xA4);  // zapis do RAM (bieżąca klatka, S part)
    for (int i = 0; i < EPD_BYTES; i++) dat(s[i]);

    cmd(0xA6);  // poprzednia klatka S (nieużywana)
    for (int i = 0; i < EPD_BYTES; i++) dat(0x00);

    // Uruchom odświeżanie
    cmd(0x22); dat(0xF7);
    cmd(0x20);
    wait_busy();

    ESP_LOGI(TAG, "odświeżanie zakończone");
}

// ---------------------------------------------------------------
//  Wyczyść do bieli
// ---------------------------------------------------------------
static void epd_clear(void) {
    cmd(0x24); for (int i = 0; i < EPD_BYTES; i++) dat(0xFF);
    cmd(0x26); for (int i = 0; i < EPD_BYTES; i++) dat(0x00);
    cmd(0xA4); for (int i = 0; i < EPD_BYTES; i++) dat(0xFF);
    cmd(0xA6); for (int i = 0; i < EPD_BYTES; i++) dat(0x00);
    cmd(0x22); dat(0xF7);
    cmd(0x20);
    wait_busy();
    ESP_LOGI(TAG, "ekran wyczyszczony");
}

// ---------------------------------------------------------------
//  Sleep — OBOWIĄZKOWY gdy ekran nie odświeża!
// ---------------------------------------------------------------
static void epd_sleep(void) {
    cmd(0x10); dat(0x01);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_PWR, 0);  // wytnij zasilanie
    ESP_LOGI(TAG, "epd sleep");
}

// ---------------------------------------------------------------
//  Minimalna czcionka 8x8 — tylko co potrzeba do napisu testowego
// ---------------------------------------------------------------
static const uint8_t F8[128][8] = {
    [' '] = {0,0,0,0,0,0,0,0},
    ['!'] = {0x18,0x18,0x18,0x18,0x00,0x00,0x18,0x00},
    ['0'] = {0x3C,0x66,0x6E,0x76,0x66,0x66,0x3C,0x00},
    ['1'] = {0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00},
    ['2'] = {0x3C,0x66,0x06,0x0C,0x18,0x30,0x7E,0x00},
    ['3'] = {0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00},
    ['4'] = {0x0C,0x1C,0x3C,0x6C,0x7E,0x0C,0x0C,0x00},
    ['5'] = {0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00},
    ['6'] = {0x3C,0x60,0x7C,0x66,0x66,0x66,0x3C,0x00},
    ['7'] = {0x7E,0x06,0x0C,0x18,0x30,0x30,0x30,0x00},
    ['8'] = {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00},
    ['9'] = {0x3C,0x66,0x66,0x3E,0x06,0x0C,0x38,0x00},
    ['A'] = {0x18,0x3C,0x66,0x7E,0x66,0x66,0x66,0x00},
    ['B'] = {0x7C,0x66,0x66,0x7C,0x66,0x66,0x7C,0x00},
    ['C'] = {0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00},
    ['D'] = {0x78,0x6C,0x66,0x66,0x66,0x6C,0x78,0x00},
    ['E'] = {0x7E,0x60,0x60,0x78,0x60,0x60,0x7E,0x00},
    ['F'] = {0x7E,0x60,0x60,0x78,0x60,0x60,0x60,0x00},
    ['G'] = {0x3C,0x66,0x60,0x6E,0x66,0x66,0x3C,0x00},
    ['H'] = {0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x00},
    ['I'] = {0x3C,0x18,0x18,0x18,0x18,0x18,0x3C,0x00},
    ['J'] = {0x1E,0x0C,0x0C,0x0C,0x0C,0x6C,0x38,0x00},
    ['K'] = {0x66,0x6C,0x78,0x70,0x78,0x6C,0x66,0x00},
    ['L'] = {0x60,0x60,0x60,0x60,0x60,0x60,0x7E,0x00},
    ['M'] = {0x63,0x77,0x7F,0x6B,0x63,0x63,0x63,0x00},
    ['N'] = {0x66,0x76,0x7E,0x6E,0x66,0x66,0x66,0x00},
    ['O'] = {0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00},
    ['P'] = {0x7C,0x66,0x66,0x7C,0x60,0x60,0x60,0x00},
    ['R'] = {0x7C,0x66,0x66,0x7C,0x6C,0x66,0x66,0x00},
    ['S'] = {0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00},
    ['T'] = {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00},
    ['U'] = {0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00},
    ['W'] = {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00},
    ['X'] = {0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00},
    ['Y'] = {0x66,0x66,0x3C,0x18,0x18,0x18,0x18,0x00},
    ['Z'] = {0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00},
    ['-'] = {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00},
};

// ---------------------------------------------------------------
//  Bufor 792×272 — jeden bajt = 8 pikseli, 1=biały 0=czarny
//  Rozmiar: 99 * 272 = 26928 bajtów
//  Bufor pełnego ekranu — epd_show() samo wytnie połowy
// ---------------------------------------------------------------
#define FULL_W_BYTES  99   // ceil(792/8)
#define FULL_SIZE     (FULL_W_BYTES * EPD_H)

static uint8_t canvas[FULL_SIZE];

static void px(int x, int y, uint8_t c) {
    if (x < 0 || x >= EPD_W || y < 0 || y >= EPD_H) return;
    int b = y * FULL_W_BYTES + (x >> 3);
    int k = 7 - (x & 7);
    if (c) canvas[b] |=  (1<<k);
    else   canvas[b] &= ~(1<<k);
}

static void fill(uint8_t c) { memset(canvas, c ? 0xFF : 0x00, FULL_SIZE); }

static void text(int x, int y, const char *s, int scale, uint8_t color) {
    for (; *s; s++, x += (8*scale + scale)) {
        uint8_t idx = (uint8_t)*s;
        if (idx > 127) continue;
        for (int r = 0; r < 8; r++)
            for (int c2 = 0; c2 < 8; c2++)
                if (F8[idx][r] & (0x80 >> c2))
                    for (int sy = 0; sy < scale; sy++)
                        for (int sx = 0; sx < scale; sx++)
                            px(x + c2*scale + sx, y + r*scale + sy, color);
    }
}

static void rect(int x, int y, int w, int h, uint8_t c) {
    for (int i = x; i < x+w; i++) { px(i, y, c); px(i, y+h-1, c); }
    for (int i = y; i < y+h; i++) { px(x, i, c); px(x+w-1, i, c); }
}

// ---------------------------------------------------------------
//  Wrapper epd_show() dla pełnego bufora (canvas)
//  Wytnij M part (bajty 0..49 każdego wiersza)
//  i S part (bajty 49..98 każdego wiersza, odwrócone dla S)
// ---------------------------------------------------------------
static uint8_t tmp_m[EPD_BYTES];
static uint8_t tmp_s[EPD_BYTES];

static void show_canvas(void) {
    // M part: pierwsze EPD_W_HALF (50) bajtów każdego wiersza
    for (int y = 0; y < EPD_H; y++)
        memcpy(tmp_m + y * EPD_W_HALF, canvas + y * FULL_W_BYTES, EPD_W_HALF);

    // S part: bajty 49..98 każdego wiersza (prawa połowa)
    // Offset = EPD_W_HALF - 1 = 49, bo S zaczyna się od bajtu nakładającego się
    for (int y = 0; y < EPD_H; y++)
        memcpy(tmp_s + y * EPD_W_HALF,
               canvas + y * FULL_W_BYTES + (EPD_W_HALF - 1), EPD_W_HALF);

    epd_show(tmp_m, tmp_s);
}

// ---------------------------------------------------------------
//  app_main
// ---------------------------------------------------------------
void app_main(void) {
    ESP_LOGI(TAG, "EPD test start");

    hw_init();
    epd_init();

    // --- Krok 1: Wyczyść ---
    ESP_LOGI(TAG, "1/4 clear");
    epd_clear();
    epd_sleep();

    vTaskDelay(pdMS_TO_TICKS(2000));

    // --- Krok 2: Szachownica (test pikseli) ---
    ESP_LOGI(TAG, "2/4 szachownica");
    epd_init();   // po sleep zawsze re-init!

    fill(1);
    for (int y = 0; y < EPD_H; y++)
        for (int x = 0; x < EPD_W; x++)
            if (((x/32) + (y/32)) % 2 == 0) px(x, y, 0);

    show_canvas();
    epd_sleep();

    vTaskDelay(pdMS_TO_TICKS(2000));

    // --- Krok 3: Prosty napis ---
    ESP_LOGI(TAG, "3/4 napis");
    epd_init();

    fill(1);  // biale tlo

    rect(10, 10, EPD_W - 20, EPD_H - 20, 0);        // ramka zewn.
    rect(14, 14, EPD_W - 28, EPD_H - 28, 0);        // ramka wewn.

    text(40,  50, "HELLO WORLD",       4, 0);
    text(40, 110, "EPD 5.79 TEST OK",  3, 0);
    text(40, 160, "ESP32-S3  792x272", 2, 0);
    text(40, 195, "SPI 4MHZ  1-BIT BW",2, 0);

    show_canvas();
    epd_sleep();

    vTaskDelay(pdMS_TO_TICKS(2000));

    // --- Krok 4: Paski (test kontrastu) ---
    ESP_LOGI(TAG, "4/4 paski");
    epd_init();

    fill(1);
    for (int y = 0; y < EPD_H; y += 32) {
        for (int x = y; x < y + 16 && x < EPD_H; x++)
            for (int col = 10; col < EPD_W - 10; col++)
                px(col, x, 0);
    }
    text(40, 120, "KONIEC TESTU", 3, 0);

    show_canvas();
    epd_sleep();

    vTaskDelay(pdMS_TO_TICKS(2000));
    epd_init();   // po sleep zawsze re-init!
    epd_clear();
    epd_sleep();  // usypiamy na czysto
    // Pętla idle — ekran śpi, nic nie robimy

    ESP_LOGI(TAG, "wszystkie testy zakonczone - EPD spi");
    while (1) vTaskDelay(pdMS_TO_TICKS(60000));
}
