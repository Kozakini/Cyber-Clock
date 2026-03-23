#pragma once
#include <stdint.h>

// Wymiary ekranu
#define EPD_W       792
#define EPD_H       272

// Strefy layoutu (px od lewej)
#define ZONE_TIME_W     300
#define ZONE_WEATHER_W  230
#define ZONE_BMP_W      262  // strefa obrazka (reszta do 792)

// Marginesy wewnętrzne stref
#define MARGIN          12

// Bufor pełnego ekranu (1 bit/px, 1=biały 0=czarny)
#define FULL_W_BYTES    99    // ceil(792/8)
#define FULL_SIZE       (FULL_W_BYTES * EPD_H)

extern uint8_t canvas[FULL_SIZE];

// ----- API rysowania -----
void ui_fill(uint8_t white);
void ui_px(int x, int y, uint8_t white);
void ui_hline(int x, int y, int len, uint8_t white);
void ui_vline(int x, int y, int len, uint8_t white);
void ui_rect_fill(int x, int y, int w, int h, uint8_t white);
void ui_rect_border(int x, int y, int w, int h, uint8_t white);

// Tekst z wbudowaną czcionką proporcjonalną
// scale: 1=8px, 2=16px, 3=24px, 4=32px itd.
void ui_text(int x, int y, const char *s, int scale, uint8_t white);
// Zwraca szerokość napisu w px (z danym scale)
int  ui_text_width(const char *s, int scale);

// Rysuj ikonę pogody (wbudowane wzorce 32x32 → 1-bit)
typedef enum {
    WEATHER_SUNNY = 0,
    WEATHER_CLOUDY,
    WEATHER_RAIN,
    WEATHER_SNOW,
    WEATHER_STORM,
    WEATHER_FOG,
    WEATHER_UNKNOWN,
} weather_icon_t;
void ui_weather_icon(int x, int y, int size, weather_icon_t icon);
void ui_text_wrap(int x, int y, const char *s, int scale, int max_width, uint8_t white);
// Rysuj obrazek BMP (1-bit, monochrome) z bufora
// buf: dane BMP (bez nagłówka), width_bytes = ceil(w/8)
void ui_bmp(int x, int y, int w, int h, const uint8_t *buf);

// ----- Główna funkcja rysowania całego UI -----
typedef struct {
    int   hour;
    int   minute;
    int   second;
    int   day;
    int   month;
    int   year;
    int   weekday;     // 0=Nd, 1=Pn, ..., 6=Sb
} ui_time_t;

typedef struct {
    float temp_c;           // temperatura w °C
    weather_icon_t icon;    // ikona
    char  description[32];  // opis słowny np. "Zachmurzenie"
    char  city[32];
} ui_weather_t;

// bmp_buf: bufor obrazka 1-bit (NULL = nie rysuj obrazka)
// bmp_w, bmp_h: wymiary obrazka w pikselach
void ui_draw(const ui_time_t *t, const ui_weather_t *w,
             const uint8_t *bmp_buf, int bmp_w, int bmp_h);
