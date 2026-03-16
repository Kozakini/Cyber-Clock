/*
 * ============================================================
 *  CZUJNIK BME280 NA ESP32 – KOD Z KOMENTARZAMI DLA NOOBÓW
 * ============================================================
 *
 *  Co robi ten program?
 *  --------------------
 *  Odczytuje temperaturę, ciśnienie i wilgotność z czujnika
 *  BME280 przez magistralę I2C i wypisuje wyniki na konsolę
 *  co 2 sekundy.
 *
 *  Co to jest I2C?
 *  ---------------
 *  I2C (ang. Inter-Integrated Circuit) to protokół komunikacji
 *  między układami elektronicznymi. Potrzebuje tylko 2 przewodów:
 *    - SDA (dane)
 *    - SCL (zegar)
 *  Mikrokontroler (ESP32) jest "masterem" i wysyła rozkazy do
 *  czujnika (BME280), który jest "slave'em".
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"   // System operacyjny czasu rzeczywistego dla ESP32
#include "freertos/task.h"       // Funkcje do zarządzania zadaniami (np. opóźnienia)
#include "driver/i2c.h"          // Sterownik I2C ESP-IDF
#include "esp_log.h"             // Logowanie na konsolę (ESP_LOGI, ESP_LOGE itp.)

/* ----------------------------------------------------------------
 *  KONFIGURACJA PINÓW I PARAMETRÓW I2C
 * ----------------------------------------------------------------
 *  Piny definiujemy jako stałe, żeby łatwo je zmienić w jednym
 *  miejscu zamiast szukać po całym kodzie.
 */
#define I2C_MASTER_SCL_IO       46        // Pin GPIO podłączony do SCL czujnika (zegar)
#define I2C_MASTER_SDA_IO       18        // Pin GPIO podłączony do SDA czujnika (dane)
#define I2C_MASTER_FREQ_HZ      400000    // Prędkość I2C = 400 kHz (tzw. "Fast Mode")
#define I2C_MASTER_NUM          I2C_NUM_0 // ESP32 ma 2 porty I2C; używamy nr 0
#define I2C_MASTER_TX_BUF       0         // Brak bufora TX (master nie potrzebuje)
#define I2C_MASTER_RX_BUF       0         // Brak bufora RX (master nie potrzebuje)

/* Adres I2C czujnika BME280.
 * Każde urządzenie I2C ma unikalny 7-bitowy adres (0x00–0x7F).
 * BME280 ma adres 0x76 lub 0x77 zależnie od stanu pinu SDO.
 * Tutaj mamy 0x77 (pin SDO podłączony do VCC). */
#define BME280_ADDR             0x77

static const char *TAG = "bme280"; // Etykieta wyświetlana w logach na konsoli

/* ----------------------------------------------------------------
 *  ADRESY REJESTRÓW CZUJNIKA BME280
 * ----------------------------------------------------------------
 *  Rejestry to "komórki pamięci" wewnątrz czujnika.
 *  Żeby coś odczytać / zapisać, podajemy adres rejestru.
 *  Wszystkie adresy są w dokumentacji (datasheet) BME280.
 */
#define BME280_REG_CHIP_ID      0xD0  // Rejestr ID układu – zawiera zawsze 0x60 (weryfikacja)
#define BME280_REG_CTRL_HUM     0xF2  // Rejestr konfiguracji pomiaru wilgotności
#define BME280_REG_CTRL_MEAS    0xF4  // Rejestr konfiguracji pomiaru temp. i ciśnienia + tryb pracy
#define BME280_REG_CONFIG       0xF5  // Rejestr ogólnej konfiguracji (czas stand-by, filtr IIR)
#define BME280_REG_PRESS_MSB    0xF7  // Pierwszy rejestr z surowymi danymi (ciśnienie, temp, wilg.)


/* ================================================================
 *  FUNKCJA: bme280_read
 * ================================================================
 *  Odczytuje `len` bajtów z rejestru `reg` czujnika BME280
 *  i zapisuje je do tablicy `data`.
 *
 *  Jak działa komunikacja I2C krok po kroku?
 *  ------------------------------------------
 *  1. START – sygnał "zaczynam nadawać" na magistrali
 *  2. Adres slave + bit WRITE – mówimy do kogo mówimy i że PISZEMY
 *  3. Adres rejestru – mówimy z jakiego miejsca chcemy czytać
 *  4. REPEATED START – ponowny start (bez zwalniania magistrali)
 *  5. Adres slave + bit READ – teraz chcemy CZYTAĆ
 *  6. Odczyt bajtów – czujnik odsyła dane
 *  7. NACK po ostatnim bajcie – mówimy "stop, to był ostatni bajt"
 *  8. STOP – koniec transmisji
 */
static esp_err_t bme280_read(uint8_t reg, uint8_t *data, size_t len)
{
    // Tworzymy "kolejkę poleceń I2C" – listę operacji do wykonania
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);   // 1. Sygnał START

    // 2. Wyślij adres czujnika z bitem WRITE (0)
    //    (BME280_ADDR << 1) przesuwa adres o 1 bit w lewo (robi miejsce na bit R/W)
    //    | I2C_MASTER_WRITE dodaje bit WRITE (0) na końcu
    //    `true` = czekaj na potwierdzenie (ACK) od slave'a
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);

    // 3. Wyślij adres rejestru który chcemy odczytać
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);   // 4. Ponowny START (Repeated Start)

    // 5. Wyślij adres czujnika z bitem READ (1)
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_READ, true);

    // 6. Odbierz `len` bajtów danych od czujnika
    //    I2C_MASTER_LAST_NACK = po ostatnim bajcie wyślij NACK (koniec odczytu)
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);

    i2c_master_stop(cmd);    // 8. Sygnał STOP

    // Wykonaj wszystkie przygotowane polecenia na magistrali I2C
    // pdMS_TO_TICKS(1000) = timeout 1000 ms
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));

    i2c_cmd_link_delete(cmd); // Zwolnij pamięć kolejki poleceń
    return ret;               // Zwróć czy operacja się udała (ESP_OK = sukces)
}

/* ================================================================
 *  FUNKCJA: bme280_write
 * ================================================================
 *  Zapisuje jeden bajt `value` do rejestru `reg` czujnika.
 *
 *  Używamy do konfiguracji czujnika (ustawienia trybów pracy).
 *  Sekwencja: START → adres+WRITE → adres rejestru → wartość → STOP
 */
static esp_err_t bme280_write(uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);   // Adres rejestru docelowego
    i2c_master_write_byte(cmd, value, true); // Wartość do zapisania
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* ================================================================
 *  STRUKTURA: bme280_calib_t – dane kalibracyjne
 * ================================================================
 *
 *  Po co kalibracja?
 *  -----------------
 *  Czujnik BME280 NIE zwraca bezpośrednio "24.5°C" czy "1013 hPa".
 *  Zwraca surowe liczby ADC (ang. Analog-to-Digital Converter) –
 *  czyli po prostu liczby całkowite, np. 519200 dla temperatury.
 *
 *  Żeby zamienić te liczby na prawdziwe jednostki (°C, hPa, %),
 *  potrzebujemy WSPÓŁCZYNNIKÓW KALIBRACYJNYCH. Są one unikalne
 *  dla każdego egzemplarza czujnika i zapisane w jego pamięci
 *  na etapie produkcji (w rejestrach 0x88–0x9F i 0xE1–0xE7).
 *
 *  dig_T1/T2/T3 = współczynniki korekcji temperatury
 *  dig_P1..P9   = współczynniki korekcji ciśnienia
 *  dig_H1..H6   = współczynniki korekcji wilgotności
 *
 *  uint16_t = liczba całkowita bez znaku 16-bitowa (0 do 65535)
 *  int16_t  = liczba całkowita ze znakiem 16-bitowa (-32768 do 32767)
 *  uint8_t  = liczba całkowita bez znaku 8-bitowa (0 do 255)
 *  int8_t   = liczba całkowita ze znakiem 8-bitowa (-128 do 127)
 */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1, dig_H3;
    int16_t  dig_H2, dig_H4, dig_H5;
    int8_t   dig_H6;
} bme280_calib_t;

static bme280_calib_t calib; // Globalna zmienna przechowująca dane kalibracyjne

/* t_fine = pośrednia wartość temperatury używana też przy obliczeniach
 * ciśnienia i wilgotności (kompensacja krzyżowa – temp. wpływa na inne pomiary) */
static int32_t t_fine;

/* ================================================================
 *  FUNKCJA: bme280_read_calib – wczytaj dane kalibracyjne
 * ================================================================
 *
 *  Odczytujemy z czujnika 26 + 7 bajtów danych kalibracyjnych.
 *
 *  Dlaczego składamy bajty parami?
 *  --------------------------------
 *  Każdy współczynnik zajmuje 2 bajty (16 bitów) w pamięci czujnika,
 *  ale I2C przesyła je bajt po bajcie. Trzeba je "skleić":
 *
 *  Przykład:  buf[0] = 0x70, buf[1] = 0x6B
 *  dig_T1 = (buf[1] << 8) | buf[0]
 *          = (0x6B << 8) | 0x70
 *          = 0x6B00 | 0x70
 *          = 0x6B70 = 27504  ← Little-Endian: młodszy bajt jest pierwszy!
 *
 *  "<< 8" przesuwa bity o 8 pozycji w lewo (= mnoży przez 256)
 *  "|"    to bitowe OR – skleja dwa bajty w jedną 16-bitową liczbę
 */
static void bme280_read_calib(void)
{
    uint8_t buf[26];
    bme280_read(0x88, buf, 26); // Czytaj 26 bajtów od adresu 0x88 (temp + ciśnienie)

    // Składamy 16-bitowe współczynniki z par bajtów (Little-Endian)
    calib.dig_T1 = (buf[1] << 8) | buf[0];   // Współczynnik T1 (unsigned)
    calib.dig_T2 = (buf[3] << 8) | buf[2];   // Współczynnik T2 (signed)
    calib.dig_T3 = (buf[5] << 8) | buf[4];   // Współczynnik T3 (signed)
    calib.dig_P1 = (buf[7] << 8) | buf[6];   // i tak dalej dla ciśnienia...
    calib.dig_P2 = (buf[9] << 8) | buf[8];
    calib.dig_P3 = (buf[11] << 8) | buf[10];
    calib.dig_P4 = (buf[13] << 8) | buf[12];
    calib.dig_P5 = (buf[15] << 8) | buf[14];
    calib.dig_P6 = (buf[17] << 8) | buf[16];
    calib.dig_P7 = (buf[19] << 8) | buf[18];
    calib.dig_P8 = (buf[21] << 8) | buf[20];
    calib.dig_P9 = (buf[23] << 8) | buf[22];
    calib.dig_H1 = buf[25];  // H1 jest 8-bitowy (1 bajt)

    uint8_t hbuf[7];
    bme280_read(0xE1, hbuf, 7); // Czytaj 7 bajtów danych kalibracyjnych wilgotności

    calib.dig_H2 = (hbuf[1] << 8) | hbuf[0]; // H2 – zwykłe 16-bitów

    calib.dig_H3 = hbuf[2]; // H3 – 8-bitowy

    // H4 i H5 są niestandardowo upakowane w 3 bajty (hbuf[3], hbuf[4], hbuf[5])
    // H4 = górne 8 bitów z hbuf[3] + dolne 4 bity z hbuf[4]
    calib.dig_H4 = (hbuf[3] << 4) | (hbuf[4] & 0x0F);  // 0x0F = maska na dolne 4 bity

    // H5 = górne 8 bitów z hbuf[5] + górne 4 bity z hbuf[4]
    calib.dig_H5 = (hbuf[5] << 4) | (hbuf[4] >> 4);    // >> 4 przesuwa górne 4 bity na dół

    calib.dig_H6 = hbuf[6]; // H6 – 8-bitowy ze znakiem (int8_t)
}

/* ================================================================
 *  FUNKCJA: bme280_compensate_temp – oblicz temperaturę
 * ================================================================
 *
 *  Wzór pochodzi bezpośrednio z datasheetu BME280 (rozdział 4.2.3).
 *  Jest to tzw. "compensation formula" – firma Bosch podaje gotowe
 *  wzory, bo przeliczenia są skomplikowane.
 *
 *  Skąd bierze się wynik w °C?
 *  ----------------------------
 *  adc_T  = surowa liczba z przetwornika ADC (np. 519200)
 *  dig_T1/T2/T3 = unikalne współczynniki kalibracyjne tego czujnika
 *
 *  Wzór liczy najpierw dwie zmienne pośrednie var1 i var2,
 *  a ich suma trafia do t_fine (globalna, używana też przez press i hum).
 *
 *  Końcowy wynik: ((t_fine * 5 + 128) >> 8) / 100.0
 *  To daje temperaturę w °C z dokładnością do 0.01°C.
 *
 *  >> 11, >> 12 itp. = przesunięcia bitowe w prawo (dzielenie przez potęgi 2)
 *  Używa się ich zamiast dzielenia bo są DUŻO szybsze na mikrokontrolerze.
 */
static float bme280_compensate_temp(int32_t adc_T)
{
    // var1: pierwsza korekta liniowa
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * calib.dig_T2) >> 11;

    // var2: korekta kwadratowa (nieliniowość czujnika)
    int32_t var2 = (((((adc_T >> 4) - (int32_t)calib.dig_T1) *
                      ((adc_T >> 4) - (int32_t)calib.dig_T1)) >> 12) * calib.dig_T3) >> 14;

    // t_fine = suma, używana też przez compensate_press i compensate_hum
    t_fine = var1 + var2;

    // Przelicz t_fine na °C i zwróć jako float
    return ((t_fine * 5 + 128) >> 8) / 100.0f;
}

/* ================================================================
 *  FUNKCJA: bme280_compensate_press – oblicz ciśnienie
 * ================================================================
 *
 *  Podobnie jak temp, ale używa int64_t (64-bitowe liczby całkowite)
 *  bo obliczenia ciśnienia dają bardzo duże liczby pośrednie.
 *
 *  Wynik jest w Pa (Pascalach) × 256 → dzielimy przez 25600
 *  żeby dostać hektopaskale (hPa), czyli "milibary".
 *
 *  Normalne ciśnienie atmosferyczne = ok. 1013 hPa.
 *
 *  var1 == 0: sprawdzamy żeby nie dzielić przez zero!
 */
static float bme280_compensate_press(int32_t adc_P)
{
    int64_t var1 = (int64_t)t_fine - 128000; // Korekta względem temperatury
    int64_t var2 = var1 * var1 * calib.dig_P6;
    var2 += (var1 * calib.dig_P5) << 17;
    var2 += (int64_t)calib.dig_P4 << 35;
    var1 = ((var1 * var1 * calib.dig_P3) >> 8) + ((var1 * calib.dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * calib.dig_P1 >> 33;
    if (var1 == 0) return 0; // Zabezpieczenie przed dzieleniem przez zero

    int64_t p = 1048576 - adc_P;               // Surowe dane ADC
    p = (((p << 31) - var2) * 3125) / var1;     // Przeliczenie na Pa × 256
    var1 = (calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25; // Korekty wyższego rzędu
    var2 = (calib.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)calib.dig_P7 << 4); // Finalna wartość Pa × 256

    return p / 25600.0f; // Przelicz Pa × 256 → hPa (dziel przez 256 × 100)
}

/* ================================================================
 *  FUNKCJA: bme280_compensate_hum – oblicz wilgotność
 * ================================================================
 *
 *  Wynik to wilgotność względna w procentach (0–100%).
 *  Znowu wzór z datasheetu – dość zawiły, ale działa :)
 *
 *  Clampowanie (ograniczanie zakresu):
 *    if (v < 0) v = 0            → nie może być ujemna wilgotność
 *    if (v > 419430400) v = ...  → nie może przekroczyć 100%
 *
 *  Końcowe (v >> 12) / 1024.0f przelicza wewnętrzną reprezentację
 *  stałoprzecinkową (Q22.10) na zwykłą liczbę zmiennoprzecinkową %.
 */
static float bme280_compensate_hum(int32_t adc_H)
{
    int32_t v = t_fine - 76800; // Odejmij offset temperaturowy

    // Długi wzór kalibracyjny z datasheetu (nie trzeba go rozumieć szczegółowo)
    v = (((adc_H << 14) - (calib.dig_H4 << 20) - (calib.dig_H5 * v)) + 16384) >> 15;
    v = v * (((((((v * calib.dig_H6) >> 10) *
                 (((v * calib.dig_H3) >> 11) + 32768)) >> 10) + 2097152) *
               calib.dig_H2 + 8192) >> 14);

    // Korekta tłumienia (nielinowości czujnika)
    v -= ((((v >> 15) * (v >> 15)) >> 7) * calib.dig_H1) >> 4;

    if (v < 0) v = 0;               // Clamp: minimum 0%
    if (v > 419430400) v = 419430400; // Clamp: maksimum 100%

    return (v >> 12) / 1024.0f; // Przelicz na procenty (%)
}


/* ================================================================
 *  FUNKCJA GŁÓWNA: app_main
 * ================================================================
 *
 *  app_main() to odpowiednik main() w zwykłym C.
 *  W ESP-IDF to punkt wejścia aplikacji.
 */
void app_main(void)
{
    /* --------------------------------------------------------
     *  KROK 1: Inicjalizacja magistrali I2C
     * --------------------------------------------------------
     *  Wypełniamy strukturę konfiguracyjną i przekazujemy ją
     *  do sterownika I2C ESP-IDF.
     */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,               // ESP32 jest masterem (inicjuje komunikację)
        .sda_io_num = I2C_MASTER_SDA_IO,       // Który pin GPIO to SDA (dane)
        .scl_io_num = I2C_MASTER_SCL_IO,       // Który pin GPIO to SCL (zegar)
        .sda_pullup_en = GPIO_PULLUP_ENABLE,   // Włącz wewnętrzny rezystor pull-up na SDA
        .scl_pullup_en = GPIO_PULLUP_ENABLE,   // Włącz wewnętrzny rezystor pull-up na SCL
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // Prędkość zegara I2C (400 kHz)
    };

    // Zaaplikuj konfigurację do portu I2C nr 0
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));

    // Zainstaluj sterownik I2C (inicjuje sprzęt)
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER,
                                       I2C_MASTER_TX_BUF, I2C_MASTER_RX_BUF, 0));

    /* --------------------------------------------------------
     *  KROK 2: Weryfikacja Chip ID
     * --------------------------------------------------------
     *  Czytamy rejestr 0xD0 – czujnik zawsze zwraca tam 0x60.
     *  To prosty test "czy w ogóle coś jest podłączone i działa".
     */
    uint8_t chip_id;
    bme280_read(BME280_REG_CHIP_ID, &chip_id, 1); // Odczytaj 1 bajt z rejestru ID
    ESP_LOGI(TAG, "Chip ID: 0x%02X (oczekiwano 0x60)", chip_id);
    // Jeśli wypisze 0x60 – wszystko OK. Inaczej – problem z połączeniem.

    /* --------------------------------------------------------
     *  KROK 3: Wczytaj dane kalibracyjne
     * --------------------------------------------------------
     *  Bez kalibracji odczyty będą bezsensownymi liczbami.
     *  Robimy to raz na początku (dane nie zmieniają się).
     */
    bme280_read_calib();

    /* --------------------------------------------------------
     *  KROK 4: Konfiguracja czujnika
     * --------------------------------------------------------
     *  Piszemy do rejestrów konfiguracyjnych żeby ustawić:
     *
     *  CTRL_HUM = 0x01 → oversampling wilgotności × 1
     *    (osample = ile razy zmierzyć i uśrednić = lepsza dokładność)
     *    Bity: 00000001 = osrs_h = 1 (1× oversampling)
     *    WAŻNE: Zmiana CTRL_HUM wchodzi w życie dopiero po zapisie CTRL_MEAS!
     *
     *  CTRL_MEAS = 0x27 → binarnie: 00100111
     *    Bity 7-5 (osrs_t): 001 = temperatura × 1 oversampling
     *    Bity 4-2 (osrs_p): 001 = ciśnienie × 1 oversampling
     *    Bity 1-0 (mode):   11  = tryb Normal (ciągłe pomiary)
     *
     *  CONFIG = 0xA0 → binarnie: 10100000
     *    Bity 7-5 (t_sb):   101 = czas stand-by 1000 ms
     *    Bity 4-2 (filter): 000 = filtr IIR wyłączony
     *    Bit  0   (spi3w):  0   = SPI 4-wire (nieistotne, używamy I2C)
     */
    bme280_write(BME280_REG_CTRL_HUM,  0x01); // Wilgotność: 1× oversampling
    bme280_write(BME280_REG_CTRL_MEAS, 0x27); // Temp 1×, Ciśnienie 1×, tryb Normal
    bme280_write(BME280_REG_CONFIG,    0xA0); // Stand-by 1000ms, filtr off

    /* --------------------------------------------------------
     *  KROK 5: Główna pętla pomiarowa
     * --------------------------------------------------------
     *  while(1) = nieskończona pętla – program nigdy nie kończy pracy.
     *  Co 2 sekundy odczytujemy dane i wypisujemy na konsolę.
     */
    while (1) {
        /* Odczytaj 8 surowych bajtów danych z czujnika.
         * Zaczyna się od rejestru 0xF7 (PRESS_MSB) i auto-inkrementuje:
         *   buf[0..2] = surowe ciśnienie (20 bitów, upper 12 + lower 8)
         *   buf[3..5] = surowa temperatura (20 bitów)
         *   buf[6..7] = surowa wilgotność (16 bitów)
         */
        uint8_t buf[8];
        bme280_read(BME280_REG_PRESS_MSB, buf, 8);

        /* Złóż surowe wartości ADC z odebranych bajtów.
         *
         * Ciśnienie zajmuje 20 bitów w 3 bajtach (buf[0], buf[1], buf[2]):
         *   buf[0] = bity 19-12 (MSB – najbardziej znaczące bity)
         *   buf[1] = bity 11-4
         *   buf[2] = bity 3-0 (tylko górne 4 bity, reszta to padding)
         *
         *   << 12 przesuwa buf[0] na pozycje 19-12
         *   << 4  przesuwa buf[1] na pozycje 11-4
         *   >> 4  przesuwa buf[2] o 4 w prawo (wyrzuca dolne 4 bity padding)
         */
        int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);

        // Temperatura: to samo co ciśnienie ale z buf[3..5]
        int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);

        // Wilgotność: 16 bitów w 2 bajtach (buf[6], buf[7]) – prostsze
        int32_t adc_H = ((int32_t)buf[6] << 8) | buf[7];

        /* Przelicz surowe wartości ADC na prawdziwe jednostki
         * używając współczynników kalibracyjnych i wzorów z datasheetu */
        float temp  = bme280_compensate_temp(adc_T);   // Temperatura w °C
        float press = bme280_compensate_press(adc_P);  // Ciśnienie w hPa
        float hum   = bme280_compensate_hum(adc_H);    // Wilgotność w %

        // Wypisz wyniki na konsolę (widoczne przez monitor szeregowy / idf.py monitor)
        // %.2f = float z 2 miejscami po przecinku
        ESP_LOGI(TAG, "Temperatura: %.2f C", temp);
        ESP_LOGI(TAG, "Cisnienie:   %.2f hPa", press);
        ESP_LOGI(TAG, "Wilgotnosc:  %.2f %%", hum);  // "%%" = wypisz dosłownie znak "%"

        // Czekaj 2000 ms (2 sekundy) przed następnym pomiarem
        // pdMS_TO_TICKS przelicza milisekundy na "ticki" FreeRTOS
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
