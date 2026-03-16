/*
 * weather_test — pobieranie pogody z OpenWeatherMap i wyswietlanie na serial monitorze
 *
 * Przeplyw programu:
 *   1. app_main()     — punkt startowy, uruchamia WiFi i petle glowna
 *   2. wifi_connect() — laczy ESP z siecia WiFi
 *   3. fetch_weather()— pobiera JSON z API, parsuje, drukuje wyniki
 */

// --- Standardowe biblioteki C ---
#include <stdio.h>      // printf
#include <string.h>     // memcpy, memset, strncpy

// --- FreeRTOS — system operacyjny czasu rzeczywistego dzialajacy na ESP ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"          // vTaskDelay — czekanie bez blokowania procesora
#include "freertos/event_groups.h"  // EventGroup — mechanizm sygnalizacji miedzy zadaniami

// --- Biblioteki ESP-IDF ---
#include "esp_wifi.h"         // obsluga WiFi
#include "esp_event.h"        // system zdarzen (WiFi connected, got IP, itp.)
#include "esp_log.h"          // ESP_LOGE — logowanie bledow z prefixem i czasem
#include "esp_http_client.h"  // klient HTTP do pobierania danych z API
#include "nvs_flash.h"        // NVS = Non-Volatile Storage, wymagane przez WiFi
#include "cJSON.h"            // parser JSON wbudowany w ESP-IDF

// =============================================================
//  KONFIGURACJA — zmien na swoje dane
// =============================================================
#define WIFI_SSID      "TwojaSiec"
#define WIFI_PASSWORD  "TwojeHaslo"
#define OWM_API_KEY    "TwojKluczAPI"  // darmowy klucz z openweathermap.org/api
#define OWM_CITY       "Warsaw"
#define OWM_COUNTRY    "PL"
// =============================================================

// TAG pojawia sie jako prefix w logach: "E (123) WEATHER: ..."
static const char *TAG = "WEATHER";

/*
 * EventGroup — specjalny obiekt FreeRTOS dzialajacy jak zestaw flag bitowych.
 * Uzywamy go zeby wifi_connect() moglo CZEKAC az WiFi sie polaczy,
 * zamiast odpytywac w petli ("czy juz jest IP? czy juz jest IP?").
 * Gdy handler dostanie zdarzenie "got IP" — ustawia flage WIFI_OK,
 * a wifi_connect() sie odblokuje.
 */
static EventGroupHandle_t wifi_eg;
#define WIFI_OK BIT0   // bit nr 0 = "WiFi polaczone i ma adres IP"


// =============================================================
//  WIFI
// =============================================================

/*
 * wifi_handler — wywoływana automatycznie przez ESP przy zdarzeniach WiFi.
 *
 * ESP-IDF uzywa systemu zdarzen: zamiast sprawdzac status w petli,
 * rejestrujesz funkcje-handler i ESP wola ja kiedy cos sie dzieje.
 *
 * Parametry:
 *   base — kategoria zdarzenia (WIFI_EVENT lub IP_EVENT)
 *   id   — konkretne zdarzenie w tej kategorii
 */
static void wifi_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        // ESP wystartował w trybie stacji (klient) — teraz probuj sie polaczyc
        esp_wifi_connect();

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        // Rozlaczono (np. zly zasieg, restart routera) — probuj ponownie
        esp_wifi_connect();

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        // ESP dostal adres IP od routera — polaczenie udane!
        // Ustawiamy flage WIFI_OK w EventGroup zeby odblokowac wifi_connect()
        xEventGroupSetBits(wifi_eg, WIFI_OK);
    }
}

/*
 * wifi_connect — inicjalizuje WiFi i czeka (maks. 10s) na polaczenie.
 * Zwraca true jesli polaczono, false jesli timeout.
 */
static bool wifi_connect(void) {

    // Utworz EventGroup — pusty zestaw flag (wszystkie bity = 0)
    wifi_eg = xEventGroupCreate();

    // Inicjalizacja stosu sieciowego ESP (wymagane raz na starcie)
    esp_netif_init();

    // Uruchom wewnetrzna petle zdarzen ESP (przetwarza eventy WiFi w tle)
    esp_event_loop_create_default();

    // Stworz domyslny interfejs sieciowy dla trybu stacji (STA = klient WiFi)
    esp_netif_create_default_wifi_sta();

    // Inicjalizacja sterownika WiFi z domyslna konfiguracja
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Zarejestruj handler dla WSZYSTKICH zdarzen WiFi (ESP_EVENT_ANY_ID)
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler, NULL);
    // Osobno dla zdarzenia "dostalm IP" — to jest w kategorii IP_EVENT, nie WIFI_EVENT
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_handler, NULL);

    // Wypelnij konfiguracje sieci: nazwa i haslo
    // wcfg = {} zeruje cala strukture (wazne — bez tego moga byc smieci w pamieci)
    wifi_config_t wcfg = {};
    strncpy((char *)wcfg.sta.ssid,     WIFI_SSID,     sizeof(wcfg.sta.ssid));
    strncpy((char *)wcfg.sta.password, WIFI_PASSWORD, sizeof(wcfg.sta.password));

    esp_wifi_set_mode(WIFI_MODE_STA);      // tryb STA = klient (nie access point)
    esp_wifi_set_config(WIFI_IF_STA, &wcfg); // wgraj konfiguracje sieci
    esp_wifi_start(); // uruchom WiFi — wywoła WIFI_EVENT_STA_START, handler zrobi connect()

    /*
     * Czekaj na flage WIFI_OK maks. 10 sekund.
     * xEventGroupWaitBits blokuje to zadanie (nie blokuje procesora — inne zadania dzialaja)
     * dopoki flaga WIFI_OK nie zostanie ustawiona przez handler LUB nie minie timeout.
     *
     * Parametry:
     *   wifi_eg              — nasz EventGroup
     *   WIFI_OK              — czekamy na ten bit
     *   pdFALSE              — nie kasuj bitu po odczytaniu
     *   pdFALSE              — wystarczy jeden bit (nie wszystkie — tu mamy tylko jeden)
     *   pdMS_TO_TICKS(10000) — timeout 10 sekund przeliczony na ticki FreeRTOS
     */
    EventBits_t bits = xEventGroupWaitBits(wifi_eg, WIFI_OK, pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(10000));

    // Jesli bit WIFI_OK jest ustawiony — polaczono. Jesli nie — timeout.
    return (bits & WIFI_OK) != 0;
}


// =============================================================
//  HTTP — odbieranie odpowiedzi z serwera
// =============================================================

// Bufor na odpowiedz HTTP — 4096B wystarczy, OWM zwraca ok. 500B JSON
#define HTTP_BUF 4096
static char http_buf[HTTP_BUF];
static int  http_len = 0;  // ile bajtow juz odebralismy

/*
 * http_handler — wywoływany przez esp_http_client przy kazdym zdarzeniu HTTP.
 *
 * Odpowiedz HTTP moze przyjsc w kawalkach (chunked transfer encoding),
 * dlatego musimy zbierac dane w buforze zamiast czytac jednorazowo.
 * Ten handler jest wolany wielokrotnie z kolejnymi kawałkami danych.
 */
static esp_err_t http_handler(esp_http_client_event_t *evt) {
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        // Dostalismy kolejny kawalek — dopisz do bufora
        // Sprawdzamy czy bufor sie nie przepelni
        if (http_len + evt->data_len < HTTP_BUF - 1) {
            memcpy(http_buf + http_len, evt->data, evt->data_len);
            http_len += evt->data_len;
        }
    }
    return ESP_OK;
}


// =============================================================
//  POBIERANIE I PARSOWANIE POGODY
// =============================================================

/*
 * fetch_weather — wysyla zapytanie do OWM API, parsuje JSON, drukuje wyniki.
 *
 * OWM zwraca JSON (uproszczony):
 * {
 *   "name": "Warsaw",
 *   "weather": [{"id": 803, "description": "zachmurzenie duze"}],
 *   "main": {"temp": 12.3, "feels_like": 10.1, "humidity": 65},
 *   "wind": {"speed": 3.5}
 * }
 *
 * Kody ID pogody (weather.id):
 *   2xx = burza z piorunami
 *   3xx = mzawka
 *   5xx = deszcz
 *   6xx = snieg
 *   7xx = mgla, dym, pyl
 *   800 = bezchmurnie
 *   801-804 = rozne stopnie zachmurzenia
 */
static void fetch_weather(void) {

    // Zbuduj URL: wstaw miasto, kraj i klucz API
    // units=metric = temperatury w °C
    // lang=pl      = opisy pogody po polsku
    char url[256];
    snprintf(url, sizeof(url),
        "http://api.openweathermap.org/data/2.5/weather"
        "?q=%s,%s&appid=%s&units=metric&lang=pl",
        OWM_CITY, OWM_COUNTRY, OWM_API_KEY);

    // Wyzeruj bufor przed nowym zapytaniem (dane z poprzedniego pobierania)
    http_len = 0;
    memset(http_buf, 0, sizeof(http_buf));

    // Konfiguracja klienta HTTP
    esp_http_client_config_t cfg = {
        .url           = url,
        .event_handler = http_handler, // nasza funkcja do zbierania danych
        .timeout_ms    = 5000,         // jesli serwer nie odpowie w 5s — blad
    };

    // Wyslij zapytanie GET i poczekaj na odpowiedz (blokujace)
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_err_t err = esp_http_client_perform(client);
    int status    = esp_http_client_get_status_code(client); // np. 200, 401, 404
    esp_http_client_cleanup(client); // zawsze zwolnij pamiec klienta!

    // Sprawdz czy zapytanie sie powiodlo:
    //   err != ESP_OK = blad sieciowy (brak internetu, timeout)
    //   status != 200 = serwer odpowiedzial bledem (401=zly klucz, 404=zle miasto)
    if (err != ESP_OK || status != 200) {
        ESP_LOGE(TAG, "HTTP blad: %s, status=%d", esp_err_to_name(err), status);
        return;
    }

    // Po perform() nasz http_buf zawiera pelna odpowiedz JSON — parsujemy
    // cJSON_Parse tworzy drzewo obiektow w dynamicznie alokowanej pamieci
    cJSON *root = cJSON_Parse(http_buf);
    if (!root) {
        ESP_LOGE(TAG, "JSON parse error");
        return;
    }

    // --- Wyciaganie danych z drzewa JSON ---
    // Kazde pole wyciagamy przez cJSON_GetObjectItem("nazwa_pola")
    // Sprawdzamy typ (cJSON_IsString, cJSON_IsNumber) zanim uzymy wartosci
    // — OWM moze nie zwrocic jakiegos pola jesli dane sa niedostepne

    float  temp       = 0;
    float  feels      = 0;
    float  wind       = 0;
    int    humidity   = 0;
    int    weather_id = 0;
    char   desc[64]   = "brak";
    char   city[64]   = "brak";

    // "name": "Warsaw"
    cJSON *jname = cJSON_GetObjectItem(root, "name");
    if (cJSON_IsString(jname))
        strncpy(city, jname->valuestring, sizeof(city) - 1);

    // "weather": [{"id": 803, "description": "..."}]  — to jest TABLICA, bierzemy element [0]
    cJSON *weather_arr = cJSON_GetObjectItem(root, "weather");
    if (cJSON_IsArray(weather_arr)) {
        cJSON *w0    = cJSON_GetArrayItem(weather_arr, 0);
        cJSON *jid   = cJSON_GetObjectItem(w0, "id");
        cJSON *jdesc = cJSON_GetObjectItem(w0, "description");
        if (cJSON_IsNumber(jid))   weather_id = jid->valueint;
        if (cJSON_IsString(jdesc)) strncpy(desc, jdesc->valuestring, sizeof(desc) - 1);
    }

    // "main": {"temp": 12.3, "feels_like": 10.1, "humidity": 65}
    cJSON *jmain = cJSON_GetObjectItem(root, "main");
    if (jmain) {
        cJSON *jt = cJSON_GetObjectItem(jmain, "temp");
        cJSON *jf = cJSON_GetObjectItem(jmain, "feels_like");
        cJSON *jh = cJSON_GetObjectItem(jmain, "humidity");
        if (cJSON_IsNumber(jt)) temp     = (float)jt->valuedouble;
        if (cJSON_IsNumber(jf)) feels    = (float)jf->valuedouble;
        if (cJSON_IsNumber(jh)) humidity = jh->valueint;
    }

    // "wind": {"speed": 3.5}
    cJSON *jwind = cJSON_GetObjectItem(root, "wind");
    if (jwind) {
        cJSON *jw = cJSON_GetObjectItem(jwind, "speed");
        if (cJSON_IsNumber(jw)) wind = (float)jw->valuedouble;
    }

    // WAZNE: zwolnij pamiec zajeta przez drzewo JSON!
    // cJSON alokuje dynamicznie — bez Delete() wycieknie pamiec po kazdym pobraniu
    cJSON_Delete(root);

    // --- Mapowanie ID na czytelny stan i nazwe pliku BMP ---
    const char *stan;
    if      (weather_id >= 200 && weather_id < 300) stan = "BURZA";
    else if (weather_id >= 300 && weather_id < 600) stan = "DESZCZ";
    else if (weather_id >= 600 && weather_id < 700) stan = "SNIEG";
    else if (weather_id >= 700 && weather_id < 800) stan = "MGLA/INNE";
    else if (weather_id == 800)                     stan = "SLONECZNIE";
    else                                            stan = "ZACHMURZENIE";

    // Nazwa pliku BMP — ta sama logika, uzyjemy tego pozniej do wyswietlenia awatara na ekranie
    const char *bmp;
    if      (weather_id >= 200 && weather_id < 300) bmp = "storm";
    else if (weather_id >= 300 && weather_id < 600) bmp = "rainy";
    else if (weather_id >= 600 && weather_id < 700) bmp = "snowy";
    else if (weather_id == 800)                     bmp = "sunny";
    else if (weather_id >  800)                     bmp = "cloudy";
    else                                            bmp = "unknown";

    // --- Wydruk na serial monitorze ---
    printf("\n");
    printf("==========================================\n");
    printf("  Pogoda dla: %s\n", city);
    printf("==========================================\n");
    printf("  Stan:        %s (%s)\n", stan, desc);
    printf("  Temperatura: %.1f C\n",   temp);
    printf("  Odczuwalna:  %.1f C\n",   feels);
    printf("  Wilgotnosc:  %d%%\n",     humidity);
    printf("  Wiatr:       %.1f m/s\n", wind);
    printf("  OWM ID:      %d\n",       weather_id);
    printf("  BMP plik:    /spiffs/%s.bmp\n", bmp);
    printf("==========================================\n\n");
}


// =============================================================
//  PUNKT STARTOWY
// =============================================================

/*
 * app_main — ESP-IDF wola te funkcje zaraz po starcie systemu.
 * Odpowiednik main() w zwyklym programie C.
 */
void app_main(void) {

    /*
     * NVS (Non-Volatile Storage) = mala pamiec flash do przechowywania ustawien.
     * WiFi uzywa jej do zapisywania konfiguracji miedzy restartami.
     * Musimy ja zainicjowac zanim uzyijemy WiFi.
     * Jesli NVS jest uszkodzone lub niekompatybilne — wymazujemy i inicjujemy od nowa.
     */
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    printf("\n=== Weather API test ===\n");

    // Polacz z WiFi — blokuje maks. 10 sekund
    printf("Laczenie z WiFi: %s ...\n", WIFI_SSID);
    if (!wifi_connect()) {
        printf("BLAD: nie mozna polaczyc z WiFi!\n");
        printf("Sprawdz SSID i haslo w kodzie.\n");
        return; // wyjdz z app_main — ESP wejdzie w restart loop
    }
    printf("WiFi OK!\n\n");

    // Petla glowna — pobieraj pogode co 30 sekund
    // vTaskDelay nie blokuje procesora — inne zadania FreeRTOS dzialaja w tym czasie
    // pdMS_TO_TICKS przelicza milisekundy na ticki FreeRTOS (domyslnie 1 tick = 1ms)
    while (1) {
        fetch_weather();
        printf("Nastepne pobieranie za 30 sekund...\n");
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
