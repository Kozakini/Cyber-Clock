#include "bme280_mqtt.h"
#include "cJSON.h"
#include "esp_log.h"

static const char *TAG = "MQTT";
static esp_mqtt_client_handle_t mqtt_client = NULL;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Połączony z brokerem ✓");
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT Rozłączony");
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "Błąd MQTT. Typ: %d", event->error_handle->error_type);
            break;

        default:
            break;
    }
}

esp_err_t mqtt_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://10.203.118.207",
        .broker.address.port = 1883,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    return ESP_OK;
}

void mqtt_publish_data(float temp, float press, float hum)
{
    if (mqtt_client == NULL) return;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "temperature", temp);
    cJSON_AddNumberToObject(root, "humidity",    hum);
    cJSON_AddNumberToObject(root, "pressure",    press);
    cJSON_AddStringToObject(root, "device",      "esp32_s3_01");

    char *json_str = cJSON_PrintUnformatted(root);

    esp_mqtt_client_publish(mqtt_client, "sensors/bme280", json_str, 0, 1, 0);

    cJSON_free(json_str);
    cJSON_Delete(root);
}
