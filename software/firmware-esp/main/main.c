#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_wifi.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <cJSON.h>
#include <cmp/cmp.h>

#include "protocol/protocol.h"

#define WIFI_SSID       "Hogger^2"
#define WIFI_PASSWORD   "12345678"
#define BUFFER_SIZE     (2*1024)
#define GPIO_LED        10

static httpd_handle_t server = NULL;
static QueueHandle_t uart_queue;

static protocol_t protocol = PROTOCOL_INIT;
static uint8_t protocol_buffer_rx[BUFFER_SIZE];
static uint8_t protocol_buffer_tx[BUFFER_SIZE];
static uint8_t protocol_buffer_decode[BUFFER_SIZE];
static SemaphoreHandle_t protocol_lock;

static SemaphoreHandle_t state_lock;
static uint8_t state[BUFFER_SIZE] = {0};
static size_t state_size = 0;

static char json[BUFFER_SIZE] = {0};
static uint8_t msgpack[BUFFER_SIZE] = {0};

static void uart_init() {
    const uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .stop_bits = UART_STOP_BITS_1,
        .parity = UART_PARITY_DISABLE,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_NUM_0, &config);
    uart_set_pin(UART_NUM_0, 21, 20, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, BUFFER_SIZE, BUFFER_SIZE, BUFFER_SIZE, &uart_queue, 0);

    ESP_LOGI("app", "uart started");
}

static void protocol_cb_receive(const uint8_t id, const void *payload, const uint32_t size) {
    (void)id;

    xSemaphoreTake(state_lock, portMAX_DELAY);
    memcpy(state, payload, size);
    state_size = size;
    xSemaphoreGive(state_lock);
}

static void protocol_init() {
    protocol_lock = xSemaphoreCreateBinary();

	protocol.callback_rx = protocol_cb_receive;
    protocol.fifo_rx.buffer = protocol_buffer_rx;
    protocol.fifo_rx.size = sizeof(protocol_buffer_rx);
    protocol.fifo_tx.buffer = protocol_buffer_tx;
    protocol.fifo_tx.size = sizeof(protocol_buffer_tx);
    protocol.decoded = protocol_buffer_decode;
    protocol.max = sizeof(protocol_buffer_decode);

    xSemaphoreGive(protocol_lock);

    ESP_LOGI("app", "protocol started");
}

static void uart_task(void *params) {
    (void)params;

    char data[BUFFER_SIZE];

    ESP_LOGI("app", "uart task started");

    while(1) {
        const int rx = uart_read_bytes(UART_NUM_0, data, sizeof(data), 1);
        for(int i=0; i<rx; i++) {
            fifo_write(&protocol.fifo_rx, data[i]);
        }

        xSemaphoreTake(protocol_lock, portMAX_DELAY);
        protocol_process(&protocol);

        const int tx = fifo_pending(&protocol.fifo_tx);
        for(int i=0; i<tx; i++) {
            data[i] = fifo_read(&protocol.fifo_tx);
        }
        xSemaphoreGive(protocol_lock);

        uart_write_bytes(UART_NUM_0, data, tx);

        vTaskDelay(1);
    }
}

static void wifi_init() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASSWORD,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &config);
    esp_wifi_start();

    ESP_LOGI("app", "wifi started ssid: \"%s\" password: \"%s\"", WIFI_SSID, WIFI_PASSWORD);
}

typedef struct {
    uint8_t *buffer;
    size_t capacity;
    size_t size;
    size_t position;
} buffer_t;

bool buffer_reader(cmp_ctx_t *ctx, void *data, size_t count) {
    buffer_t *buf = (buffer_t *)ctx->buf;

    if((buf->position+count)>buf->capacity) {
        return false;
    }

    memcpy(data, buf->buffer+buf->position, count);
    buf->position +=count;

    return true;
}

bool buffer_skipper(cmp_ctx_t *ctx, size_t count) {
    buffer_t *buf = (buffer_t *)ctx->buf;

    if((buf->position+count)>buf->capacity) {
        return false;
    }

    buf->position +=count;

    return true;
}

size_t buffer_writer(cmp_ctx_t *ctx, const void *data, size_t count) {
    buffer_t *buf = (buffer_t *)ctx->buf;

    //ESP_LOGI("kurwa", "(write) pos %d count %d", buf->position, count);

    if((buf->position+count)>buf->capacity) {
        return 0;
    }

    memcpy(buf->buffer+buf->position, data, count);
    buf->position +=count;
    buf->size = buf->position;

    return count;
}

void to_msgpack(cmp_ctx_t *cmp, cJSON *json) {
    if(cJSON_IsObject(json)) {
        const int size = cJSON_GetArraySize(json);
        cmp_write_map(cmp, size);

        cJSON *child = NULL;
        cJSON_ArrayForEach(child, json) {
            cmp_write_str(cmp, child->string, strlen(child->string));
            to_msgpack(cmp, child);
        }

        return;
    }

    if(cJSON_IsArray(json)) {
        const int size = cJSON_GetArraySize(json);
        cmp_write_array(cmp, size);

        cJSON *element = NULL;
        cJSON_ArrayForEach(element, json) {
            to_msgpack(cmp, element);
        }

        return;
    }

    if(cJSON_IsString(json)) {
        const char *str = cJSON_GetStringValue(json);
        cmp_write_str(cmp, str, strlen(str));
        return;
    }

    if(cJSON_IsNumber(json)) {
        cmp_write_float(cmp, json->valuedouble);
        return;
    }

    if(cJSON_IsBool(json)) {
        cmp_write_bool(cmp, cJSON_IsTrue(json));
        return;
    }

    if(cJSON_IsNull(json)) {
        cmp_write_nil(cmp);
        return;
    }
}

cJSON *to_json(cmp_ctx_t *cmp) {
    cmp_object_t object;
    if(!cmp_read_object(cmp, &object)) {
        ESP_LOGE("MSGPACK to JSON", "parse error");
        return NULL;
    }

    uint32_t array_size = 0;
    if(cmp_object_as_array(&object, &array_size)) {
        cJSON *array = cJSON_CreateArray();

        for(uint32_t i=0; i<array_size; i++) {
            cJSON_AddItemToArray(array, to_json(cmp));
        }

        return array;
    }

    uint32_t map_size = 0;
    if(cmp_object_as_map(&object, &map_size)) {
        cJSON *object = cJSON_CreateObject();

        for(uint32_t i=0; i<map_size; i++) {
            char key[256];
            uint32_t key_len = sizeof(key);
            if(!cmp_read_str(cmp, key, &key_len)) {
                ESP_LOGE("MSGPACK to JSON", "parse error (in map %lu) error %d", map_size, cmp->error);
                return NULL;
            }

            cJSON_AddItemToObject(object, key, to_json(cmp));
        }

        return object;
    }

    char str[256];
    uint32_t str_len = 0;
    if(cmp_object_as_str(&object, &str_len)) {
        if(!cmp_object_to_str(cmp, &object, str, sizeof(str))) {
            ESP_LOGE("MSGPACK to JSON", "parse error (in str) error %d", cmp->error);
            return NULL;
        }
        return cJSON_CreateString(str);
    }

    int64_t integer = 0;
    if(cmp_object_as_long(&object, &integer)) {
        return cJSON_CreateNumber(integer);
    }

    float floating = 0;
    if(cmp_object_as_float(&object, &floating)) {
        return cJSON_CreateNumber(floating);
    }

    bool boolean = false;
    if(cmp_object_as_bool(&object, &boolean)) {
        return cJSON_CreateBool(boolean);
    }

    return cJSON_CreateNull();
}

static esp_err_t http_handler_get(httpd_req_t *req) {
    xSemaphoreTake(state_lock, portMAX_DELAY);
    buffer_t buffer = {
        .buffer = state,
        .capacity = state_size,
        .size = state_size,
        .position = 0,
    };
    cmp_ctx_t cmp;
    cmp_init(&cmp, &buffer, buffer_reader, NULL, NULL);
    cJSON *json_obj = to_json(&cmp);
    xSemaphoreGive(state_lock);

    if(json_obj) {
        char *str = cJSON_Print(json_obj);
        httpd_resp_send(req, str, HTTPD_RESP_USE_STRLEN);
        free(str);
        cJSON_Delete(json_obj);
    } else {
        ESP_LOGE("app", "JSON parser failed");
        httpd_resp_send(req, NULL, 0);
    }

    return ESP_OK;
}

static esp_err_t http_handler_post(httpd_req_t *req) {
    httpd_req_recv(req, json, sizeof(json));
    httpd_resp_send(req, NULL, 0);

    cJSON *json_obj = cJSON_Parse(json);
    if(!json_obj) {
        ESP_LOGE("app", "Error parsing incoming JSON\n");
        return ESP_OK;
    }

    buffer_t buffer = {
        .buffer = msgpack,
        .capacity = sizeof(msgpack),
        .size = 0,
        .position = 0,
    };
    cmp_ctx_t cmp;
    cmp_init(&cmp, &buffer, NULL, NULL, buffer_writer);
    to_msgpack(&cmp, json_obj);

    cJSON_Delete(json_obj);

    xSemaphoreTake(protocol_lock, portMAX_DELAY);
    protocol_enqueue(&protocol, 0, buffer.buffer, buffer.size);
    xSemaphoreGive(protocol_lock);

    return ESP_OK;
}

static void http_init() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    if(httpd_start(&server, &config)!=ESP_OK) {
        return;
    }

    const httpd_uri_t uri_get = {
        .uri      = "/get",
        .method   = HTTP_GET,
        .handler  = http_handler_get,
        .user_ctx = NULL
    };

    const httpd_uri_t uri_post = {
        .uri      = "/post",
        .method   = HTTP_POST,
        .handler  = http_handler_post,
        .user_ctx = NULL
    };

    httpd_register_uri_handler(server, &uri_get);
    httpd_register_uri_handler(server, &uri_post);

    ESP_LOGI("app", "http server started");
}

static void blink_task(void *params) {
    (void)params;

    gpio_reset_pin(GPIO_LED);
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LED, 0);

    uint8_t led = false;

    ESP_LOGI("app", "blink task started");

    while(1) {
        led = !led;

        xSemaphoreTake(protocol_lock, portMAX_DELAY);
        protocol_enqueue(&protocol, 1, &led, sizeof(led));
        xSemaphoreGive(protocol_lock);

        gpio_set_level(GPIO_LED, led);
        vTaskDelay(pdMS_TO_TICKS(led ? 100 : 900));
    }
}

void app_main() {
    const esp_err_t result = nvs_flash_init();
    if((result==ESP_ERR_NVS_NO_FREE_PAGES) || (result==ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    uart_init();
    protocol_init();
    wifi_init();
    http_init();

    state_lock = xSemaphoreCreateBinary();
    xSemaphoreGive(state_lock);

    xTaskCreate(uart_task, "uart reader", 4096, NULL, 5, NULL);
    xTaskCreate(blink_task, "blink", 4096, NULL, 5, NULL);
}
