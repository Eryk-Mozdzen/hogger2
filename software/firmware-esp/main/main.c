#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <cJSON.h>
#include <cmp/cmp.h>
#include <lrcp/stream.h>
#include <lrcp/frame.h>

#define WIFI_SSID       "Hogger^2"
#define WIFI_PASSWORD   "12345678"
#define RX_PORT         3333
#define TX_PORT         4444
#define BUFFER_SIZE     (4*1024)

typedef struct {
    lrcp_stream_t base;
    QueueHandle_t queue;
} serial_t;

typedef struct {
    uint8_t *buffer;
    size_t capacity;
    size_t size;
    size_t position;
} buffer_t;

static TimerHandle_t host_watchdog;
static TimerHandle_t station_watchdog;

static SemaphoreHandle_t state_lock;
static uint8_t state_data[BUFFER_SIZE];
static size_t state_size = 0;

static serial_t serial;

static uint32_t serial_reader(void *context, void *data, const uint32_t data_capacity) {
    (void)context;

    const int result = uart_read_bytes(UART_NUM_0, data, data_capacity, 1);

    if(result<0) {
        return 0;
    }

    return result;
}

static uint32_t serial_writer(void *context, const void *data, const uint32_t data_size) {
    (void)context;

    const int result = uart_write_bytes(UART_NUM_0, data, data_size);

    if(result<0) {
        return 0;
    }

    return result;
}

static void serial_init() {
    lrcp_stream_init(&serial.base, NULL, serial_reader, serial_writer);

    const uart_config_t config = {
        .baud_rate = 230400,
        .data_bits = UART_DATA_8_BITS,
        .stop_bits = UART_STOP_BITS_1,
        .parity = UART_PARITY_DISABLE,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_NUM_0, &config);
    uart_set_pin(UART_NUM_0, 21, 20, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, BUFFER_SIZE, BUFFER_SIZE, BUFFER_SIZE, &serial.queue, 0);

    ESP_LOGI("app", "serial started");
}

static void serial_receive_task(void *params) {
    (void)params;

    uint8_t buffer[BUFFER_SIZE];
    lrcp_decoder_t decoder;
    lrcp_frame_decoder_init(&decoder);

    ESP_LOGI("app", "serial receive task started");

    while(1) {
        const uint32_t size = lrcp_frame_decode(&serial.base, &decoder, buffer, sizeof(buffer));

        if(size>0) {
            xSemaphoreTake(state_lock, portMAX_DELAY);
            memcpy(state_data, buffer, size);
            state_size = size;
            xSemaphoreGive(state_lock);
            xTimerReset(host_watchdog, portMAX_DELAY);
        } else {
            vTaskDelay(1);
        }
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
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .channel = 1,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &config);
    esp_wifi_start();

    ESP_LOGI("app", "wifi started ssid: \"%s\" password: \"%s\"", WIFI_SSID, WIFI_PASSWORD);
}

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

    if((buf->position+count)>buf->capacity) {
        return 0;
    }

    memcpy(buf->buffer+buf->position, data, count);
    buf->position +=count;
    buf->size = buf->position;

    return count;
}

void json_to_msgpack(cmp_ctx_t *cmp, const cJSON *json) {
    if(cJSON_IsObject(json)) {
        const int size = cJSON_GetArraySize(json);
        cmp_write_map(cmp, size);

        cJSON *child = NULL;
        cJSON_ArrayForEach(child, json) {
            cmp_write_str(cmp, child->string, strlen(child->string));
            json_to_msgpack(cmp, child);
        }

        return;
    }

    if(cJSON_IsArray(json)) {
        const int size = cJSON_GetArraySize(json);
        cmp_write_array(cmp, size);

        cJSON *element = NULL;
        cJSON_ArrayForEach(element, json) {
            json_to_msgpack(cmp, element);
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

cJSON *msgpack_to_json(cmp_ctx_t *cmp) {
    cmp_object_t object;
    if(!cmp_read_object(cmp, &object)) {
        ESP_LOGE("MSGPACK to JSON", "parse error (in object): %s", cmp_strerror(cmp));
        return NULL;
    }

    uint32_t array_size = 0;
    if(cmp_object_as_array(&object, &array_size)) {
        cJSON *array = cJSON_CreateArray();

        for(uint32_t i=0; i<array_size; i++) {
            cJSON_AddItemToArray(array, msgpack_to_json(cmp));
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
                ESP_LOGE("MSGPACK to JSON", "parse error (in map %lu): %s", map_size, cmp_strerror(cmp));
                return NULL;
            }

            cJSON_AddItemToObject(object, key, msgpack_to_json(cmp));
        }

        return object;
    }

    char str[256];
    uint32_t str_len = 0;
    if(cmp_object_as_str(&object, &str_len)) {
        if(!cmp_object_to_str(cmp, &object, str, sizeof(str))) {
            ESP_LOGE("MSGPACK to JSON", "parse error (in str): %s", cmp_strerror(cmp));
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

static void udp_broadcast_task(void *params) {
    (void)params;

    const int udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if(udp_sock<0) {
        ESP_LOGE("app", "Failed to create UDP socket");
        vTaskDelete(NULL);
    }

    int broadcast = 1;
    setsockopt(udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(TX_PORT),
        .sin_addr.s_addr = inet_addr("192.168.4.255"),
    };

    ESP_LOGI("app", "UDP broadcasting on port %d", TX_PORT);

    cJSON *json = NULL;

    while(1) {
        xSemaphoreTake(state_lock, portMAX_DELAY);

        if(state_size>0) {
            buffer_t buffer = {
                .buffer = state_data,
                .capacity = state_size,
                .size = state_size,
                .position = 0,
            };
            cmp_ctx_t cmp;
            cmp_init(&cmp, &buffer, buffer_reader, NULL, NULL);
            json = msgpack_to_json(&cmp);

            if(!json) {
                ESP_LOGE("app", "JSON parser failed, buffer.size = %d", buffer.size);
            }
        } else {
            json = cJSON_CreateObject();
            cJSON_AddItemToObject(json, "error", cJSON_CreateString("host_timeout"));
        }

        xSemaphoreGive(state_lock);

        if(json) {
            char *str = cJSON_Print(json);
            cJSON_Delete(json);

            sendto(udp_sock, str, strlen(str), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

            free(str);
        }

        vTaskDelay(1);
    }
}

void udp_server_task(void *pvParameters) {
    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(RX_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    char buffer_rx[BUFFER_SIZE];
    uint8_t buffer_tx[BUFFER_SIZE];

    struct sockaddr_storage source_addr;
    socklen_t addr_len = sizeof(source_addr);

    ESP_LOGI("app", "UDP server started on port %d", RX_PORT);

    while(1) {
        const int len = recvfrom(sock, buffer_rx, sizeof(buffer_rx), 0, (struct sockaddr *)&source_addr, &addr_len);

        if(len>0) {
            //ESP_LOGI("app", "%.*s", len, buffer_rx);

            cJSON *json = cJSON_Parse(buffer_rx);
            if(!json) {
                ESP_LOGE("app", "Error parsing incoming JSON");
                continue;
            }

            buffer_t buffer = {
                .buffer = buffer_tx,
                .capacity = sizeof(buffer_tx),
                .size = 0,
                .position = 0,
            };
            cmp_ctx_t cmp;
            cmp_init(&cmp, &buffer, NULL, NULL, buffer_writer);
            json_to_msgpack(&cmp, json);

            cJSON_Delete(json);

            xTimerReset(station_watchdog, portMAX_DELAY);

            lrcp_frame_encode(&serial.base, buffer.buffer, buffer.size);
        }

        vTaskDelay(1);
    }

    close(sock);
    vTaskDelete(NULL);
}

static void blink_task(void *params) {
    (void)params;

    gpio_reset_pin(10);
    gpio_set_direction(10, GPIO_MODE_OUTPUT);
    gpio_set_level(10, 0);

    bool led = false;

    ESP_LOGI("app", "blink task started");

    while(1) {
        led = !led;

        uint8_t buf[128];

        buffer_t buffer = {
            .buffer = buf,
            .capacity = sizeof(buf),
            .size = 0,
            .position = 0,
        };
        cmp_ctx_t cmp;
        cmp_init(&cmp, &buffer, NULL, NULL, buffer_writer);

        cmp_write_map(&cmp, 1);
        cmp_write_str(&cmp, "blink", 5);
        cmp_write_bool(&cmp, led);

        lrcp_frame_encode(&serial.base, buffer.buffer, buffer.size);

        gpio_set_level(10, led);
        vTaskDelay(pdMS_TO_TICKS(led ? 100 : 900));
    }
}

static void host_timeout(TimerHandle_t timer) {
    (void)timer;

    xSemaphoreTake(state_lock, portMAX_DELAY);
    memset(state_data, 0, sizeof(state_data));
    state_size = 0;
    xSemaphoreGive(state_lock);

    ESP_LOGW("app", "host timeout");
}

static void station_timeout(TimerHandle_t timer) {
    (void)timer;

    uint8_t buf[128];

    buffer_t buffer = {
        .buffer = buf,
        .capacity = sizeof(buf),
        .size = 0,
        .position = 0,
    };
    cmp_ctx_t cmp;
    cmp_init(&cmp, &buffer, NULL, NULL, buffer_writer);

    cmp_write_map(&cmp, 1);
    cmp_write_str(&cmp, "stop", 4);
    cmp_write_nil(&cmp);

    lrcp_frame_encode(&serial.base, buffer.buffer, buffer.size);

    ESP_LOGW("app", "station timeout");
}

void app_main() {
    const esp_err_t result = nvs_flash_init();
    if((result==ESP_ERR_NVS_NO_FREE_PAGES) || (result==ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    serial_init();
    wifi_init();

    state_lock = xSemaphoreCreateBinary();
    xSemaphoreGive(state_lock);

    host_watchdog = xTimerCreate("host watchdog", pdMS_TO_TICKS(1000), true, NULL, host_timeout);
    station_watchdog = xTimerCreate("station watchdog", pdMS_TO_TICKS(1000), true, NULL, station_timeout);

    xTimerStart(host_watchdog, portMAX_DELAY);
    xTimerStart(station_watchdog, portMAX_DELAY);

    xTaskCreate(blink_task, "blink", 4096, NULL, 5, NULL);
    xTaskCreate(serial_receive_task, "serial receive", 8192, NULL, 5, NULL);
    xTaskCreate(udp_broadcast_task, "udp broadcast", 4096, NULL, 5, NULL);
    xTaskCreate(udp_server_task, "udp server", 16384, NULL, 5, NULL);
}
