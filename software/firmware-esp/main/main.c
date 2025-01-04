#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "common/protocol/protocol.h"

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
static char state[BUFFER_SIZE] = {0};
static size_t state_size = 0;

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

static void protocol_cb_transmit(void *user, const void *data, const uint32_t size) {
    (void)user;
    uart_write_bytes(UART_NUM_0, data, size);
}

static void protocol_cb_receive(void *user, const uint8_t id, const uint32_t time, const void *payload, const uint32_t size) {
    (void)user;
    (void)id;
    (void)time;

    xSemaphoreTake(state_lock, portMAX_DELAY);
    memcpy(state, payload, size);
    state_size = size;
    xSemaphoreGive(state_lock);
}

static uint32_t protocol_cb_time(void *user) {
	(void)user;
	return xTaskGetTickCount()*portTICK_PERIOD_MS;
}

static void protocol_init() {
    protocol_lock = xSemaphoreCreateBinary();

    protocol.callback_tx = protocol_cb_transmit;
	protocol.callback_rx = protocol_cb_receive;
	protocol.callback_time = protocol_cb_time;
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
        const int bytes = uart_read_bytes(UART_NUM_0, data, sizeof(data), 1);

        for(int i=0; i<bytes; i++) {
            protocol.fifo_rx.buffer[protocol.fifo_rx.write] = data[i];
            protocol.fifo_rx.write++;
            protocol.fifo_rx.write %=protocol.fifo_rx.size;
        }

        size_t size;
        uart_get_tx_buffer_free_size(UART_NUM_0, &size);
        protocol.available = (size>(BUFFER_SIZE/2));

        xSemaphoreTake(protocol_lock, portMAX_DELAY);
        protocol_process(&protocol);
        xSemaphoreGive(protocol_lock);

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

static esp_err_t http_handler_get(httpd_req_t *req) {
    xSemaphoreTake(state_lock, portMAX_DELAY);
    httpd_resp_send(req, state, state_size);
    xSemaphoreGive(state_lock);

    return ESP_OK;
}

static esp_err_t http_handler_post(httpd_req_t *req) {
    char payload[BUFFER_SIZE] = {0};

    const int size = httpd_req_recv(req, payload, sizeof(payload));

    if(size>0) {
        xSemaphoreTake(protocol_lock, portMAX_DELAY);
        protocol_enqueue(&protocol, 0, payload, size);
        xSemaphoreGive(protocol_lock);
    }

    httpd_resp_send(req, NULL, 0);

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
