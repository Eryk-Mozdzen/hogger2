#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define WIFI_SSID       "Hogger^2"
#define WIFI_PASSWORD   "12345678"
#define JSON_SIZE_MAX   1024

static httpd_handle_t server = NULL;

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

static esp_err_t get_handler(httpd_req_t *req) {
    char json[JSON_SIZE_MAX];
    sprintf(json, "{\"tick\": %lu}", xTaskGetTickCount());
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

static esp_err_t post_handler(httpd_req_t *req) {
    char json[JSON_SIZE_MAX] = {0};
    httpd_req_recv(req, json, sizeof(json));
    httpd_resp_send(req, NULL, 0);
    ESP_LOGI("app", "%s", json);
    return ESP_OK;
}

static httpd_uri_t get_uri = {
    .uri      = "/get",
    .method   = HTTP_GET,
    .handler  = get_handler,
    .user_ctx = NULL
};

static httpd_uri_t post_uri = {
    .uri      = "/post",
    .method   = HTTP_POST,
    .handler  = post_handler,
    .user_ctx = NULL
};

static void http_init() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    if(httpd_start(&server, &config)!=ESP_OK) {
        return;
    }

    httpd_register_uri_handler(server, &get_uri);
    httpd_register_uri_handler(server, &post_uri);

    ESP_LOGI("app", "http server started");
}

void app_main() {
    const esp_err_t ret = nvs_flash_init();
    if((ret==ESP_ERR_NVS_NO_FREE_PAGES) || (ret==ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    wifi_init();
    http_init();
}
