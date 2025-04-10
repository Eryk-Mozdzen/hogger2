#include <cmp/cmp.h>
#include <driver/gpio.h>
#include <driver/spi_slave.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <lrcp/frame.h>
#include <lrcp/stream.h>
#include <netdb.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>

#define WIFI_SSID        "Hogger^2"
#define WIFI_PASSWORD    "12345678"
#define RX_PORT          3333
#define TX_PORT          4444
#define BUFFER_SIZE      (10 * 1024)
#define TRANSACTION_SIZE 2048
#define HEADER_SIZE      sizeof(uint32_t)
#define PAYLOAD_SIZE     (TRANSACTION_SIZE - HEADER_SIZE)

typedef struct {
    uint8_t decoder_buffer[BUFFER_SIZE];
    uint8_t rx_buffer[TRANSACTION_SIZE];
    uint8_t tx_buffer[TRANSACTION_SIZE];

    uint32_t rx_buffer_size;
    uint32_t tx_buffer_size;

    SemaphoreHandle_t lock;
    lrcp_stream_t stream;
    lrcp_decoder_t decoder;

    QueueHandle_t rx_queue;
    QueueHandle_t tx_queue;
} slave_t;

typedef struct {
    uint8_t *buffer;
    size_t capacity;
    size_t size;
    size_t position;
} buffer_t;

static TimerHandle_t host_watchdog;
static TimerHandle_t station_watchdog;

static slave_t slave;

static int s_retry_num = 0;

static void slave_transaction_start(spi_slave_transaction_t *transaction) {
    (void)transaction;

    memset(slave.rx_buffer, 0, sizeof(slave.rx_buffer));
    memset(slave.tx_buffer, 0, sizeof(slave.tx_buffer));

    slave.rx_buffer_size = 0;
    slave.tx_buffer_size = 0;

    for(slave.tx_buffer_size = 0; slave.tx_buffer_size < PAYLOAD_SIZE; slave.tx_buffer_size++) {
        BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
        if(!xQueueReceiveFromISR(slave.tx_queue,
                                 &slave.tx_buffer[HEADER_SIZE + slave.tx_buffer_size],
                                 &pxHigherPriorityTaskWoken)) {
            break;
        }
    }

    memcpy(slave.tx_buffer, &slave.tx_buffer_size, HEADER_SIZE);
}

static void slave_transaction_end(spi_slave_transaction_t *transaction) {
    (void)transaction;

    memcpy(&slave.rx_buffer_size, slave.rx_buffer, HEADER_SIZE);

    if(slave.rx_buffer_size > PAYLOAD_SIZE) {
        slave.rx_buffer_size = PAYLOAD_SIZE;
    }

    for(uint32_t i = 0; i < slave.rx_buffer_size; i++) {
        BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(slave.rx_queue, &slave.rx_buffer[HEADER_SIZE + i],
                          &pxHigherPriorityTaskWoken);
    }
}

static uint32_t slave_reader(void *context, void *data, const uint32_t data_capacity) {
    (void)context;

    for(uint32_t i = 0; i < data_capacity; i++) {
        if(!xQueueReceive(slave.rx_queue, &((uint8_t *)data)[i], 0)) {
            return i;
        }
    }

    return data_capacity;
}

static uint32_t slave_writer(void *context, const void *data, const uint32_t data_size) {
    (void)context;

    for(uint32_t i = 0; i < data_size; i++) {
        if(!xQueueSend(slave.tx_queue, &((uint8_t *)data)[i], 0)) {
            return i;
        }
    }

    return data_size;
}

static void slave_init() {
    lrcp_stream_init(&slave.stream, NULL, slave_reader, slave_writer);
    lrcp_frame_decoder_init(&slave.decoder);

    slave.rx_queue = xQueueCreate(BUFFER_SIZE, 1);
    slave.tx_queue = xQueueCreate(BUFFER_SIZE, 1);
    slave.lock = xSemaphoreCreateMutex();

    const spi_bus_config_t buscfg = {
        .mosi_io_num = 4,
        .miso_io_num = 5,
        .sclk_io_num = 6,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TRANSACTION_SIZE,
    };

    const spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = 7,
        .queue_size = 64,
        .flags = 0,
        .post_setup_cb = slave_transaction_start,
        .post_trans_cb = slave_transaction_end,
    };

    spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);

    ESP_LOGI("app", "slave started");
}

void slave_task(void *arg) {
    ESP_LOGI("app", "slave task started");

    spi_slave_transaction_t transaction = {
        .length = TRANSACTION_SIZE * 8,
        .tx_buffer = &slave.tx_buffer,
        .rx_buffer = &slave.rx_buffer,
    };

    while(1) {
        spi_slave_transmit(SPI2_HOST, &transaction, portMAX_DELAY);
    }
}

bool buffer_reader(cmp_ctx_t *ctx, void *data, size_t count) {
    buffer_t *buf = (buffer_t *)ctx->buf;

    if((buf->position + count) > buf->capacity) {
        return false;
    }

    memcpy(data, buf->buffer + buf->position, count);
    buf->position += count;

    return true;
}

bool buffer_skipper(cmp_ctx_t *ctx, size_t count) {
    buffer_t *buf = (buffer_t *)ctx->buf;

    if((buf->position + count) > buf->capacity) {
        return false;
    }

    buf->position += count;

    return true;
}

size_t buffer_writer(cmp_ctx_t *ctx, const void *data, size_t count) {
    buffer_t *buf = (buffer_t *)ctx->buf;

    if((buf->position + count) > buf->capacity) {
        return 0;
    }

    memcpy(buf->buffer + buf->position, data, count);
    buf->position += count;
    buf->size = buf->position;

    return count;
}

static void blink_task(void *params) {
    (void)params;

    gpio_reset_pin(0);
    gpio_set_direction(0, GPIO_MODE_OUTPUT);
    gpio_set_level(0, 0);

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

        if(xSemaphoreTake(slave.lock, portMAX_DELAY)) {
            lrcp_frame_encode(&slave.stream, buffer.buffer, buffer.size);
            xSemaphoreGive(slave.lock);
        }

        gpio_set_level(0, led);
        vTaskDelay(pdMS_TO_TICKS(led ? 100 : 900));
    }
}

static void udp_broadcast_task(void *params) {
    (void)params;

    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if(sock < 0) {
        ESP_LOGE("udp", "Failed to create socket");
        return;
    }

    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif_sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if(!netif_sta || esp_netif_get_ip_info(netif_sta, &ip_info) != ESP_OK) {
        ESP_LOGE("app", "Failed to get IP/netmask info");
        close(sock);
        return;
    }

    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(TX_PORT),
        .sin_addr.s_addr = (ip_info.ip.addr & ip_info.netmask.addr) | (~ip_info.netmask.addr),
    };

    ESP_LOGI("app", "UDP broadcast on port %d", TX_PORT);

    while(1) {
        if(xSemaphoreTake(slave.lock, portMAX_DELAY)) {
            const uint32_t size = lrcp_frame_decode(
                &slave.stream, &slave.decoder, slave.decoder_buffer, sizeof(slave.decoder_buffer));

            xSemaphoreGive(slave.lock);

            if(size > 0) {
                xTimerReset(host_watchdog, portMAX_DELAY);

                sendto(sock, slave.decoder_buffer, size, 0, (struct sockaddr *)&dest_addr,
                       sizeof(dest_addr));

                // ESP_LOGI("app", "lrcp rx size %lu", size);
            }
        }

        vTaskDelay(1);
    }
}

static void udp_server_task(void *pvParameters) {
    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in addr = {
        .sin_family = AF_INET, .sin_port = htons(RX_PORT), .sin_addr.s_addr = htonl(INADDR_ANY)};

    bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    char buffer[BUFFER_SIZE];

    struct sockaddr_storage source_addr;
    socklen_t addr_len = sizeof(source_addr);

    ESP_LOGI("app", "UDP server started on port %d", RX_PORT);

    while(1) {
        const int size =
            recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&source_addr, &addr_len);

        if(size > 0) {
            xTimerReset(station_watchdog, portMAX_DELAY);

            if(xSemaphoreTake(slave.lock, portMAX_DELAY)) {
                lrcp_frame_encode(&slave.stream, buffer, size);
                xSemaphoreGive(slave.lock);
            }

            // ESP_LOGI("app", "lrcp tx size %d", size);
        }

        vTaskDelay(1);
    }

    close(sock);
    vTaskDelete(NULL);
}

static void wifi_event(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if(s_retry_num < 10) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI("wifi", "retry to connect to the AP");
        }
    } else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = event_data;

        s_retry_num = 0;

        ESP_LOGI("wifi", "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI("wifi", "connected to AP ssid: \"%s\" password: \"%s\"", WIFI_SSID, WIFI_PASSWORD);

        xTaskCreate(udp_broadcast_task, "udp broadcast", 4096, NULL, 5, NULL);
        xTaskCreate(udp_server_task, "udp server", 16384, NULL, 5, NULL);
    }
}

static void wifi_init() {
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event,
                                                        NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event,
                                                        NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
                .ssid = WIFI_SSID,
                .password = WIFI_PASSWORD,
                .threshold.authmode = WIFI_AUTH_WPA2_PSK,
                }
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void host_timeout(TimerHandle_t timer) {
    (void)timer;

    // ESP_LOGW("app", "host timeout");
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

    lrcp_frame_encode(&slave.stream, buffer.buffer, buffer.size);

    // ESP_LOGW("app", "station timeout");
}

void app_main() {
    const esp_err_t result = nvs_flash_init();
    if((result == ESP_ERR_NVS_NO_FREE_PAGES) || (result == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    slave_init();
    wifi_init();

    host_watchdog = xTimerCreate("host wdg", pdMS_TO_TICKS(1000), true, NULL, host_timeout);
    station_watchdog = xTimerCreate("sta wdg", pdMS_TO_TICKS(1000), true, NULL, station_timeout);

    xTimerStart(host_watchdog, portMAX_DELAY);
    xTimerStart(station_watchdog, portMAX_DELAY);

    xTaskCreate(blink_task, "blink", 4096, NULL, 5, NULL);
    xTaskCreate(slave_task, "slave", 8192, NULL, 5, NULL);
}
