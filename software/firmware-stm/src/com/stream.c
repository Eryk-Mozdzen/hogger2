#include <lrcp/frame.h>
#include <lrcp/stream.h>
#include <main.h>
#include <stm32h5xx_hal.h>
#include <string.h>

#include "com/stream.h"
#include "utils/mpack.h"
#include "utils/task.h"

#define MAX_REGISTERED   16
#define TRANSACTION_SIZE 2048
#define HEADER_SIZE      sizeof(uint32_t)
#define PAYLOAD_SIZE     (TRANSACTION_SIZE - HEADER_SIZE)

#define MAX(a, b) ((a) > (b) ? (a) : (b))

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim6;

typedef struct {
    uint8_t buffer[10 * 1024];
    uint32_t read;
    uint32_t write;
} fifo_t;

typedef struct {
    uint8_t rx_buffer[TRANSACTION_SIZE];
    uint8_t tx_buffer[TRANSACTION_SIZE];

    uint32_t rx_buffer_size;
    uint32_t tx_buffer_size;

    volatile uint32_t complete;
} master_t;

typedef struct {
    const char *type;
    stream_receiver_t receiver;
} registered_t;

static lrcp_stream_t stream;
static lrcp_decoder_t decoder;
static fifo_t fifo_rx;
static fifo_t fifo_tx;

static master_t master;

static registered_t registered[MAX_REGISTERED] = {0};
static uint32_t count = 0;

static inline uint32_t fifo_pending(const fifo_t *fifo) {
    return ((fifo->read > fifo->write ? sizeof(fifo->buffer) : 0) + fifo->write) - fifo->read;
}

static uint32_t reader(void *context, void *data, const uint32_t data_capacity) {
    (void)context;
    uint32_t i;

    for(i = 0; fifo_pending(&fifo_rx) && i < data_capacity; i++) {
        ((uint8_t *)data)[i] = fifo_rx.buffer[fifo_rx.read];
        fifo_rx.read++;
        fifo_rx.read %= sizeof(fifo_rx.buffer);
    }

    return i;
}

static uint32_t writer(void *context, const void *data, const uint32_t data_size) {
    (void)context;

    for(uint32_t i = 0; i < data_size; i++) {
        fifo_tx.buffer[fifo_tx.write] = ((const uint8_t *)data)[i];
        fifo_tx.write++;
        fifo_tx.write %= sizeof(fifo_tx.buffer);
    }

    return data_size;
}

void stream_register(const char *type, const stream_receiver_t receiver) {
    registered[count].type = type;
    registered[count].receiver = receiver;
    count++;
}

void stream_transmit(const mpack_t *mpack) {
    lrcp_frame_encode(&stream, mpack->buffer, mpack->size);
}

static void transaction_start() {
    master.complete = 0;

    memset(master.rx_buffer, 0, sizeof(master.rx_buffer));
    memset(master.tx_buffer, 0, sizeof(master.tx_buffer));

    master.rx_buffer_size = 0;
    master.tx_buffer_size = 0;

    for(master.tx_buffer_size = 0; (master.tx_buffer_size < PAYLOAD_SIZE) && fifo_pending(&fifo_tx);
        master.tx_buffer_size++) {
        master.tx_buffer[HEADER_SIZE + master.tx_buffer_size] = fifo_tx.buffer[fifo_tx.read];
        fifo_tx.read++;
        fifo_tx.read %= sizeof(fifo_tx.buffer);
    }

    memcpy(master.tx_buffer, &master.tx_buffer_size, HEADER_SIZE);

    HAL_GPIO_WritePin(ESP_CS_GPIO_Port, ESP_CS_Pin, 0);
    HAL_SPI_TransmitReceive_DMA(&hspi1, master.tx_buffer, master.rx_buffer, TRANSACTION_SIZE);
}

static void transaction_isr(SPI_HandleTypeDef *hspi) {
    (void)hspi;

    HAL_GPIO_WritePin(ESP_CS_GPIO_Port, ESP_CS_Pin, 1);

    memcpy(&master.rx_buffer_size, master.rx_buffer, HEADER_SIZE);

    if(master.rx_buffer_size > PAYLOAD_SIZE) {
        master.rx_buffer_size = PAYLOAD_SIZE;
    }

    master.complete = 1;
}

static void transaction_end() {
    for(uint32_t i = 0; i < master.rx_buffer_size; i++) {
        fifo_rx.buffer[fifo_rx.write] = master.rx_buffer[HEADER_SIZE + i];
        fifo_rx.write++;
        fifo_rx.write %= sizeof(fifo_rx.buffer);
    }
}

static void init() {
    HAL_SPI_RegisterCallback(&hspi1, HAL_SPI_TX_RX_COMPLETE_CB_ID, transaction_isr);

    lrcp_stream_init(&stream, NULL, reader, writer);
    lrcp_frame_decoder_init(&decoder);

    fifo_rx.read = 0;
    fifo_rx.write = 0;
    fifo_tx.read = 0;
    fifo_tx.write = 0;

    master.complete = 0;

    HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, 1);
    HAL_Delay(100);
}

static void decode() {
    static uint8_t decoded[10 * 1024];
    const uint32_t size = lrcp_frame_decode(&stream, &decoder, decoded, sizeof(decoded));

    if(size > 0) {
        mpack_t mpack;
        if(mpack_create_from(&mpack, decoded, size)) {
            char type[32] = {0};
            uint32_t type_size = sizeof(type);
            if(!cmp_read_str(&mpack.cmp, type, &type_size)) {
                return;
            }

            for(uint32_t i = 0; i < count; i++) {
                if((strncmp(type, registered[i].type, type_size) == 0) && registered[i].receiver) {
                    mpack_t copy;
                    mpack_copy(&copy, &mpack);
                    registered[i].receiver(&copy);
                }
            }
        }
    }
}

TASK_REGISTER_INIT(init)
TASK_REGISTER_PERIODIC(decode, 1000)
TASK_REGISTER_PERIODIC(transaction_start, 1000)
TASK_REGISTER_INTERRUPT(transaction_end, &master.complete);
