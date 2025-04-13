#include <lrcp/frame.h>
#include <lrcp/stream.h>
#include <main.h>
#include <stm32h5xx_hal.h>
#include <string.h>

#include "com/stream.h"
#include "utils/interrupt.h"
#include "utils/mpack.h"
#include "utils/task.h"

#define MAX_REGISTERED 16

#define SPI_TRANSACTION_SIZE 2048
#define SPI_HEADER_SIZE      sizeof(uint16_t)
#define SPI_PAYLOAD_SIZE     (SPI_TRANSACTION_SIZE - SPI_HEADER_SIZE)

extern SPI_HandleTypeDef hspi1;

typedef struct {
    uint8_t buffer[32 * 1024];
    uint32_t read;
    uint32_t write;
} fifo_t;

typedef struct {
    uint8_t rx_buffer[SPI_TRANSACTION_SIZE];
    uint8_t tx_buffer[SPI_TRANSACTION_SIZE];

    uint16_t rx_size;
    uint16_t tx_size;

    lrcp_stream_t base;
    fifo_t fifo_rx;
    fifo_t fifo_tx;

    volatile uint32_t handshake;
    volatile uint32_t complete;
} stream_t;

typedef struct {
    const char *type;
    stream_receiver_t receiver;
} registered_t;

static stream_t stream;
static lrcp_decoder_t decoder;
static uint8_t decoded[32 * 1024];

static registered_t registered[MAX_REGISTERED] = {0};
static uint32_t count = 0;

static inline uint32_t fifo_pending(const fifo_t *fifo) {
    return ((fifo->read > fifo->write ? sizeof(fifo->buffer) : 0) + fifo->write) - fifo->read;
}

static uint32_t stream_reader(void *context, void *data, const uint32_t data_capacity) {
    (void)context;
    uint32_t i;

    for(i = 0; fifo_pending(&stream.fifo_rx) && i < data_capacity; i++) {
        ((uint8_t *)data)[i] = stream.fifo_rx.buffer[stream.fifo_rx.read];
        stream.fifo_rx.read++;
        stream.fifo_rx.read %= sizeof(stream.fifo_rx.buffer);
    }

    return i;
}

static uint32_t stream_writer(void *context, const void *data, const uint32_t data_size) {
    (void)context;

    for(uint32_t i = 0; i < data_size; i++) {
        stream.fifo_tx.buffer[stream.fifo_tx.write] = ((const uint8_t *)data)[i];
        stream.fifo_tx.write++;
        stream.fifo_tx.write %= sizeof(stream.fifo_tx.buffer);
    }

    return data_size;
}

void stream_register(const char *type, const stream_receiver_t receiver) {
    registered[count].type = type;
    registered[count].receiver = receiver;
    count++;
}

void stream_transmit(const mpack_t *mpack) {
    lrcp_frame_encode(&stream.base, mpack->buffer, mpack->size);
}

static void stream_decode() {
    while(1) {
        const uint32_t size = lrcp_frame_decode(&stream.base, &decoder, decoded, sizeof(decoded));

        if(!size) {
            return;
        }

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

static void transaction_handshake() {
    stream.handshake = 1;
}

static void transaction_start() {
    for(stream.tx_size = 0; (stream.tx_size < SPI_PAYLOAD_SIZE) && fifo_pending(&stream.fifo_tx);
        stream.tx_size++) {
        stream.tx_buffer[SPI_HEADER_SIZE + stream.tx_size] =
            stream.fifo_tx.buffer[stream.fifo_tx.read];
        stream.fifo_tx.read++;
        stream.fifo_tx.read %= sizeof(stream.fifo_tx.buffer);
    }

    memcpy(stream.tx_buffer, &stream.tx_size, SPI_HEADER_SIZE);

    HAL_GPIO_WritePin(ESP_CS_GPIO_Port, ESP_CS_Pin, 0);
    HAL_SPI_TransmitReceive_DMA(&hspi1, stream.tx_buffer, stream.rx_buffer, SPI_TRANSACTION_SIZE);
}

static void transaction_complete(SPI_HandleTypeDef *hspi) {
    (void)hspi;

    HAL_GPIO_WritePin(ESP_CS_GPIO_Port, ESP_CS_Pin, 1);

    memcpy(&stream.rx_size, stream.rx_buffer, SPI_HEADER_SIZE);

    if(stream.rx_size > SPI_PAYLOAD_SIZE) {
        stream.rx_size = SPI_PAYLOAD_SIZE;
    }

    stream.complete = 1;
}

static void transaction_finalize() {
    for(uint32_t i = 0; i < stream.rx_size; i++) {
        stream.fifo_rx.buffer[stream.fifo_rx.write] = stream.rx_buffer[SPI_HEADER_SIZE + i];
        stream.fifo_rx.write++;
        stream.fifo_rx.write %= sizeof(stream.fifo_rx.buffer);
    }
}

static void init() {
    lrcp_stream_init(&stream.base, NULL, stream_reader, stream_writer);
    lrcp_frame_decoder_init(&decoder);

    stream.fifo_rx.read = 0;
    stream.fifo_rx.write = 0;
    stream.fifo_tx.read = 0;
    stream.fifo_tx.write = 0;

    stream.complete = 0;

    HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, 1);
    HAL_Delay(100);

    HAL_SPI_RegisterCallback(&hspi1, HAL_SPI_TX_RX_COMPLETE_CB_ID, transaction_complete);
    interrupt_register(transaction_handshake, ESP_HANDSHAKE_Pin);
}

TASK_REGISTER_INIT(init)
TASK_REGISTER_PERIODIC(stream_decode, 1000)
TASK_REGISTER_INTERRUPT(transaction_start, &stream.handshake)
TASK_REGISTER_INTERRUPT(transaction_finalize, &stream.complete)
