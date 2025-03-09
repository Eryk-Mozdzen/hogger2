#include <lrcp/frame.h>
#include <lrcp/stream.h>
#include <stm32u5xx_hal.h>
#include <string.h>

#include "com/stream.h"
#include "utils/mpack.h"
#include "utils/task.h"

#define MAX_REGISTERED   4
#define RX_TIME_WATCHDOG 1000

#define MIN(a, b) ((a) < (b) ? (a) : (b))

extern UART_HandleTypeDef huart1;

typedef struct {
    uint8_t buffer[10 * 1024];
    volatile uint32_t read;
    volatile uint32_t write;
} serial_fifo_t;

static lrcp_stream_t stream;
static lrcp_decoder_t decoder;
static serial_fifo_t fifo_rx;
static serial_fifo_t fifo_tx;
static volatile uint32_t transmission;
static uint32_t time_last_rx;

typedef struct {
    const char *type;
    stream_receiver_t receiver;
} registered_t;

static registered_t registered[MAX_REGISTERED] = {0};
static uint32_t count = 0;

static inline uint32_t fifo_pending(const serial_fifo_t *fifo) {
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

static void tick() {
    const uint32_t time = HAL_GetTick();
    const uint32_t rx_position =
        (sizeof(fifo_rx.buffer) - __HAL_DMA_GET_COUNTER(huart1.hdmarx)) % sizeof(fifo_rx.buffer);

    if(rx_position != fifo_rx.write) {
        time_last_rx = time;
    }

    fifo_rx.write = rx_position;

    if((time - time_last_rx) >= RX_TIME_WATCHDOG) {
        time_last_rx = time;
        HAL_UART_Receive_DMA(&huart1, fifo_rx.buffer, sizeof(fifo_rx.buffer));
    }

    const uint32_t tx_pending = fifo_pending(&fifo_tx);

    if(!transmission && tx_pending) {
        const uint32_t len = MIN(tx_pending, sizeof(fifo_tx.buffer) - fifo_tx.read);

        transmission = 1;
        HAL_UART_Transmit_DMA(&huart1, &fifo_tx.buffer[fifo_tx.read], len);
        fifo_tx.read += len;
        fifo_tx.read %= sizeof(fifo_tx.buffer);
    }
}

static void isr_transmit(UART_HandleTypeDef *huart) {
    (void)huart;

    transmission = 0;

    const uint32_t tx_pending = fifo_pending(&fifo_tx);

    if(tx_pending) {
        const uint32_t len = MIN(tx_pending, sizeof(fifo_tx.buffer) - fifo_tx.read);

        transmission = 1;
        HAL_UART_Transmit_DMA(&huart1, &fifo_tx.buffer[fifo_tx.read], len);
        fifo_tx.read += len;
        fifo_tx.read %= sizeof(fifo_tx.buffer);
    }
}

void stream_register(const char *type, const stream_receiver_t receiver) {
    registered[count].type = type;
    registered[count].receiver = receiver;
    count++;
}

void stream_transmit(const mpack_t *mpack) {
    lrcp_stream_write(&stream, mpack->buffer, mpack->size);
}

static void init() {
    HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, isr_transmit);

    lrcp_stream_init(&stream, NULL, reader, writer);
    lrcp_frame_decoder_init(&decoder);

    time_last_rx = 0;
    transmission = 0;

    fifo_rx.read = 0;
    fifo_rx.write = 0;
    fifo_tx.read = 0;
    fifo_tx.write = 0;

    HAL_UART_Receive_DMA(&huart1, fifo_rx.buffer, sizeof(fifo_rx.buffer));
}

static void loop() {
    tick();

    static uint8_t decoded[2048];
    const uint32_t size = lrcp_frame_decode(&stream, &decoder, decoded, sizeof(decoded));

    char type[32];
    mpack_t mpack;
    if(mpack_create_from(&mpack, type, decoded, size)) {
        for(uint32_t i = 0; i < count; i++) {
            if((strcmp(type, registered[i].type) == 0) && registered[i].receiver) {
                mpack_t copy;
                mpack_copy(&copy, &mpack);
                registered[i].receiver(&copy);
            }
        }
    }
}

TASK_REGISTER_INIT(init)
TASK_REGISTER_NONSTOP(loop)
