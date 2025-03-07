#include <lrcp/frame.h>
#include <lrcp/stream.h>
#include <stm32u5xx_hal.h>
#include <string.h>

#include "com/stream.h"
#include "utils/mpack.h"
#include "utils/tasks.h"

#define MAX_REGISTERED   4
#define RX_TIME_WATCHDOG 1000

#define MIN(a, b) ((a) < (b) ? (a) : (b))

extern UART_HandleTypeDef huart1;

typedef struct {
    uint8_t buffer[10 * 1024];
    volatile uint32_t read;
    volatile uint32_t write;
} serial_fifo_t;

typedef struct {
    lrcp_stream_t base;
    UART_HandleTypeDef *uart;
    serial_fifo_t fifo_rx;
    serial_fifo_t fifo_tx;
    volatile uint32_t transmission;
    uint32_t time_last_rx;
} serial_t;

typedef struct {
    const char *type;
    stream_receiver_t receiver;
} registered_t;

static registered_t registered[MAX_REGISTERED] = {0};
static uint32_t count = 0;

static lrcp_decoder_t decoder = {0};
static serial_t serial = {
    .uart = &huart1,
};

static inline uint32_t fifo_pending(const serial_fifo_t *fifo) {
    return ((fifo->read > fifo->write ? sizeof(fifo->buffer) : 0) + fifo->write) - fifo->read;
}

static uint32_t reader(void *context, void *data, const uint32_t data_capacity) {
    serial_t *serial = context;
    uint32_t i;

    for(i = 0; fifo_pending(&serial->fifo_rx) && i < data_capacity; i++) {
        ((uint8_t *) data)[i] = serial->fifo_rx.buffer[serial->fifo_rx.read];
        serial->fifo_rx.read++;
        serial->fifo_rx.read %= sizeof(serial->fifo_rx.buffer);
    }

    return i;
}

static uint32_t writer(void *context, const void *data, const uint32_t data_size) {
    serial_t *serial = context;

    for(uint32_t i = 0; i < data_size; i++) {
        serial->fifo_tx.buffer[serial->fifo_tx.write] = ((const uint8_t *) data)[i];
        serial->fifo_tx.write++;
        serial->fifo_tx.write %= sizeof(serial->fifo_tx.buffer);
    }

    return data_size;
}

static void tick() {
    const uint32_t time = HAL_GetTick();
    const uint32_t rx_position =
        (sizeof(serial.fifo_rx.buffer) - __HAL_DMA_GET_COUNTER(serial.uart->hdmarx)) %
        sizeof(serial.fifo_rx.buffer);

    if(rx_position != serial.fifo_rx.write) {
        serial.time_last_rx = time;
    }

    serial.fifo_rx.write = rx_position;

    if((time - serial.time_last_rx) >= RX_TIME_WATCHDOG) {
        serial.time_last_rx = time;
        HAL_UART_Receive_DMA(serial.uart, serial.fifo_rx.buffer, sizeof(serial.fifo_rx.buffer));
    }

    const uint32_t tx_pending = fifo_pending(&serial.fifo_tx);

    if(!serial.transmission && tx_pending) {
        const uint32_t len = MIN(tx_pending, sizeof(serial.fifo_tx.buffer) - serial.fifo_tx.read);

        serial.transmission = 1;
        HAL_UART_Transmit_DMA(serial.uart, &serial.fifo_tx.buffer[serial.fifo_tx.read], len);
        serial.fifo_tx.read += len;
        serial.fifo_tx.read %= sizeof(serial.fifo_tx.buffer);
    }
}

static void isr_transmit(UART_HandleTypeDef *huart) {
    (void) huart;

    serial.transmission = 0;

    const uint32_t tx_pending = fifo_pending(&serial.fifo_tx);

    if(tx_pending) {
        const uint32_t len = MIN(tx_pending, sizeof(serial.fifo_tx.buffer) - serial.fifo_tx.read);

        serial.transmission = 1;
        HAL_UART_Transmit_DMA(serial.uart, &serial.fifo_tx.buffer[serial.fifo_tx.read], len);
        serial.fifo_tx.read += len;
        serial.fifo_tx.read %= sizeof(serial.fifo_tx.buffer);
    }
}

void stream_register(const char *type, const stream_receiver_t receiver) {
    registered[count].type = type;
    registered[count].receiver = receiver;
    count++;
}

void stream_transmit(const mpack_t *mpack) {
    lrcp_stream_write(&serial.base, mpack->buffer, mpack->size);
}

static void init() {
    HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, isr_transmit);

    lrcp_stream_init(&serial.base, &serial, reader, writer);
    lrcp_frame_decoder_init(&decoder);

    serial.time_last_rx = 0;
    serial.transmission = 0;

    serial.fifo_rx.read = 0;
    serial.fifo_rx.write = 0;
    serial.fifo_tx.read = 0;
    serial.fifo_tx.write = 0;

    HAL_UART_Receive_DMA(serial.uart, serial.fifo_rx.buffer, sizeof(serial.fifo_rx.buffer));
}

static void loop() {
    tick();

    static uint8_t decoded[2048];
    const uint32_t size = lrcp_frame_decode(&serial.base, &decoder, decoded, sizeof(decoded));

    char type[32];
    mpack_t mpack;
    if(mpack_create_from(&mpack, type, decoded, size)) {
        for(uint32_t i = 0; i < count; i++) {
            if((strcmp(type, registered[i].type) == 0) && registered[i].receiver) {
                mpack_t copy = mpack;
                registered[i].receiver(&copy);
            }
        }
    }
}

TASKS_REGISTER(init, loop, 0)
