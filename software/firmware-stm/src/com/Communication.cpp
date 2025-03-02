#include <lrcp/frame.h>
#include <stm32u5xx_hal.h>

#include "com/MailboxRegistry.hpp"
#include "com/TelemetrySerializer.hpp"
#include "freertos/Mutex.hpp"
#include "freertos/Task.hpp"
#include "freertos/Timer.hpp"
#include "utils/lrcp.hpp"

extern UART_HandleTypeDef huart1;

class Communication {
    uint8_t streamBuffer[1024];
    lrcp::Stream stream;

    void transmitterTask();
    void receiverTask();
    void timeout();

public:
    freertos::TaskMember<Communication, 2048> transmitter;
    freertos::TaskMember<Communication, 2048> receiver;
    freertos::TimerMember<Communication> timer;
    freertos::Mutex lock;

    Communication();
};

Communication communication;

static void transmitISR(UART_HandleTypeDef *huart) {
    (void) huart;

    BaseType_t pxHigherPriorityTaskWoken;
    vTaskNotifyGiveFromISR(communication.transmitter, &pxHigherPriorityTaskWoken);
}

Communication::Communication()
    : stream(streamBuffer, sizeof(streamBuffer)),
      transmitter{"com tx", this, &Communication::transmitterTask, 7},
      receiver{"com rx", this, &Communication::receiverTask, 8},
      timer{"com rx timeout", this, &Communication::timeout, 2000, pdTRUE} {
}

void Communication::transmitterTask() {
    const TelemetrySerializer &serializer = TelemetrySerializer::getInstance();

    uint8_t buffer[1024];
    uint8_t encoded[1024];
    uint32_t time = 0;

    if(xSemaphoreTake(lock, portMAX_DELAY)) {
        HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, transmitISR);
        xSemaphoreGive(lock);
    }

    while(true) {
        const uint32_t size = serializer.serialize(buffer, sizeof(buffer));

        lrcp::Stream stream(encoded, sizeof(encoded));
        lrcp_frame_encode(&stream, buffer, size);

        if(xSemaphoreTake(lock, 10)) {
            HAL_UART_Transmit_DMA(&huart1, stream.buffer, stream.wr);
            xSemaphoreGive(lock);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        vTaskDelayUntil(&time, 20);
    }
}

void Communication::receiverTask() {
    MailboxRegistry &registry = MailboxRegistry::getInstance();

    uint8_t payload[1024];
    uint32_t time = 0;

    if(xSemaphoreTake(lock, portMAX_DELAY)) {
        HAL_UART_Receive_DMA(&huart1, streamBuffer, sizeof(streamBuffer));
        xSemaphoreGive(lock);
    }

    lrcp_decoder_t decoder;
    lrcp_frame_decoder_init(&decoder);

    xTimerStart(timer, portMAX_DELAY);

    while(true) {
        if(xSemaphoreTake(lock, 100)) {
            stream.wr = (sizeof(streamBuffer) - __HAL_DMA_GET_COUNTER(huart1.hdmarx)) %
                        sizeof(streamBuffer);
            xSemaphoreGive(lock);
        }

        const uint32_t size = lrcp_frame_decode(&stream, &decoder, payload, sizeof(payload));

        if(size > 0) {
            xTimerReset(timer, portMAX_DELAY);

            registry.send(payload, size);
        }

        vTaskDelayUntil(&time, 1);
    }
}

void Communication::timeout() {
    if(xSemaphoreTake(lock, portMAX_DELAY)) {
        HAL_UART_Receive_DMA(&huart1, streamBuffer, sizeof(streamBuffer));
        xSemaphoreGive(lock);
    }
    stream.rd = 0;
}
