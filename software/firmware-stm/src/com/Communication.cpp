#include <lrcp/frame.h>
#include <main.h>
#include <stm32u5xx_hal.h>

#include "Internals.hpp"
#include "actuate/Servos.hpp"
#include "freertos/Mutex.hpp"
#include "freertos/Task.hpp"
#include "freertos/Timer.hpp"
#include "lrcp.hpp"

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
      transmitter{"com tx", this, &Communication::transmitterTask, 8},
      receiver{"com rx", this, &Communication::receiverTask, 8},
      timer{"com rx timeout", this, &Communication::timeout, 2000, pdTRUE} {
}

void Communication::transmitterTask() {
    uint8_t buffer[1024];

    if(xSemaphoreTake(lock, portMAX_DELAY)) {
        HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, transmitISR);
        xSemaphoreGive(lock);
    }

    while(true) {
        const size_t bytes =
            xMessageBufferReceive(internals::comTxQueue(), buffer, sizeof(buffer), portMAX_DELAY);

        if(xSemaphoreTake(lock, 100)) {
            HAL_UART_Transmit_DMA(&huart1, buffer, bytes);
            xSemaphoreGive(lock);
            ulTaskNotifyTake(pdTRUE, 100);
        }
    }
}

void Communication::receiverTask() {
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

            if(size == 1) {
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,
                                  payload[0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
                Servos_SetLed(payload[0], payload[0], payload[0], payload[0]);
            } else {
                xMessageBufferSend(internals::comRxQueue(), payload, size, 10);
            }
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
