#include <stm32u5xx_hal.h>

#include "utils/task.h"

extern void SystemClock_Config();
extern void MX_GPIO_Init();
extern void MX_GPDMA1_Init();
extern void MX_TIM1_Init();
extern void MX_TIM2_Init();
extern void MX_TIM8_Init();
extern void MX_USART1_UART_Init();
extern void MX_UART4_Init();
extern void MX_TIM3_Init();
extern void MX_I2C1_Init();
extern void MX_ADC1_Init();
extern void MX_SPI2_Init();

int main() {

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_GPDMA1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM8_Init();
    MX_USART1_UART_Init();
    MX_UART4_Init();
    MX_TIM3_Init();
    MX_I2C1_Init();
    MX_ADC1_Init();
    MX_SPI2_Init();

    void task_call_init();

    while(1) {
        task_call();
    }

    return 0;
}
