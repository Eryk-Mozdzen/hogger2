#include <stm32h5xx_hal.h>

#include "utils/task.h"

extern void SystemClock_Config();
extern void MX_GPIO_Init();
extern void MX_GPDMA1_Init();
extern void MX_GPDMA2_Init();
extern void MX_TIM1_Init();
extern void MX_TIM2_Init();
extern void MX_TIM5_Init();
extern void MX_TIM8_Init();
extern void MX_TIM12_Init();
extern void MX_TIM15_Init();
extern void MX_ADC1_Init();
extern void MX_ADC2_Init();
extern void MX_USART1_UART_Init();
extern void MX_UART4_Init();
extern void MX_I2C1_Init();
extern void MX_I2C2_Init();
extern void MX_SPI1_Init();
extern void MX_SPI3_Init();
extern void MX_ICACHE_Init();

int main() {

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_GPDMA1_Init();
    MX_GPDMA2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM5_Init();
    MX_TIM8_Init();
    MX_TIM12_Init();
    MX_TIM15_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_USART1_UART_Init();
    MX_UART4_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_SPI1_Init();
    MX_SPI3_Init();
    MX_ICACHE_Init();

    task_call_init();

    while(1) {
        task_call();
    }

    return 0;
}
