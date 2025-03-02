#include <FreeRTOS.h>
#include <stm32u5xx_hal.h>
#include <task.h>

extern TIM_HandleTypeDef htim6;

extern void SystemClock_Config();
extern void MX_GPIO_Init();
extern void MX_GPDMA1_Init();
extern void MX_TIM1_Init();
extern void MX_TIM2_Init();
extern void MX_TIM3_Init();
extern void MX_TIM4_Init();
extern void MX_TIM6_Init();
extern void MX_TIM8_Init();
extern void MX_USART1_UART_Init();
extern void MX_UART4_Init();
extern void MX_ADC1_Init();

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
    return HAL_OK;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim == &htim6) {
        HAL_IncTick();
    }
}

int main() {
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_GPDMA1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_TIM8_Init();
    MX_USART1_UART_Init();
    MX_UART4_Init();
    MX_ADC1_Init();

    HAL_TIM_Base_Start_IT(&htim6);

    vTaskStartScheduler();

    return 0;
}
