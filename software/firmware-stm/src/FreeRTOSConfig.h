#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
#include <stm32u5xx_hal.h>

extern TIM_HandleTypeDef htim4;

 /******************************************************************************/
 /* Hardware description related definitions. **********************************/
 /******************************************************************************/

#define configCPU_CLOCK_HZ               ( ( unsigned long ) 160000000 )
#define configENABLE_FPU                 1
#define configENABLE_MPU                 0
#define configENABLE_TRUSTZONE           0
#define configCHECK_HANDLER_INSTALLATION 0

 /******************************************************************************/
 /* Scheduling behaviour related definitions. **********************************/
 /******************************************************************************/

#define configTICK_RATE_HZ                         1000U
#define configUSE_PREEMPTION                       1
#define configUSE_TIME_SLICING                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION    0
#define configUSE_TICKLESS_IDLE                    0
#define configMAX_PRIORITIES                       16U
#define configMINIMAL_STACK_SIZE                   128U
#define configMAX_TASK_NAME_LEN                    32U
#define configTICK_TYPE_WIDTH_IN_BITS              TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                    1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES      32U
#define configQUEUE_REGISTRY_SIZE                  32U
#define configENABLE_BACKWARD_COMPATIBILITY        0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS    0
#define configSTACK_DEPTH_TYPE                     uint32_t
#define configMESSAGE_BUFFER_LENGTH_TYPE           uint32_t
#define configUSE_NEWLIB_REENTRANT                 0

 /******************************************************************************/
 /* Software timer related definitions. ****************************************/
 /******************************************************************************/

#define configUSE_TIMERS                1
#define configTIMER_TASK_PRIORITY       15U
#define configTIMER_TASK_STACK_DEPTH    1024U
#define configTIMER_QUEUE_LENGTH        100U

 /******************************************************************************/
 /* Memory allocation related definitions. *************************************/
 /******************************************************************************/

#define configSUPPORT_STATIC_ALLOCATION              1
#define configSUPPORT_DYNAMIC_ALLOCATION             0
#define configTOTAL_HEAP_SIZE                        0U
#define configAPPLICATION_ALLOCATED_HEAP             0
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP    0
#define configUSE_MINI_LIST_ITEM                     0

 /******************************************************************************/
 /* Interrupt nesting behaviour configuration. *********************************/
 /******************************************************************************/

#define configKERNEL_INTERRUPT_PRIORITY          5U
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     5U
#define configMAX_API_CALL_INTERRUPT_PRIORITY    5U

 /******************************************************************************/
 /* Hook and callback function related definitions. ****************************/
 /******************************************************************************/

#define configUSE_IDLE_HOOK                   0
#define configUSE_TICK_HOOK                   0
#define configUSE_MALLOC_FAILED_HOOK          0
#define configUSE_DAEMON_TASK_STARTUP_HOOK    0
#define configCHECK_FOR_STACK_OVERFLOW        0

 /******************************************************************************/
 /* Run time and task stats gathering related definitions. *********************/
 /******************************************************************************/

#define configGENERATE_RUN_TIME_STATS            1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() HAL_TIM_Base_Start(&htim4)
#define portGET_RUN_TIME_COUNTER_VALUE()         __HAL_TIM_GET_COUNTER(&htim4)
#define configUSE_TRACE_FACILITY                 1
#define configRECORD_STACK_HIGH_ADDRESS          1
#define configUSE_STATS_FORMATTING_FUNCTIONS     0
#define configKERNEL_PROVIDED_STATIC_MEMORY      1

 /******************************************************************************/
 /* Definitions that include or exclude functionality. *************************/
 /******************************************************************************/

#define configUSE_TASK_NOTIFICATIONS           1
#define configUSE_MUTEXES                      1
#define configUSE_RECURSIVE_MUTEXES            0
#define configUSE_COUNTING_SEMAPHORES          1
#define configUSE_QUEUE_SETS                   0
#define configUSE_APPLICATION_TASK_TAG         0
#define INCLUDE_vTaskPrioritySet               0
#define INCLUDE_uxTaskPriorityGet              0
#define INCLUDE_vTaskDelete                    0
#define INCLUDE_vTaskSuspend                   0
#define INCLUDE_vTaskDelayUntil                1
#define INCLUDE_vTaskDelay                     1
#define INCLUDE_xTaskGetSchedulerState         0
#define INCLUDE_xTaskGetCurrentTaskHandle      1
#define INCLUDE_uxTaskGetStackHighWaterMark    0
#define INCLUDE_xTaskGetIdleTaskHandle         0
#define INCLUDE_eTaskGetState                  0
#define INCLUDE_xTimerPendFunctionCall         1
#define INCLUDE_xTaskAbortDelay                0
#define INCLUDE_xTaskGetHandle                 0
#define INCLUDE_xTaskResumeFromISR             0

#endif /* FREERTOS_CONFIG_H */
