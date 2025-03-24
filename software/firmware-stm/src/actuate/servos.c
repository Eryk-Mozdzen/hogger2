#include "actuate/dynamixel.h"
#include "com/telemetry.h"
#include "utils/task.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;

static dynamixel_t dynamixel1 = {
    .uart = &huart1,
};

static dynamixel_t dynamixel2 = {
    .uart = &huart4,
};

static dynamixel_servo_t *servo_1_x = NULL;
static dynamixel_servo_t *servo_1_y = NULL;
static dynamixel_servo_t *servo_2_x = NULL;
static dynamixel_servo_t *servo_2_y = NULL;

void servos_set_position(const float x1, const float y1, const float x2, const float y2) {
    servo_1_x->goal = x1;
    servo_1_y->goal = y1;
    servo_2_x->goal = x2;
    servo_2_y->goal = y2;
}

void servos_set_led(const bool x1, const bool y1, const bool x2, const bool y2) {
    servo_1_x->led = x1;
    servo_1_y->led = y1;
    servo_2_x->led = x2;
    servo_2_y->led = y2;
}

static void isr_transmit(UART_HandleTypeDef *huart) {
    dynamixel_transmit_callback(&dynamixel1, huart);
    dynamixel_transmit_callback(&dynamixel2, huart);
}

static void serialize(cmp_ctx_t *cmp, void *context) {
    const dynamixel_servo_t *servo = context;

    const uint8_t valid = ((HAL_GetTick() - servo->timestamp) <= 500);

    cmp_write_map(cmp, 5);
    cmp_write_str(cmp, "pos_ref", 7);
    cmp_write_float(cmp, servo->goal);
    cmp_write_str(cmp, "pos", 3);
    cmp_write_float(cmp, valid ? servo->position : NAN);
    cmp_write_str(cmp, "vel", 3);
    cmp_write_float(cmp, valid ? servo->velocity : NAN);
    cmp_write_str(cmp, "load", 4);
    cmp_write_float(cmp, valid ? servo->load : NAN);
    cmp_write_str(cmp, "temp", 4);
    cmp_write_float(cmp, valid ? servo->temperature : NAN);
}

static void init() {
    HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, isr_transmit);
    HAL_UART_RegisterCallback(&huart4, HAL_UART_TX_COMPLETE_CB_ID, isr_transmit);

    dynamixel_init(&dynamixel1);
    dynamixel_init(&dynamixel2);
    servo_1_x = dynamixel_register(&dynamixel1, 0);
    servo_1_y = dynamixel_register(&dynamixel1, 1);
    servo_2_x = dynamixel_register(&dynamixel2, 2);
    servo_2_y = dynamixel_register(&dynamixel2, 3);

    telemetry_register("servo_1_x", serialize, servo_1_x);
    telemetry_register("servo_1_y", serialize, servo_1_y);
    telemetry_register("servo_2_x", serialize, servo_2_x);
    telemetry_register("servo_2_y", serialize, servo_2_y);
}

static void loop() {
    dynamixel_tick(&dynamixel1);
    dynamixel_tick(&dynamixel2);
}

TASK_REGISTER_INIT(init)
TASK_REGISTER_PERIODIC(loop, 100)
