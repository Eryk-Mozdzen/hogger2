#include "actuate/dynamixel.h"
#include "com/config.h"
#include "com/stream.h"
#include "com/telemetry.h"
#include "utils/task.h"

#define DEG2RAD 0.017453293f
#define MAX_DEG 10.f
#define MIN_DEG -10.f

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

static float offset[4] = {0};

static float validate(const float position) {
    if(isnan(position)) {
        return 0;
    }

    if(isinf(position)) {
        return 0;
    }

    if(position > (MAX_DEG * DEG2RAD)) {
        return (MAX_DEG * DEG2RAD);
    }

    if(position < (MIN_DEG * DEG2RAD)) {
        return (MIN_DEG * DEG2RAD);
    }

    return position;
}

void servos_set_position(const float phi_1,
                         const float theta_1,
                         const float phi_2,
                         const float theta_2) {
    servo_1_x->goal = validate(phi_1 + offset[0]);
    servo_1_y->goal = validate(theta_1 + offset[1]);
    servo_2_x->goal = validate(phi_2 + offset[2]);
    servo_2_y->goal = validate(theta_2 + offset[3]);
}

void servos_get_position(float *phi_1, float *theta_1, float *phi_2, float *theta_2) {
    *phi_1 = servo_1_x->position - offset[0];
    *theta_1 = servo_1_y->position - offset[1];
    *phi_2 = servo_2_x->position - offset[2];
    *theta_2 = servo_2_y->position - offset[3];
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
    servo_1_x = dynamixel_register(&dynamixel1, 0x00, DYNAMIXEL_DIRECTION_NORMAL);
    servo_1_y = dynamixel_register(&dynamixel1, 0x01, DYNAMIXEL_DIRECTION_NORMAL);
    servo_2_x = dynamixel_register(&dynamixel2, 0x03, DYNAMIXEL_DIRECTION_NORMAL);
    servo_2_y = dynamixel_register(&dynamixel2, 0x02, DYNAMIXEL_DIRECTION_REVERSE);

    telemetry_register("servo_1_x", serialize, servo_1_x);
    telemetry_register("servo_1_y", serialize, servo_1_y);
    telemetry_register("servo_2_x", serialize, servo_2_x);
    telemetry_register("servo_2_y", serialize, servo_2_y);
}

static void loop() {
    dynamixel_tick(&dynamixel1);
    dynamixel_tick(&dynamixel2);
}

static void blink(mpack_t *mpack) {
    bool state;
    if(mpack_read_bool(mpack, &state)) {
        servo_1_x->led = state;
        servo_1_y->led = state;
        servo_2_x->led = state;
        servo_2_y->led = state;
    }
}

TASK_REGISTER_INIT(init)
TASK_REGISTER_PERIODIC(loop, 100)
STREAM_REGISTER("blink", blink);
CONFIG_REGISTER("servo_offset", offset, 4)
