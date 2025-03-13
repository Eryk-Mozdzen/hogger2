#include <cmp/cmp.h>
#include <stdint.h>
#include <stm32u5xx_hal.h>
#include <string.h>

#include "com/stream.h"
#include "com/telemetry.h"
#include "utils/mpack.h"
#include "utils/task.h"

#define MAX 16

typedef struct {
    const char *name;
    telemetry_serialize_t serialize;
    void *context;
} registered_t;

static registered_t registered[MAX] = {0};
static uint32_t count = 0;

void telemetry_register(const char *name, const telemetry_serialize_t serialize, void *context) {
    registered[count].name = name;
    registered[count].serialize = serialize;
    registered[count].context = context;
    count++;
}

static void loop() {
    const uint32_t timestamp = HAL_GetTick();

    static uint8_t buffer[2048];
    mpack_t mpack;
    mpack_create_empty(&mpack, "telemetry", buffer, sizeof(buffer));

    cmp_write_map(&mpack.cmp, count + 1);
    cmp_write_str(&mpack.cmp, "timestamp", 9);
    cmp_write_u32(&mpack.cmp, timestamp);

    for(uint32_t i = 0; i < count; i++) {
        cmp_write_str(&mpack.cmp, registered[i].name, strlen(registered[i].name));
        if(registered[i].serialize) {
            registered[i].serialize(&mpack.cmp, registered[i].context);
        }
    }

    stream_transmit(&mpack);
}

TASK_REGISTER_PERIODIC(loop, 20000)
