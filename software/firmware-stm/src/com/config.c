#include <stm32u5xx_hal.h>
#include <string.h>

#include "com/stream.h"
#include "utils/tasks.h"

#define REGISTERED_MAX 8
#define PAGE_BEGIN     0x0807E000
#define PAGE_SIZE      0x2000

typedef struct {
    const char *name;
    float *vector;
    uint32_t dim;
} registered_t;

static registered_t registered[REGISTERED_MAX] = {0};
static uint32_t count = 0;

void config_register(const char *name, float *vector, const uint32_t dim) {
    registered[count].name = name;
    registered[count].vector = vector;
    registered[count].dim = dim;
    count++;
}

static void nvm_store(const void *src, const uint32_t len) {
    uint8_t page[PAGE_SIZE];
    memcpy(page, (void *) PAGE_BEGIN, PAGE_SIZE);
    memcpy(page, src, len);

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks = FLASH_BANK_2,
        .Page = 31,
        .NbPages = 1,
    };

    uint32_t error;
    HAL_FLASHEx_Erase(&erase, &error);

    for(uint32_t i = 0; i < PAGE_SIZE; i += 16) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, PAGE_BEGIN + i, (uint32_t) (page) + i);
    }

    HAL_FLASH_Lock();
}

static void nvm_load() {
    mpack_t config;
    if(!mpack_create_from(&config, NULL, (void *) PAGE_BEGIN, PAGE_SIZE)) {
        return;
    }

    uint32_t map_size = 0;
    if(!cmp_read_map(&config.cmp, &map_size)) {
        return;
    }

    for(uint32_t i = 0; i < map_size; i++) {
        char sensor[32];
        uint32_t sensor_size = sizeof(sensor);
        if(!cmp_read_str(&config.cmp, sensor, &sensor_size)) {
            continue;
        }

        for(uint32_t j = 0; j < count; j++) {
            if(strcmp(registered[j].name, sensor) == 0) {
                mpack_read_array(&config, registered[j].vector, registered[j].dim);
                continue;
            }
        }
    }
}

static void receiver(mpack_t *mpack) {
    nvm_store(mpack->buffer, mpack->size);

    nvm_load();
}

TASKS_REGISTER(nvm_load, NULL, 0)
STREAM_REGISTER("config", receiver)
