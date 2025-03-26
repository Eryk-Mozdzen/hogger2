#include <stm32h5xx_hal.h>
#include <string.h>

#include "com/stream.h"
#include "utils/task.h"

#define REGISTERED_MAX 8
#define SECTOR_BEGIN   0x0807E000
#define SECTOR_SIZE    0x2000

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

static void load() {
    const uint32_t *size = (uint32_t *)(SECTOR_BEGIN + 0);
    volatile uint8_t *flash = (volatile uint8_t *)(SECTOR_BEGIN + 4);

    if(*size > SECTOR_SIZE) {
        return;
    }

    uint8_t buffer[SECTOR_SIZE];
    for(uint32_t i = 0; i < *size; i++) {
        buffer[i] = flash[i];
    }

    mpack_t config;
    if(!mpack_create_from(&config, NULL, buffer, *size)) {
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
            return;
        }

        for(uint32_t j = 0; j < count; j++) {
            if(strcmp(registered[j].name, sensor) == 0) {
                mpack_read_array(&config, registered[j].vector, registered[j].dim);
                return;
            }
        }
    }
}

static void store(mpack_t *mpack) {
    uint8_t page[SECTOR_SIZE];
    memcpy(page, (void *)SECTOR_BEGIN, SECTOR_SIZE);

    memcpy(&page[0], &mpack->size, 4);
    memcpy(&page[4], mpack->buffer, mpack->size);

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = FLASH_BANK_2,
        .Sector = 31,
        .NbSectors = 1,
    };

    uint32_t error;
    HAL_FLASHEx_Erase(&erase, &error);

    for(uint32_t i = 0; i < SECTOR_SIZE; i += 16) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, SECTOR_BEGIN + i, (uint32_t)page + i);
    }

    HAL_FLASH_Lock();

    load();
}

static void request(mpack_t *mpack) {
    (void)mpack;

    mpack_t config;

    const uint32_t *size = (uint32_t *)(SECTOR_BEGIN + 0);
    uint32_t *flash = (uint32_t *)(SECTOR_BEGIN + 4);

    if(*size > SECTOR_SIZE) {
        return;
    }

    uint8_t buffer[SECTOR_SIZE];
    memcpy(buffer, flash, *size);

    if(mpack_create_from(&config, NULL, buffer, *size)) {
        stream_transmit(&config);
    }
}

//TASK_REGISTER_INIT(load)
//STREAM_REGISTER("config", store)
//STREAM_REGISTER("config_req", request)
