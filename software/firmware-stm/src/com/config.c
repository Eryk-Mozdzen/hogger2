#include <stm32h5xx_hal.h>
#include <string.h>

#include "com/stream.h"
#include "utils/task.h"

#define REGISTERED_MAX 8

#define SECTOR         FLASH_SECTOR_31
#define SECTOR_BANK    FLASH_BANK_2
#define SECTOR_ADDRESS 0x0807E000
#define SECTOR_SIZE    0x2000

typedef struct {
    const char *name;
    float *vector;
    uint32_t dim;
} registered_t;

static registered_t registered[REGISTERED_MAX] = {0};
static uint32_t count = 0;
static uint8_t sector[SECTOR_SIZE];

void config_register(const char *name, float *vector, const uint32_t dim) {
    registered[count].name = name;
    registered[count].vector = vector;
    registered[count].dim = dim;
    count++;
}

static uint8_t read(mpack_t *mpack) {
    memcpy(sector, (void *)SECTOR_ADDRESS, SECTOR_SIZE);

    uint32_t size;
    memcpy(&size, &sector[0], 4);

    if(size > SECTOR_SIZE) {
        return 0;
    }

    if(!mpack_create_from(mpack, &sector[4], size)) {
        return 0;
    }

    return 1;
}

static void write(mpack_t *mpack) {
    memcpy(sector, (void *)SECTOR_ADDRESS, SECTOR_SIZE);

    memcpy(&sector[0], &mpack->size, 4);
    memcpy(&sector[4], mpack->buffer, mpack->size);

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = SECTOR_BANK,
        .Sector = SECTOR,
        .NbSectors = 1,
    };

    uint32_t error;
    HAL_FLASHEx_Erase(&erase, &error);

    for(uint32_t i = 0; i < SECTOR_SIZE; i += 16) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, SECTOR_ADDRESS + i, (uint32_t)sector + i);
    }

    HAL_FLASH_Lock();
}

static void load() {
    mpack_t config;
    if(!read(&config)) {
        return;
    }

    char type[32] = {0};
    uint32_t type_size = sizeof(type);
    if(!cmp_read_str(&config.cmp, type, &type_size)) {
        return;
    }
    if(strncmp(type, "config", type_size) != 0) {
        return;
    }

    uint32_t map_size = 0;
    if(!cmp_read_map(&config.cmp, &map_size)) {
        return;
    }

    for(uint32_t i = 0; i < map_size; i++) {
        char key[32];
        uint32_t key_size = sizeof(key);
        if(!cmp_read_str(&config.cmp, key, &key_size)) {
            return;
        }

        for(uint32_t j = 0; j < count; j++) {
            if(strncmp(registered[j].name, key, key_size) == 0) {
                mpack_read_array(&config, registered[j].vector, registered[j].dim);
            }
        }
    }
}

static void request(mpack_t *mpack) {
    (void)mpack;

    mpack_t config;
    if(read(&config)) {
        stream_transmit(&config);
    } else {
        uint8_t buffer[128];
        mpack_t error;
        mpack_create_empty(&error, buffer, sizeof(buffer));
        cmp_write_str(&error.cmp, "config", 6);
        cmp_write_array(&error.cmp, 0);
    }
}

static void store(mpack_t *mpack) {
    write(mpack);
    load();
    request(NULL);
}

TASK_REGISTER_INIT(load)
STREAM_REGISTER("config_req", request)
STREAM_REGISTER("config", store)
