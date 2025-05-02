#include <stm32h5xx_hal.h>
#include <string.h>

#include "com/stream.h"
#include "utils/task.h"

#define REGISTERED_MAX_NUM 8
#define REGISTERED_MAX_DIM 32

#define SECTOR         FLASH_SECTOR_31
#define SECTOR_BANK    FLASH_BANK_2
#define SECTOR_ADDRESS 0x0807E000
#define SECTOR_SIZE    0x2000

typedef struct {
    const char *name;
    float *vector;
    uint32_t dim;
} registered_t;

static registered_t registered[REGISTERED_MAX_NUM] = {0};
static uint32_t count = 0;
static uint8_t buffer_flash[SECTOR_SIZE];
static uint8_t buffer_mpack[SECTOR_SIZE];

void config_register(const char *name, float *vector, const uint32_t dim) {
    registered[count].name = name;
    registered[count].vector = vector;
    registered[count].dim = (dim <= REGISTERED_MAX_DIM) ? dim : REGISTERED_MAX_DIM;
    count++;
}

static bool flash_read(mpack_t *mpack) {
    memcpy(buffer_flash, (void *)SECTOR_ADDRESS, SECTOR_SIZE);

    uint32_t size;
    memcpy(&size, &buffer_flash[0], 4);

    if(size > SECTOR_SIZE) {
        return false;
    }

    mpack_create_from(mpack, &buffer_flash[4], size);

    return true;
}

static void flash_write(const mpack_t *mpack) {
    memset(buffer_flash, 0xFF, SECTOR_SIZE);

    memcpy(&buffer_flash[0], &mpack->size, 4);
    memcpy(&buffer_flash[4], mpack->buffer, mpack->size);

    HAL_ICACHE_Disable();
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
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, SECTOR_ADDRESS + i,
                          (uint32_t)buffer_flash + i);
    }

    HAL_FLASH_Lock();
    HAL_ICACHE_Enable();
}

static void update() {
    for(uint32_t i = 0; i < count; i++) {
        memset(registered[i].vector, 0, registered[i].dim * sizeof(float));
    }

    mpack_t config;
    if(!flash_read(&config)) {
        return;
    }

    uint32_t map_size = 0;
    if(!mpack_read_map(&config, &map_size)) {
        return;
    }

    for(uint32_t i = 0; i < map_size; i++) {
        char key[32];
        if(!mpack_read_str(&config, key, sizeof(key))) {
            return;
        }

        float val[REGISTERED_MAX_DIM] = {0};
        if(!mpack_read_float32_array(&config, val, REGISTERED_MAX_DIM, NULL)) {
            return;
        }

        for(uint32_t j = 0; j < count; j++) {
            if(strcmp(registered[j].name, key) == 0) {
                memcpy(registered[j].vector, val, registered[j].dim * sizeof(float));
            }
        }
    }
}

static void request(mpack_t *message) {
    mpack_t config;
    if(!flash_read(&config)) {
        return;
    }

    uint32_t config_size;
    if(!mpack_read_map(&config, &config_size)) {
        return;
    }

    uint32_t message_size;
    if(!mpack_read_array(message, &message_size)) {
        return;
    }

    if(message_size == 0) {
        mpack_t response;
        mpack_create_empty(&response, buffer_mpack, sizeof(buffer_mpack));
        cmp_write_map(&response.cmp, 1);
        cmp_write_str(&response.cmp, "config", 6);
        cmp_write_map(&response.cmp, config_size);

        for(uint32_t i = 0; i < config_size; i++) {
            char key[32];
            if(!mpack_read_str(&config, key, sizeof(key))) {
                return;
            }

            float val[REGISTERED_MAX_DIM];
            uint32_t dim;
            if(!mpack_read_float32_array(&config, val, REGISTERED_MAX_DIM, &dim)) {
                return;
            }

            cmp_write_str(&response.cmp, key, strlen(key));
            cmp_write_array(&response.cmp, dim);
            for(uint32_t k = 0; k < dim; k++) {
                cmp_write_float(&response.cmp, val[k]);
            }
        }

        stream_transmit(&response);
        return;
    }

    mpack_t response;
    mpack_create_empty(&response, buffer_mpack, sizeof(buffer_mpack));
    cmp_write_map(&response.cmp, 1);
    cmp_write_str(&response.cmp, "config", 6);
    cmp_write_map(&response.cmp, message_size);

    for(uint32_t i = 0; i < message_size; i++) {
        char message_key[32];
        if(!mpack_read_str(message, message_key, sizeof(message_key))) {
            return;
        }

        mpack_t config_copy;
        mpack_create_copy(&config_copy, &config);

        bool found = false;

        cmp_write_str(&response.cmp, message_key, strlen(message_key));

        for(uint32_t j = 0; j < config_size; j++) {
            char key[32];
            if(!mpack_read_str(&config_copy, key, sizeof(key))) {
                return;
            }

            float val[REGISTERED_MAX_DIM];
            uint32_t dim;
            if(!mpack_read_float32_array(&config_copy, val, REGISTERED_MAX_DIM, &dim)) {
                return;
            }

            if(strcmp(key, message_key) == 0) {
                found = true;
                cmp_write_array(&response.cmp, dim);
                for(uint32_t k = 0; k < dim; k++) {
                    cmp_write_float(&response.cmp, val[k]);
                }
                break;
            }
        }

        if(!found) {
            cmp_write_array(&response.cmp, 0);
        }
    }

    stream_transmit(&response);
}

static void clear(mpack_t *mpack) {
    (void)mpack;

    mpack_t empty;
    mpack_create_empty(&empty, buffer_mpack, sizeof(buffer_mpack));
    cmp_write_map(&empty.cmp, 0);

    flash_write(&empty);

    update();
}

static void store(mpack_t *message) {
    mpack_t config;
    if(!flash_read(&config)) {
        return;
    }

    uint32_t config_size;
    if(!mpack_read_map(&config, &config_size)) {
        return;
    }

    uint32_t message_size;
    if(!mpack_read_map(message, &message_size)) {
        return;
    }

    uint32_t merged_size = config_size;

    {
        mpack_t message_copy;
        mpack_create_copy(&message_copy, message);

        for(uint32_t i = 0; i < message_size; i++) {
            char message_key[32];
            if(!mpack_read_str(&message_copy, message_key, sizeof(message_key))) {
                return;
            }

            float message_val[REGISTERED_MAX_DIM];
            uint32_t message_dim;
            if(!mpack_read_float32_array(&message_copy, message_val, REGISTERED_MAX_DIM,
                                         &message_dim)) {
                return;
            }

            mpack_t config_copy;
            mpack_create_copy(&config_copy, &config);

            bool config_not_found = true;

            for(uint32_t j = 0; j < config_size; j++) {
                char config_key[32];
                if(!mpack_read_str(&config_copy, config_key, sizeof(config_key))) {
                    return;
                }

                if(!mpack_read_float32_array(&config_copy, NULL, 0, NULL)) {
                    return;
                }

                if(strcmp(config_key, message_key) == 0) {
                    config_not_found = false;
                    break;
                }
            }

            if(config_not_found) {
                merged_size++;
            }
        }
    }

    mpack_t merged;
    mpack_create_empty(&merged, buffer_mpack, sizeof(buffer_mpack));
    cmp_write_map(&merged.cmp, merged_size);

    {
        mpack_t config_copy;
        mpack_create_copy(&config_copy, &config);

        for(uint32_t i = 0; i < config_size; i++) {
            char config_key[32];
            if(!mpack_read_str(&config_copy, config_key, sizeof(config_key))) {
                return;
            }

            float config_val[REGISTERED_MAX_DIM];
            uint32_t config_dim;
            if(!mpack_read_float32_array(&config_copy, config_val, REGISTERED_MAX_DIM,
                                         &config_dim)) {
                return;
            }

            mpack_t message_copy;
            mpack_create_copy(&message_copy, message);

            bool message_found = false;
            float message_val[REGISTERED_MAX_DIM];
            uint32_t message_dim;

            for(uint32_t j = 0; j < message_size; j++) {
                char message_key[32];
                if(!mpack_read_str(&message_copy, message_key, sizeof(message_key))) {
                    return;
                }

                if(!mpack_read_float32_array(&message_copy, message_val, REGISTERED_MAX_DIM,
                                             &message_dim)) {
                    return;
                }

                if(strcmp(config_key, message_key) == 0) {
                    message_found = true;
                    break;
                }
            }

            const uint32_t dim = message_found ? message_dim : config_dim;
            const float *val = message_found ? message_val : config_val;

            cmp_write_str(&merged.cmp, config_key, strlen(config_key));
            cmp_write_array(&merged.cmp, dim);
            for(uint32_t j = 0; j < dim; j++) {
                cmp_write_float(&merged.cmp, val[j]);
            }
        }
    }

    {
        mpack_t message_copy;
        mpack_create_copy(&message_copy, message);

        for(uint32_t i = 0; i < message_size; i++) {
            char message_key[32];
            if(!mpack_read_str(&message_copy, message_key, sizeof(message_key))) {
                return;
            }

            float message_val[REGISTERED_MAX_DIM];
            uint32_t message_dim;
            if(!mpack_read_float32_array(&message_copy, message_val, REGISTERED_MAX_DIM,
                                         &message_dim)) {
                return;
            }

            mpack_t config_copy;
            mpack_create_copy(&config_copy, &config);

            bool config_not_found = true;

            for(uint32_t j = 0; j < config_size; j++) {
                char config_key[32];
                if(!mpack_read_str(&config_copy, config_key, sizeof(config_key))) {
                    return;
                }

                if(!mpack_read_float32_array(&config_copy, NULL, 0, NULL)) {
                    return;
                }

                if(strcmp(config_key, message_key) == 0) {
                    config_not_found = false;
                    break;
                }
            }

            if(config_not_found) {
                cmp_write_str(&merged.cmp, message_key, strlen(message_key));
                cmp_write_array(&merged.cmp, message_dim);
                for(uint32_t j = 0; j < message_dim; j++) {
                    cmp_write_float(&merged.cmp, message_val[j]);
                }
            }
        }
    }

    flash_write(&merged);

    update();
}

TASK_REGISTER_INIT(update)
STREAM_REGISTER("config_req", request)
STREAM_REGISTER("config_clr", clear)
STREAM_REGISTER("config", store)
