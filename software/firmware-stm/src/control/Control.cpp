#include <cmath>
#include <cstring>

#include "Internals.hpp"
#include "actuate/Motors.hpp"
#include "actuate/Servos.hpp"
#include "com/cmp.hpp"
#include "freertos/Mutex.hpp"
#include "freertos/Task.hpp"
#include "freertos/Timer.hpp"

class Control {
    freertos::TaskMember<Control, 1024> receiver;
    freertos::TaskMember<Control, 1024> controller;
    freertos::TimerMember<Control> timer;
    freertos::Mutex lock;
    float ref_cfg[6] = {0, 0, 0, 0, 0, 0};

    static bool deserializeRefCfg(const uint8_t *data, const uint32_t data_size, float *cfg);
    static bool deserializeStop(const uint8_t *data, const uint32_t data_size);

    void receiverTask();
    void controllerTask();
    void shutdown();

public:
    Control();
};

Control control;

Control::Control()
    : receiver{"ctrl recv", this, &Control::receiverTask, 1},
      controller{"ctrl control", this, &Control::controllerTask, 1},
      timer{"ctrl recv timeout", this, &Control::shutdown, 1000, pdTRUE} {
}

void Control::receiverTask() {
    uint8_t data[1024];

    while(true) {
        const uint32_t size =
            xMessageBufferReceive(internals::comRxQueue(), data, sizeof(data), portMAX_DELAY);

        if(size > 0) {
            float cfg[6];
            if(deserializeRefCfg(data, size, cfg)) {
                xTimerReset(timer, portMAX_DELAY);

                if(xSemaphoreTake(lock, 100)) {
                    memcpy(ref_cfg, cfg, sizeof(cfg));
                    xSemaphoreGive(lock);
                }
            } else if(deserializeStop(data, size)) {
                shutdown();
            }
        }
    }
}

void Control::controllerTask() {
    uint32_t time = 0;
    float cfg[6];

    while(true) {
        if(xSemaphoreTake(lock, portMAX_DELAY)) {
            memcpy(cfg, ref_cfg, sizeof(cfg));
            xSemaphoreGive(lock);
        }

        Motors_SetVelocity(cfg[2], cfg[5]);
        Servos_SetGoal(cfg[0], cfg[1], cfg[3], cfg[4]);

        vTaskDelayUntil(&time, 10);
    }
}

void Control::shutdown() {
    if(xSemaphoreTake(lock, portMAX_DELAY)) {
        memset(ref_cfg, 0, sizeof(ref_cfg));
        xSemaphoreGive(lock);
    }
}

bool Control::deserializeRefCfg(const uint8_t *data, const uint32_t data_size, float *cfg) {
    cmp::MessagePack mpack = cmp::MessagePack::createFromData(data, data_size);

    cmp_ctx_t cmp;
    cmp_init(&cmp, &mpack, cmp::reader, NULL, NULL);

    uint32_t map_size = 0;
    if(!cmp_read_map(&cmp, &map_size)) {
        return false;
    }
    if(map_size != 2) {
        return false;
    }

    char key[32] = {0};
    uint32_t key_size = sizeof(key);
    if(!cmp_read_str(&cmp, key, &key_size)) {
        return false;
    }
    if(strncmp(key, "command", key_size) != 0) {
        return false;
    }

    char command[32] = {0};
    uint32_t command_size = sizeof(command);
    if(!cmp_read_str(&cmp, command, &command_size)) {
        return false;
    }
    if(strncmp(command, "manual", command_size) != 0) {
        return false;
    }

    key_size = sizeof(key);
    if(!cmp_read_str(&cmp, key, &key_size)) {
        return false;
    }
    if(strncmp(key, "ref_cfg", key_size) != 0) {
        return false;
    }

    uint32_t array_size = 0;
    if(!cmp_read_array(&cmp, &array_size)) {
        return false;
    }
    if(array_size != 6) {
        return false;
    }

    for(uint8_t i = 0; i < 6; i++) {
        if(!cmp_read_float(&cmp, &cfg[i])) {
            return false;
        }
    }

    return true;
}

bool Control::deserializeStop(const uint8_t *data, const uint32_t data_size) {
    cmp::MessagePack mpack = cmp::MessagePack::createFromData(data, data_size);

    cmp_ctx_t cmp;
    cmp_init(&cmp, &mpack, cmp::reader, NULL, NULL);

    uint32_t map_size = 0;
    if(!cmp_read_map(&cmp, &map_size)) {
        return false;
    }
    if(map_size != 1) {
        return false;
    }

    char key[32] = {0};
    uint32_t key_size = sizeof(key);
    if(!cmp_read_str(&cmp, key, &key_size)) {
        return false;
    }
    if(strncmp(key, "command", key_size) != 0) {
        return false;
    }

    char command[32] = {0};
    uint32_t command_size = sizeof(command);
    if(!cmp_read_str(&cmp, command, &command_size)) {
        return false;
    }
    if(strncmp(command, "stop", command_size) != 0) {
        return false;
    }

    return true;
}
