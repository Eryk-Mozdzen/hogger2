#include <cmath>
#include <cstring>

#include "actuate/Motors.hpp"
#include "actuate/Servos.hpp"
#include "com/Mailbox.hpp"
#include "freertos/Mutex.hpp"
#include "freertos/Task.hpp"
#include "freertos/Timer.hpp"
#include "utils/cmp.hpp"

class Control {
    Mailbox<10 * 1024> mailbox;
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
    : mailbox{2},
      receiver{"ctrl recv", this, &Control::receiverTask, 10},
      controller{"ctrl control", this, &Control::controllerTask, 10},
      timer{"ctrl recv timeout", this, &Control::shutdown, 1000, pdTRUE} {
}

void Control::receiverTask() {
    uint8_t data[1024];

    while(true) {
        const uint32_t size =
            xMessageBufferReceive(mailbox.getBuffer(), data, sizeof(data), portMAX_DELAY);

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

void Control::controllerTask() {
    uint32_t time = 0;
    float cfg[6];

    while(true) {
        if(xSemaphoreTake(lock, portMAX_DELAY)) {
            memcpy(cfg, ref_cfg, sizeof(cfg));
            xSemaphoreGive(lock);
        }

        motor::setVelocity(cfg[2], cfg[5]);
        servo::setGoal(cfg[0], cfg[1], cfg[3], cfg[4]);

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

    uint32_t map_size = 0;
    if(!cmp_read_map(&mpack.ctx, &map_size)) {
        return false;
    }
    if(map_size != 2) {
        return false;
    }

    char key[32] = {0};
    uint32_t key_size = sizeof(key);
    if(!cmp_read_str(&mpack.ctx, key, &key_size)) {
        return false;
    }
    if(strncmp(key, "command", key_size) != 0) {
        return false;
    }

    char command[32] = {0};
    uint32_t command_size = sizeof(command);
    if(!cmp_read_str(&mpack.ctx, command, &command_size)) {
        return false;
    }
    if(strncmp(command, "manual", command_size) != 0) {
        return false;
    }

    key_size = sizeof(key);
    if(!cmp_read_str(&mpack.ctx, key, &key_size)) {
        return false;
    }
    if(strncmp(key, "ref_cfg", key_size) != 0) {
        return false;
    }

    uint32_t array_size = 0;
    if(!cmp_read_array(&mpack.ctx, &array_size)) {
        return false;
    }
    if(array_size != 6) {
        return false;
    }

    for(uint8_t i = 0; i < 6; i++) {
        if(!cmp_read_float(&mpack.ctx, &cfg[i])) {
            return false;
        }
    }

    return true;
}

bool Control::deserializeStop(const uint8_t *data, const uint32_t data_size) {
    cmp::MessagePack mpack = cmp::MessagePack::createFromData(data, data_size);

    uint32_t map_size = 0;
    if(!cmp_read_map(&mpack.ctx, &map_size)) {
        return false;
    }
    if(map_size != 1) {
        return false;
    }

    char key[32] = {0};
    uint32_t key_size = sizeof(key);
    if(!cmp_read_str(&mpack.ctx, key, &key_size)) {
        return false;
    }
    if(strncmp(key, "command", key_size) != 0) {
        return false;
    }

    char command[32] = {0};
    uint32_t command_size = sizeof(command);
    if(!cmp_read_str(&mpack.ctx, command, &command_size)) {
        return false;
    }
    if(strncmp(command, "stop", command_size) != 0) {
        return false;
    }

    return true;
}
