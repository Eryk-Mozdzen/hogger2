#pragma once

#include <stm32u5xx_hal.h>

#include "com/TelemetrySource.hpp"

class BLDC : public TelemetrySource {
public:
    enum Phase {
        U,
        V,
        W,
    };

    struct BEMF {
        uint16_t pin;
        GPIO_TypeDef *port;
    };

    struct Config {
        TIM_HandleTypeDef *commut_timer;
        TIM_HandleTypeDef *control_timer;
        uint32_t control_timer_itr;
        BEMF bemf[3];
    };

private:
    static constexpr Phase feedback_src_lookup[6] = {
        U, W, V, U, W, V,
    };

    static constexpr uint8_t feedback_dir_lookup[6] = {
        0, 1, 0, 1, 0, 1,
    };

    static constexpr uint8_t filter_lookup[256] = {
        0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3,
        4, 4, 5, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4,
        4, 5, 4, 5, 5, 6, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4,
        5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5,
        4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2,
        3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5,
        5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4,
        5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 3, 4, 4, 5, 4, 5, 5, 6,
        4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
    };

    enum State {
        IDLE,
        STARTUP_ALIGN1,
        STARTUP_ALIGN2,
        STARTUP_RAMP,
        RUNNING,
        PANIC,
    };

    struct PID {
        const float kp;
        const float ki;
        volatile float process;
        volatile float setpoint;
        volatile float value;
        volatile float error_integral;
        volatile float error_prev;
        volatile float dt;

        PID();
        void reset();
        void calculate();
    };

    const char *name;
    const Config config;
    PID pid;
    State state;
    uint32_t system_time;
    uint32_t state_start_time;
    uint32_t ramp_task;
    uint32_t vel_task;
    float switch_over;
    volatile float pulse;
    volatile uint8_t step;
    volatile uint8_t zc_filter;
    volatile uint8_t zc_occur;
    volatile uint32_t zc_count;
    float vel;
    float vel_setpoint;

    void configPWM(const uint32_t channel);
    void configOC(const uint32_t channel, const uint32_t mode);
    void shutdown();
    void moveTo(const State next);
    void serialize(cmp_ctx_t *cmp) const;
    const char *getName() const;

    static bool softwareTimer(uint32_t *prev, const uint32_t time, const uint32_t period);

public:
    BLDC(const char *name, const Config &config);

    void tick();

    void setVelocity(const float velocity);
    float getVelocity() const;

    void commutationISR(const TIM_HandleTypeDef *htim);
    void sampleISR(const TIM_HandleTypeDef *htim);
};
