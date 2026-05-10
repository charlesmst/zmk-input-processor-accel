/*
 * Copyright (c) 2024 Alice Zhang
 * SPDX-License-Identifier: MIT
 *
 * maccel-style mouse acceleration for ZMK input processors.
 * Algorithm: f(v) = upper - (upper - m) / (1 + exp(k*(v-s)))^(g/k)
 * Same curve as burkfers/Wimads maccel, ported from zmk-pmw3610-driver.
 * Requires declaring the sensor CPI so velocity can be normalized.
 */

#define DT_DRV_COMPAT zmk_input_processor_sigmoid_accel

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>
#include "math.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct maccel_config {
    int32_t cpi;
    int32_t takeoff;     /* k * 100 */
    int32_t growth_rate; /* g * 100 */
    int32_t offset;      /* s * 100 */
    int32_t limit;       /* lower factor * 100 */
    int32_t limit_upper; /* upper factor * 100 */
};

struct group_state {
    int64_t last_time_us;
    float velocity;
    float carry_primary;
    float carry_secondary;
    float last_factor;
    bool initialized;
};

struct maccel_state {
    struct group_state pointer;
    struct group_state scroll;
};

static float compute_factor(const struct maccel_config *cfg, float v) {
    float k = cfg->takeoff / 100.0f;
    float g = cfg->growth_rate / 100.0f;
    float s = cfg->offset / 100.0f;
    float m = cfg->limit / 100.0f;
    float upper = cfg->limit_upper / 100.0f;
    return upper - (upper - m) / powf(1.0f + expf(k * (v - s)), g / k);
}

static int maccel_handle_event(const struct device *dev, struct input_event *event,
                               uint32_t param1, uint32_t param2,
                               struct zmk_input_processor_state *state) {
    const struct maccel_config *cfg = dev->config;
    struct maccel_state *data = dev->data;

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    struct group_state *group;
    float *carry;

    switch (event->code) {
    case INPUT_REL_X:
        group = &data->pointer;
        carry = &group->carry_primary;
        break;
    case INPUT_REL_Y:
        group = &data->pointer;
        carry = &group->carry_secondary;
        break;
    case INPUT_REL_WHEEL:
        group = &data->scroll;
        carry = &group->carry_primary;
        break;
    case INPUT_REL_HWHEEL:
        group = &data->scroll;
        carry = &group->carry_secondary;
        break;
    default:
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    int64_t dt_us = now_us - group->last_time_us;

    if (!group->initialized || dt_us > 200000) {
        group->velocity = 0.0f;
        group->carry_primary = 0.0f;
        group->carry_secondary = 0.0f;
        group->last_factor = cfg->limit / 100.0f;
        group->initialized = true;
        group->last_time_us = now_us;
        dt_us = 8000;
    }

    float factor;
    if (dt_us > 500) {
        /* New polling cycle: compute instant velocity (matches pmw3610 maccel) */
        float dt_ms = (float)dt_us / 1000.0f;
        group->velocity = fabsf((float)event->value) * 1000.0f / ((float)cfg->cpi * dt_ms);
        group->last_time_us = now_us;
        factor = compute_factor(cfg, group->velocity);
        group->last_factor = factor;
    } else {
        /* Second axis in same poll burst (e.g. Y right after X): reuse factor */
        factor = group->last_factor;
    }

    float scaled = (float)event->value * factor + *carry;
    int32_t output = (int32_t)scaled;
    *carry = scaled - (float)output;

    LOG_DBG("maccel v=%.3f factor=%.3f in=%d out=%d",
            (double)group->velocity, (double)factor, event->value, output);

    event->value = output;
    return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api maccel_driver_api = {
    .handle_event = maccel_handle_event,
};

#define MACCEL_INST(n)                                                      \
    static struct maccel_state maccel_state_##n = {};                       \
    static const struct maccel_config maccel_config_##n = {                 \
        .cpi         = DT_INST_PROP(n, cpi),                                \
        .takeoff     = DT_INST_PROP(n, takeoff),                            \
        .growth_rate = DT_INST_PROP(n, growth_rate),                        \
        .offset      = DT_INST_PROP(n, offset),                             \
        .limit       = DT_INST_PROP(n, limit),                              \
        .limit_upper = DT_INST_PROP(n, limit_upper),                        \
    };                                                                      \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &maccel_state_##n,                 \
                          &maccel_config_##n, POST_KERNEL,                  \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,              \
                          &maccel_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MACCEL_INST)
