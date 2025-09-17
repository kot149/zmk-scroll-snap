/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_scroll_snap

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <drivers/input_processor.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include <string.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct scroll_snap_sample {
    int32_t dx;
    int32_t dy;
};

#define DIRECTION_NONE 0
#define DIRECTION_X 1
#define DIRECTION_Y 2
#define DIRECTION_DIAG_PLUS 3
#define DIRECTION_DIAG_MINUS 4

struct input_processor_scroll_snap_data {
    uint16_t head;
    struct scroll_snap_sample samples[CONFIG_ZMK_SCROLL_SNAP_MAX_BUF_SIZE];
    uint16_t sample_count;
    struct scroll_snap_sample sample_sum;

    struct scroll_snap_sample remainder;

    int64_t last_event_ts_ms;
    uint8_t lock_direction;
    uint16_t lock_events_remaining;
    int64_t lock_expires_at_ms;
};

struct input_processor_scroll_snap_config {
    uint32_t x_thresh_num;
    uint32_t x_thresh_den;
    uint32_t y_thresh_num;
    uint32_t y_thresh_den;
    uint32_t xy_thresh_num;
    uint32_t xy_thresh_den;

    uint16_t require_n_samples;
    uint32_t immediate_snap_threshold;
    uint32_t lock_duration_ms;
    uint16_t lock_for_next_n_events;
    uint32_t idle_reset_timeout_ms;

    uint8_t event_type;
    uint16_t event_code_x;
    uint16_t event_code_y;
    bool track_remainders;
};

static int input_processor_scroll_snap_init(const struct device *dev);

static int input_processor_scroll_snap_handle_event(const struct device *dev,
                                                      struct input_event *event,
                                                      uint32_t param1, uint32_t param2,
                                                      struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    struct input_processor_scroll_snap_data *data = dev->data;
    const struct input_processor_scroll_snap_config *config = dev->config;

    // Check if event type matches configured type
    if (event->type != config->event_type) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // Check if event code matches configured codes and determine axis
    bool is_x_axis = (event->code == config->event_code_x);
    bool is_y_axis = (event->code == config->event_code_y);

    if (!is_x_axis && !is_y_axis) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // Reset after inactivity timeout
    int64_t now_ms = k_uptime_get();
    if (config->idle_reset_timeout_ms > 0) {
        int64_t elapsed = now_ms - data->last_event_ts_ms;
        if (elapsed >= config->idle_reset_timeout_ms) {
            input_processor_scroll_snap_init(dev);
        }
    }

    data->last_event_ts_ms = now_ms;

    // Expire time-based lock
    if (data->lock_direction != DIRECTION_NONE && config->lock_duration_ms > 0 && data->lock_expires_at_ms > 0) {
        if (now_ms >= data->lock_expires_at_ms) {
            data->lock_direction = DIRECTION_NONE;
            data->lock_expires_at_ms = 0;
            data->lock_events_remaining = 0;
        }
    }

    // Accumulate samples using ring buffer
    struct scroll_snap_sample incoming = { .dx = 0, .dy = 0 };
    if (is_x_axis) {
        incoming.dx = event->value;
    } else if (is_y_axis) {
        incoming.dy = event->value;
    }

    // When buffer is full, delete the oldest sample
    if (data->sample_count >= config->require_n_samples) {
        struct scroll_snap_sample old = data->samples[data->head];
        data->sample_sum.dx -= abs(old.dx);
        data->sample_sum.dy -= abs(old.dy);
    }

    data->samples[data->head] = incoming;
    data->sample_sum.dx += abs(incoming.dx);
    data->sample_sum.dy += abs(incoming.dy);
    data->remainder.dx += incoming.dx;
    data->remainder.dy += incoming.dy;
    data->sample_count++;
    data->head = (data->head + 1) % config->require_n_samples;

    uint16_t abs_x = (uint16_t)(data->sample_sum.dx);
    uint16_t abs_y = (uint16_t)(data->sample_sum.dy);

    // Check if we have enough samples
    if (!(data->sample_count >= config->require_n_samples || abs_x > config->immediate_snap_threshold || abs_y > config->immediate_snap_threshold)) {
        event->value = 0;
        event->sync = false;
        return ZMK_INPUT_PROC_STOP;
    }

    int32_t new_x = 0, new_y = 0;
    uint8_t detected_direction = DIRECTION_NONE;

    // Detect direction from thresholds
    if (abs_y * config->y_thresh_den > abs_x * config->y_thresh_num) {
        detected_direction = DIRECTION_Y;
    } else if (abs_y * config->x_thresh_den < abs_x * config->x_thresh_num) {
        detected_direction = DIRECTION_X;
    } else if (abs_x * config->xy_thresh_num < abs_y * config->xy_thresh_den &&
               abs_y * config->xy_thresh_num < abs_x * config->xy_thresh_den) {
        detected_direction = ((data->remainder.dx > 0) == (data->remainder.dy > 0)) ? DIRECTION_DIAG_PLUS : DIRECTION_DIAG_MINUS;
    } else {
        detected_direction = DIRECTION_NONE;
    }

    // Check if lock is active
    bool is_lock_active = false;
    if (config->lock_duration_ms > 0) {
        is_lock_active = (data->lock_direction != DIRECTION_NONE) && (data->lock_expires_at_ms > now_ms);
    }
    is_lock_active |= (data->lock_events_remaining > 0);

    // Snap to the decided direction
    uint8_t decided_direction = is_lock_active ? data->lock_direction : detected_direction;
    switch (decided_direction) {
        case DIRECTION_X:
            LOG_DBG("Snapping to X axis");
            new_x = data->remainder.dx;
            new_y = 0;
            data->remainder.dy = 0;
            break;
        case DIRECTION_Y:
            LOG_DBG("Snapping to Y axis");
            new_y = data->remainder.dy;
            new_x = 0;
            data->remainder.dx = 0;
            break;
        case DIRECTION_DIAG_PLUS:
            LOG_DBG("Snapping to Diagonal (+)");
            // TODO !!
            break;
        case DIRECTION_DIAG_MINUS:
            LOG_DBG("Snapping to Diagonal (-)");
            // TODO !!
            break;
        default:
            new_x = 0;
            new_y = 0;
            break;
    }

    // Modify the current event to be the snapped scroll version and clear remainders
    if (is_y_axis) {
        event->value = new_y;
        data->remainder.dy = 0;
    } else if (is_x_axis) {
        event->value = new_x;
        data->remainder.dx = 0;
    }

    // Lock handling: start/refresh/decrement
    if (config->lock_duration_ms > 0 || config->lock_for_next_n_events > 0) {
        if (is_lock_active) {
            // Refresh when detected direction matches current lock
            if (detected_direction != DIRECTION_NONE && detected_direction == data->lock_direction) {
                if (config->lock_duration_ms > 0) {
                    data->lock_expires_at_ms = now_ms + (int64_t)config->lock_duration_ms;
                }
                if (config->lock_for_next_n_events > 0) {
                    data->lock_events_remaining = config->lock_for_next_n_events;
                }
            } else {
                // No refresh: decrement event-based lock
                if (config->lock_duration_ms == 0 && config->lock_for_next_n_events > 0) {
                    if (data->lock_events_remaining > 0) {
                        data->lock_events_remaining--;
                        if (data->lock_events_remaining == 0) {
                            data->lock_direction = DIRECTION_NONE;
                        }
                    }
                }
            }
        } else if (decided_direction != DIRECTION_NONE) {
            // Start a new lock
            if (config->lock_duration_ms > 0) {
                data->lock_direction = decided_direction;
                data->lock_expires_at_ms = now_ms + (int64_t)config->lock_duration_ms;
                data->lock_events_remaining = 0;
            }
            if (config->lock_for_next_n_events > 0) {
                data->lock_direction = decided_direction;
                data->lock_events_remaining = config->lock_for_next_n_events;
            }
        } else {
            // No locking configured or no decision
            if (config->lock_duration_ms == 0 && config->lock_for_next_n_events == 0) {
                data->lock_direction = DIRECTION_NONE;
                data->lock_events_remaining = 0;
                data->lock_expires_at_ms = 0;
            }
        }
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static int input_processor_scroll_snap_init(const struct device *dev) {
    struct input_processor_scroll_snap_data *data = dev->data;
    const struct input_processor_scroll_snap_config *config = dev->config;

    data->sample_count = 0;
    data->sample_sum.dx = 0;
    data->sample_sum.dy = 0;
    data->remainder.dx = 0;
    data->remainder.dy = 0;
    data->head = 0;
    data->last_event_ts_ms = k_uptime_get();
    data->lock_events_remaining = 0;
    data->lock_direction = DIRECTION_NONE;
    data->lock_expires_at_ms = 0;

    memset(data->samples, 0, sizeof(struct scroll_snap_sample) * config->require_n_samples);

    return 0;
}

static const struct zmk_input_processor_driver_api input_processor_scroll_snap_driver_api = {
    .handle_event = input_processor_scroll_snap_handle_event,
};

#define SCROLL_SNAP_INPUT_PROCESSOR_INST(n)                                                             \
    static struct input_processor_scroll_snap_data input_processor_scroll_snap_data_##n = {};           \
    static const struct input_processor_scroll_snap_config input_processor_scroll_snap_config_##n = {   \
        .x_thresh_num = DT_INST_PROP_BY_IDX(n, x_threshold, 0),                                         \
        .x_thresh_den = DT_INST_PROP_BY_IDX(n, x_threshold, 1),                                         \
        .y_thresh_num = DT_INST_PROP_BY_IDX(n, y_threshold, 0),                                         \
        .y_thresh_den = DT_INST_PROP_BY_IDX(n, y_threshold, 1),                                         \
        .xy_thresh_num = DT_INST_PROP_BY_IDX(n, xy_threshold, 0),                                       \
        .xy_thresh_den = DT_INST_PROP_BY_IDX(n, xy_threshold, 1),                                       \
        .immediate_snap_threshold = DT_INST_PROP(n, immediate_snap_threshold),                          \
        .require_n_samples = CLAMP(DT_INST_PROP_OR(n, require_n_samples, 0), 1, CONFIG_ZMK_SCROLL_SNAP_MAX_BUF_SIZE), \
        .idle_reset_timeout_ms = DT_INST_PROP_OR(n, idle_reset_timeout_ms, 0),                          \
        .lock_duration_ms = DT_INST_PROP_OR(n, lock_duration_ms, 0),                                    \
        .lock_for_next_n_events = DT_INST_PROP_OR(n, lock_for_next_n_events, 0),                        \
        .event_type = DT_INST_PROP_OR(n, event_type, INPUT_EV_REL),                                     \
        .event_code_x = DT_INST_PROP_OR(n, event_code_x, INPUT_REL_HWHEEL),                             \
        .event_code_y = DT_INST_PROP_OR(n, event_code_y, INPUT_REL_WHEEL),                              \
        .track_remainders = DT_INST_PROP_OR(n, track_remainders, false),                                \
    };                                                                                                  \
    DEVICE_DT_INST_DEFINE(n, input_processor_scroll_snap_init, NULL,                                    \
                          &input_processor_scroll_snap_data_##n,                                        \
                          &input_processor_scroll_snap_config_##n, POST_KERNEL,                         \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                          \
                          &input_processor_scroll_snap_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_SNAP_INPUT_PROCESSOR_INST)

#endif
