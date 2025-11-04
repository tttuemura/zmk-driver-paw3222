/*
 * paw3222_scroll_accel.c
 *
 * Scroll acceleration and scroll-event processing for PAW3222 driver.
 *
 * Copy this file to zmk-driver-paw3222/src/paw3222_scroll_accel.c
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/input/input.h>
#include <math.h>
#include <stdlib.h> /* for abs() */
#include "../include/paw3222.h"

#ifdef CONFIG_PAW3222_SCROLL_ACCEL

void paw3222_apply_scroll_accel(int16_t x, int16_t y, struct paw32xx_data *data,
                                int32_t *accel_x, int32_t *accel_y)
{
    /* Default: passthrough */
    *accel_x = x;
    *accel_y = y;

    /* Safety: require data pointer */
    if (!data) {
        return;
    }

    int32_t movement = abs(x) + abs(y);
    int64_t current_time = k_uptime_get();
    int64_t delta_time = (data->last_scroll_time > 0) ? (current_time - data->last_scroll_time) : 0;

    /* Only compute acceleration when we have a recent prior sample */
    if (delta_time > 0 && delta_time < 100) {
        float speed = (float)movement / (float)delta_time; /* units: counts per ms */
        float base_sensitivity = (float)CONFIG_PAW3222_SCROLL_ACCEL_SENSITIVITY;
        /* Sigmoid-like mapping: adjust curve parameters here if needed */
        float acceleration = 1.0f + (base_sensitivity - 1.0f) *
                             (1.0f / (1.0f + expf(-0.2f * (speed - 10.0f))));

        *accel_x = (int32_t)((float)x * acceleration);
        *accel_y = (int32_t)((float)y * acceleration);

        /* Preserve small movements to avoid noise amplification */
        if (abs(x) <= 1) {
            *accel_x = x;
        }
        if (abs(y) <= 1) {
            *accel_y = y;
        }
    }

    data->last_scroll_time = current_time;
}

/* process_scroll_events: emit INPUT_REL_WHEEL or INPUT_REL_HWHEEL reports
 * This function is declared in the public header for optional external use.
 */
void paw3222_process_scroll_events(const struct device *dev, struct paw32xx_data *data,
                                   int32_t delta, bool is_horizontal)
{
    if (!dev || !data) {
        return;
    }

    if (abs(delta) <= CONFIG_PAW3222_SCROLL_TICK) {
        return;
    }

    int event_count = abs(delta) / CONFIG_PAW3222_SCROLL_TICK;
    const int MAX_EVENTS = 20;
    int32_t *target_delta = is_horizontal ? &data->scroll_delta_x : &data->scroll_delta_y;

    if (event_count > MAX_EVENTS) {
        event_count = MAX_EVENTS;
        *target_delta = (delta > 0) ?
            delta - (MAX_EVENTS * CONFIG_PAW3222_SCROLL_TICK) :
            delta + (MAX_EVENTS * CONFIG_PAW3222_SCROLL_TICK);
        data->last_remainder_time = k_uptime_get();
    } else {
        *target_delta = delta % CONFIG_PAW3222_SCROLL_TICK;
    }

    /* Determine report value polarity, honoring optional inversion Kconfig */
    int report_val = (delta > 0) ? 1 : -1;

#if defined(CONFIG_PAW3222_INVERT_SCROLL_X) || defined(CONFIG_PAW3222_INVERT_SCROLL_Y)
    if (is_horizontal) {
#  ifdef CONFIG_PAW3222_INVERT_SCROLL_X
        report_val = -report_val;
#  endif
    } else {
#  ifdef CONFIG_PAW3222_INVERT_SCROLL_Y
        report_val = -report_val;
#  endif
    }
#endif

    for (int i = 0; i < event_count; i++) {
        input_report_rel(dev,
                         is_horizontal ? INPUT_REL_HWHEEL : INPUT_REL_WHEEL,
                         report_val,
                         (i == event_count - 1),
                         K_MSEC(10));
    }

    if (is_horizontal) {
        data->scroll_delta_y = 0;
    } else {
        data->scroll_delta_x = 0;
    }
}

#endif /* CONFIG_PAW3222_SCROLL_ACCEL */
