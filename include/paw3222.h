/*
 * Copyright 2024 Google LLC
 * Modifications Copyright 2025 sekigon-gonnoc
 * Original source code: https://github.com/zephyrproject-rtos/zephyr/blob/19c6240b6865bcb28e1d786d4dcadfb3a02067a0/include/zephyr/input/input_paw32xx.h
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_INPUT_PAW32XX_H_
#define ZEPHYR_INCLUDE_INPUT_PAW32XX_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/device.h>

/**
 * @brief Set resolution on a paw32xx device
 *
 * @param dev paw32xx device.
 * @param res_cpi CPI resolution, 200 to 3200.
 */
int paw32xx_set_resolution(const struct device *dev, uint16_t res_cpi);

/**
 * @brief Set force awake mode on a paw32xx device
 *
 * @param dev paw32xx device.
 * @param enable whether to enable or disable force awake mode.
 */
int paw32xx_force_awake(const struct device *dev, bool enable);

#ifdef CONFIG_PAW3222_SCROLL_ACCEL

/* Forward-declare runtime data structure used by driver */
struct paw32xx_data;

/**
 * Apply scroll acceleration to raw x/y deltas
 *
 * @param x raw X delta
 * @param y raw Y delta
 * @param data runtime data pointer
 * @param accel_x out accelerated X
 * @param accel_y out accelerated Y
 */
void paw3222_apply_scroll_accel(int16_t x, int16_t y, struct paw32xx_data *data,
                                int32_t *accel_x, int32_t *accel_y);

/**
 * Process scroll events and emit wheel/hwheel reports.
 *
 * @param dev Zephyr device pointer
 * @param data runtime data pointer
 * @param delta accumulated scroll delta
 * @param is_horizontal true => horizontal (hwheel) else vertical (wheel)
 */
void paw3222_process_scroll_events(const struct device *dev, struct paw32xx_data *data,
                                   int32_t delta, bool is_horizontal);

#endif /* CONFIG_PAW3222_SCROLL_ACCEL */

#endif /* ZEPHYR_INCLUDE_INPUT_PAW32XX_H_ */
