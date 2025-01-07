/*
 * Copyright (c) 2025 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_STEPPER_STEPPER_STEP_DIR_H_
#define ZEPHYR_INCLUDE_DRIVERS_STEPPER_STEPPER_STEP_DIR_H_

#include <zephyr/drivers/stepper.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run the stepper in a given direction with given ticks between steps.
 *
 * @param dev Pointer to the device structure.
 * @param direction The direction of movement (positive or negative).
 * @param ticks The number of ticks to wait between steps.
 * @return 0 on success, or a negative error code on failure.
 */
int step_dir_stepper_run_ticks(const struct device* dev, enum stepper_direction, uint32_t ticks);

/**
 * @brief Set the minimum number of ticks (in context of the timing source) between steps.
 *
 * @param dev Pointer to the device structure.
 * @param ticks The minimum number of ticks between steps.
 * @return 0 on success, or a negative error code on failure.
 */
int step_dir_stepper_set_min_step_ticks(const struct device* dev, uint32_t ticks);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_STEPPER_STEPPER_STEP_DIR_H_ */
