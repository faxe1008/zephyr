/*
 * Copyright 2023 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULES_LVGL_LV_ZEPHYR_RTOS_H_
#define ZEPHYR_MODULES_LVGL_LV_ZEPHYR_RTOS_H_

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

BUILD_ASSERT(CONFIG_DYNAMIC_THREAD, "Dynamic thread creation must be enabled");

typedef struct {
	k_tid_t tid;
	k_thread_stack_t *stack;
	struct k_thread thread;
} lv_thread_t;

typedef struct k_mutex lv_mutex_t;

typedef struct {
	struct k_mutex mutex;
	struct k_condvar cond;
	bool v;
} lv_thread_sync_t;

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*ZEPHYR_MODULES_LVGL_LV_ZEPHYR_RTOS_H_*/
