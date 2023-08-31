/*
 * Copyright 2023 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lv_os.h"

typedef void (*lv_thread_entry)(void *);

static void thread_entry(void *thread, void *cb, void *user_data);

lv_res_t lv_thread_init(lv_thread_t *thread, lv_thread_prio_t prio, void (*callback)(void *),
			size_t stack_size, void *user_data)
{
	int thread_priority;

	thread->stack = k_thread_stack_alloc(stack_size, 0);
	if (thread->stack == NULL) {
		return LV_RES_INV;
	}

	thread_priority = (CONFIG_NUM_PREEMPT_PRIORITIES - 1) -
			  ((prio * (CONFIG_NUM_PREEMPT_PRIORITIES - 1)) / LV_THREAD_PRIO_HIGHEST);

	thread->tid = k_thread_create(&thread->thread, thread->stack, stack_size, thread_entry,
				      thread, callback, user_data, thread_priority, 0, K_NO_WAIT);

	return LV_RES_OK;
}

lv_res_t lv_thread_delete(lv_thread_t *thread)
{
	lv_res_t ret = LV_RES_OK;

	k_thread_abort(thread->tid);

	if (k_thread_stack_free(thread->stack) < 0) {
		ret = LV_RES_INV;
	}

	return ret;
}

lv_res_t lv_mutex_init(lv_mutex_t *mutex)
{
	int ret = k_mutex_init(mutex);

	if (ret) {
		LV_LOG_WARN("Error: %d", ret);
		return LV_RES_INV;
	} else {
		return LV_RES_OK;
	}
}

lv_res_t lv_mutex_lock(lv_mutex_t *mutex)
{
	int ret = k_mutex_lock(mutex, K_FOREVER);

	if (ret) {
		LV_LOG_WARN("Error: %d", ret);
		return LV_RES_INV;
	} else {
		return LV_RES_OK;
	}
}

lv_res_t lv_mutex_lock_isr(lv_mutex_t *mutex)
{
	int ret = k_mutex_lock(mutex, K_NO_WAIT);

	if (ret) {
		LV_LOG_WARN("Error: %d", ret);
		return LV_RES_INV;
	} else {
		return LV_RES_OK;
	}
}

lv_res_t lv_mutex_unlock(lv_mutex_t *mutex)
{
	int ret = k_mutex_unlock(mutex);

	if (ret) {
		LV_LOG_WARN("Error: %d", ret);
		return LV_RES_INV;
	} else {
		return LV_RES_OK;
	}
}

lv_res_t lv_mutex_delete(lv_mutex_t *mutex)
{
	LV_UNUSED(mutex);
	return LV_RES_OK;
}

lv_res_t lv_thread_sync_init(lv_thread_sync_t *sync)
{
	k_mutex_init(&sync->mutex);
	k_condvar_init(&sync->cond);
	sync->v = false;
	return LV_RES_OK;
}

lv_res_t lv_thread_sync_wait(lv_thread_sync_t *sync)
{
	k_mutex_lock(&sync->mutex, K_FOREVER);
	while (!sync->v) {
		k_condvar_wait(&sync->cond, &sync->mutex, K_FOREVER);
	}
	sync->v = false;
	k_mutex_unlock(&sync->mutex);
	return LV_RES_OK;
}

lv_res_t lv_thread_sync_signal(lv_thread_sync_t *sync)
{
	k_mutex_lock(&sync->mutex, K_FOREVER);
	sync->v = true;
	k_condvar_signal(&sync->cond);
	k_mutex_unlock(&sync->mutex);

	return LV_RES_OK;
}

lv_res_t lv_thread_sync_delete(lv_thread_sync_t *sync)
{
	LV_UNUSED(sync);
	return LV_RES_OK;
}

void thread_entry(void *thread, void *cb, void *user_data)
{
	lv_thread_entry entry_cb = (lv_thread_entry)cb;

	entry_cb(user_data);
	lv_thread_delete((lv_thread_t *)thread);
}
