/*
 * Copyright (c) 2018 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <lvgl_input_device.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

#define IMG_FILE_PATH "/mnt/img.bin"

#define LVGL_PARTITION		storage_partition
#define LVGL_PARTITION_ID	FIXED_PARTITION_ID(LVGL_PARTITION)

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(cstorage);

static struct fs_mount_t mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &cstorage,
	.storage_dev = (void *)LVGL_PARTITION_ID,
	.mnt_point = "/mnt"
};


int setup_fs(void)
{
	struct fs_file_t img;
	struct fs_dirent info;
	int ret;
	const lv_img_dsc_t *c_img = get_lvgl_img();

	ret = fs_mount(&mnt);
	if (ret < 0) {
		return ret;
	}

	ret = fs_stat(IMG_FILE_PATH, &info);
	if ((ret == 0) && (info.type == FS_DIR_ENTRY_FILE)) {
		return ret;
	}

	fs_file_t_init(&img);
	ret = fs_open(&img, IMG_FILE_PATH, FS_O_CREATE | FS_O_WRITE);
	if (ret < 0) {
		return ret;
	}

	ret = fs_write(&img, &c_img->header, sizeof(lv_img_header_t));
	if (ret < 0) {
		return ret;
	}

	ret = fs_write(&img, c_img->data, c_img->data_size);
	if (ret < 0) {
		return ret;
	}

	ret = fs_close(&img);
	if (ret < 0) {
		return ret;
	}
	return ret;
}

int main(void)
{
	int ret;
	const struct device *display_dev;
	lv_obj_t *screen;
	lv_obj_t *img;

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device not ready, aborting test");
		return 0;
	}

	lv_task_handler();
	display_blanking_off(display_dev);

	ret = setup_fs();
	if(ret != 0){
		LOG_ERR("Error setting up FS: %d", ret);
	}

	screen = lv_obj_create(NULL);

	img = lv_img_create(lv_scr_act());
	lv_img_set_src(img, IMG_FILE_PATH);
	lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);

	while (1) {
		lv_task_handler();
		k_sleep(K_MSEC(10));
	}
}
