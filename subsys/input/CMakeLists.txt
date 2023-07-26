# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(input.c)
zephyr_library_sources(input_utils.c)

zephyr_library_sources_ifdef(CONFIG_INPUT_LONGPRESS input_longpress.c)

if(CONFIG_INPUT_LVGL)

zephyr_library_sources(input_lvgl_common.c)

zephyr_library_sources_ifdef(CONFIG_INPUT_LVGL_POINTER input_lvgl_pointer.c)

endif()
