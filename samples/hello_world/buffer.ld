#include <zephyr/linker/linker-defs.h>
#include <zephyr/linker/linker-tool.h>
#include <zephyr/linker/sections.h>

SECTIONS
{
    .data : ALIGN(4) {
        /* Keep the existing .data content here */

        /* Add the custom section inside .data */
        . = ALIGN(4);
        __vl53l5cx_buf_start = .;
        KEEP(*(SORT(.vl53l5cx_buf*)));
        __vl53l5cx_buf_end = .;

        /* Other .data contents follow here */
    } > SRAM
}
