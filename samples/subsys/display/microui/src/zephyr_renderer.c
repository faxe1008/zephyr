#include <zephyr/kernel.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/display.h>
#include <zephyr/input/input.h>
#include <string.h>
#include "renderer.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(microui_renderer, LOG_LEVEL_INF);

#define DISPLAY_NODE DT_CHOSEN(zephyr_display)
#define DISPLAY_WIDTH DT_PROP(DISPLAY_NODE, width)
#define DISPLAY_HEIGHT DT_PROP(DISPLAY_NODE, height)
#define DISPLAY_PIXEL_FORMAT PIXEL_FORMAT_RGB_888

#define DISPLAY_BUFFER_SIZE (4 * DISPLAY_WIDTH * DISPLAY_HEIGHT)

static const struct device *display_dev = DEVICE_DT_GET(DISPLAY_NODE);
static uint8_t display_buffer[DISPLAY_BUFFER_SIZE] __aligned(4);

// Clipping rectangle
static mu_Rect clip_rect = {0, 0, 0, 0};
static bool has_clip_rect = false;

static mu_Context mu_ctx;

// Font implementation
#include "font_generated.h"

typedef struct {
    uint32_t x;
    uint32_t y;
    bool pressed;
} pointer_input_data;

static pointer_input_data pointer_data = {0, 0};

static const FontGlyph* find_glyph(uint32_t unicode) {
    // Check if character is in our supported range
    if (unicode < FONT_FIRST_CHAR || unicode > FONT_LAST_CHAR) {
        return NULL;
    }
    return &font_data[unicode - FONT_FIRST_CHAR];
}

// Helper function to convert mu_Color to display-specific pixel format
static uint32_t color_to_pixel(mu_Color color) {
    switch (DISPLAY_PIXEL_FORMAT) {
        case PIXEL_FORMAT_RGB_888:
            return (color.r << 16) | (color.g << 8) | color.b;
            
        case PIXEL_FORMAT_ARGB_8888:
            return (color.a << 24) | (color.r << 16) | (color.g << 8) | color.b;
            
        case PIXEL_FORMAT_RGB_565:
            return ((color.r & 0xF8) << 8) | ((color.g & 0xFC) << 3) | (color.b >> 3);
            
        case PIXEL_FORMAT_BGR_565:
            return ((color.b & 0xF8) << 8) | ((color.g & 0xFC) << 3) | (color.r >> 3);
            
        case PIXEL_FORMAT_MONO01:
        case PIXEL_FORMAT_MONO10: {
            // Calculate luminance (0.299*R + 0.587*G + 0.114*B)
            uint8_t luma = (299 * color.r + 587 * color.g + 114 * color.b) / 1000;
            if (DISPLAY_PIXEL_FORMAT == PIXEL_FORMAT_MONO01) {
                return (luma > 127) ? 1 : 0;
            } else {
                return (luma > 127) ? 0 : 1;
            }
        }
        
        case PIXEL_FORMAT_L_8:
            // 8-bit grayscale
            return (299 * color.r + 587 * color.g + 114 * color.b) / 1000;
            
        default:
            return 0;
    }
    return 0;
}

// Set a pixel in the display buffer
static void set_pixel(int x, int y, mu_Color color) {
    // Check display bounds
    if (x < 0 || y < 0 || x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT) {
        return;
    }
    
    // Check clip rect
    if (has_clip_rect) {
        if (x < clip_rect.x || y < clip_rect.y ||
            x >= clip_rect.x + clip_rect.w ||
            y >= clip_rect.y + clip_rect.h) {
            return;
        }
    }
    
    uint32_t pixel = color_to_pixel(color);
    
    switch (DISPLAY_PIXEL_FORMAT) {
        case PIXEL_FORMAT_RGB_888: {
            int index = (y * DISPLAY_WIDTH + x) * 3;
            display_buffer[index + 0] = (pixel >> 16) & 0xFF; // R
            display_buffer[index + 1] = (pixel >> 8) & 0xFF;  // G
            display_buffer[index + 2] = pixel & 0xFF;         // B
            break;
        }
        
        case PIXEL_FORMAT_ARGB_8888: {
            int index = (y * DISPLAY_WIDTH + x) * 4;
            display_buffer[index + 0] = (pixel >> 24) & 0xFF; // A
            display_buffer[index + 1] = (pixel >> 16) & 0xFF; // R
            display_buffer[index + 2] = (pixel >> 8) & 0xFF;  // G
            display_buffer[index + 3] = pixel & 0xFF;         // B
            break;
        }
        
        case PIXEL_FORMAT_RGB_565:
        case PIXEL_FORMAT_BGR_565: {
            int index = (y * DISPLAY_WIDTH + x) * 2;
            display_buffer[index + 0] = (pixel >> 8) & 0xFF;
            display_buffer[index + 1] = pixel & 0xFF;
            break;
        }
        
        case PIXEL_FORMAT_MONO01:
        case PIXEL_FORMAT_MONO10: {
            int index = y * ((DISPLAY_WIDTH + 7) / 8) + (x / 8);
            uint8_t bit_pos = 7 - (x % 8);
            
            if (pixel) {
                display_buffer[index] |= (1 << bit_pos);
            } else {
                display_buffer[index] &= ~(1 << bit_pos);
            }
            break;
        }
        
        case PIXEL_FORMAT_L_8: {
            display_buffer[y * DISPLAY_WIDTH + x] = pixel & 0xFF;
            break;
        }
    }
}

static void draw_char(char c, int x, int y, mu_Color color) {
    const FontGlyph* glyph = find_glyph((uint32_t)c);
    if (!glyph) return;
    
    for (int row = 0; row < FONT_HEIGHT; row++) {
        // Handle different bitmap widths
        if (FONT_BITMAP_WIDTH <= 8) {
            uint8_t row_data = glyph->bitmap[row];
            for (int col = 0; col < glyph->width && col < FONT_BITMAP_WIDTH; col++) {
                if (row_data & (0x80 >> col)) {
                    set_pixel(x + col, y + row, color);
                }
            }
        } else if (FONT_BITMAP_WIDTH <= 16) {
            uint16_t row_data = (glyph->bitmap[row * 2] << 8) | glyph->bitmap[row * 2 + 1];
            for (int col = 0; col < glyph->width && col < FONT_BITMAP_WIDTH; col++) {
                if (row_data & (0x8000 >> col)) {
                    set_pixel(x + col, y + row, color);
                }
            }
        } else { // FONT_BITMAP_WIDTH <= 32
            uint32_t row_data = (glyph->bitmap[row * 4] << 24) | 
                               (glyph->bitmap[row * 4 + 1] << 16) |
                               (glyph->bitmap[row * 4 + 2] << 8) | 
                               glyph->bitmap[row * 4 + 3];
            for (int col = 0; col < glyph->width && col < FONT_BITMAP_WIDTH; col++) {
                if (row_data & (0x80000000 >> col)) {
                    set_pixel(x + col, y + row, color);
                }
            }
        }
    }
}


static void draw_icon_pattern(int id, mu_Rect rect, mu_Color color) {
   
}

void r_init(void) {
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device is not ready");
        return;
    }

    display_blanking_off(display_dev);
    
    clip_rect.x = 0;
    clip_rect.y = 0;
    clip_rect.w = DISPLAY_WIDTH;
    clip_rect.h = DISPLAY_HEIGHT;
    has_clip_rect = false;
    
    memset(display_buffer, 0, DISPLAY_BUFFER_SIZE);
    
    LOG_INF("MicroUI renderer initialized for %dx%d display", DISPLAY_WIDTH, DISPLAY_HEIGHT);
}

void r_draw_rect(mu_Rect rect, mu_Color color) {
    // Draw rectangle (filled)
    for (int y = rect.y; y < rect.y + rect.h; y++) {
        for (int x = rect.x; x < rect.x + rect.w; x++) {
            set_pixel(x, y, color);
        }
    }
}

void r_draw_text(const char *text, mu_Vec2 pos, mu_Color color) {
    int x = pos.x;
    
    while (*text) {
        const FontGlyph* glyph = find_glyph((uint32_t)*text);
        if (glyph) {
            draw_char(*text, x, pos.y, color);
            x += glyph->width + FONT_CHAR_SPACING;  // Use actual character width
        } else {
            x += FONT_DEFAULT_WIDTH + FONT_CHAR_SPACING;  // Fallback width
        }
        text++;
    }
}

void r_draw_icon(int id, mu_Rect rect, mu_Color color) {

}

 int r_get_text_width(const char *text, int len){
    int width = 0;
    int char_count = 0;
    
    while (char_count < len) {
        const FontGlyph* glyph = find_glyph((uint32_t)*text);
        if (glyph) {
            width += glyph->width;
        } else {
            width += FONT_DEFAULT_WIDTH;
        }
        char_count++;
        text++;
    }
    
    // Add spacing between characters (but not after the last one)
    if (char_count > 0) {
        width += (char_count - 1) * FONT_CHAR_SPACING;
    }
    
    return width;
}

int r_get_text_height(void) {
    return FONT_HEIGHT;
}

void r_set_clip_rect(mu_Rect rect) {
    clip_rect = rect;
    has_clip_rect = true;
    
    if (clip_rect.x < 0) {
        clip_rect.w += clip_rect.x;
        clip_rect.x = 0;
    }
    
    if (clip_rect.y < 0) {
        clip_rect.h += clip_rect.y;
        clip_rect.y = 0;
    }
    
    if (clip_rect.x + clip_rect.w > DISPLAY_WIDTH) {
        clip_rect.w = DISPLAY_WIDTH - clip_rect.x;
    }
    
    if (clip_rect.y + clip_rect.h > DISPLAY_HEIGHT) {
        clip_rect.h = DISPLAY_HEIGHT - clip_rect.y;
    }
}

void r_clear(mu_Color color) {
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        for (int x = 0; x < DISPLAY_WIDTH; x++) {
            set_pixel(x, y, color);
        }
    }
}

void r_present(void) {
    static struct display_buffer_descriptor desc = {
        .buf_size = DISPLAY_BUFFER_SIZE,
        .width = DISPLAY_WIDTH,
        .height = DISPLAY_HEIGHT,
        .pitch = DISPLAY_WIDTH,
        .frame_incomplete = false,
    };
    display_write(display_dev, 0, 0, &desc, display_buffer);
}

mu_Context *r_get_context(void) {
    return &mu_ctx;
}

static void microui_input_handler(struct input_event *evt, void *user_data){
    switch(evt->code){
        case INPUT_ABS_X:
            pointer_data.x = evt->value;
            break;
        case INPUT_ABS_Y:
            pointer_data.y = evt->value;
            break;
        case INPUT_BTN_TOUCH:
            if(evt->value) {
                pointer_data.pressed = true;
            } else {
                pointer_data.pressed = false;
            }
            break;
    }

    if(!evt->sync){
        return;
    }

    if(pointer_data.pressed) {
        mu_input_mousedown(&mu_ctx, pointer_data.x, pointer_data.y, MU_MOUSE_LEFT);
    } else {
        mu_input_mouseup(&mu_ctx, pointer_data.x, pointer_data.y, MU_MOUSE_LEFT);
    }
    mu_input_mousemove(&mu_ctx, pointer_data.x, pointer_data.y);
}

INPUT_CALLBACK_DEFINE(NULL, microui_input_handler, NULL);
