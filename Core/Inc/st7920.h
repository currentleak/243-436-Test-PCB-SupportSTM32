/* st7920.h - simple 8-bit parallel ST7920 driver (HAL) */
#ifndef __ST7920_H
#define __ST7920_H

#include "main.h"
#include <stdint.h>

// Pin mapping: prefer CubeMX-generated macros from main.h, otherwise fall back
// Control pins
#ifndef ST7920_WR_Pin
#define ST7920_WR_GPIO_Port GPIOD
#define ST7920_WR_Pin GPIO_PIN_10
#endif

#ifndef ST7920_RS_Pin
#define ST7920_RS_GPIO_Port GPIOD
#define ST7920_RS_Pin GPIO_PIN_11
#endif

#ifndef ST7920_E_Pin
#define ST7920_E_GPIO_Port GPIOB
#define ST7920_E_Pin GPIO_PIN_8
#endif

// Data bus on PE8..PE15 => D0..D7
#ifndef ST7920_D0_Pin
#define ST7920_D0_GPIO_Port GPIOE
#define ST7920_D0_Pin GPIO_PIN_8
#endif
#ifndef ST7920_D1_Pin
#define ST7920_D1_GPIO_Port GPIOE
#define ST7920_D1_Pin GPIO_PIN_9
#endif
#ifndef ST7920_D2_Pin
#define ST7920_D2_GPIO_Port GPIOE
#define ST7920_D2_Pin GPIO_PIN_10
#endif
#ifndef ST7920_D3_Pin
#define ST7920_D3_GPIO_Port GPIOE
#define ST7920_D3_Pin GPIO_PIN_11
#endif
#ifndef ST7920_D4_Pin
#define ST7920_D4_GPIO_Port GPIOE
#define ST7920_D4_Pin GPIO_PIN_12
#endif
#ifndef ST7920_D5_Pin
#define ST7920_D5_GPIO_Port GPIOE
#define ST7920_D5_Pin GPIO_PIN_13
#endif
#ifndef ST7920_D6_Pin
#define ST7920_D6_GPIO_Port GPIOE
#define ST7920_D6_Pin GPIO_PIN_14
#endif
#ifndef ST7920_D7_Pin
#define ST7920_D7_GPIO_Port GPIOE
#define ST7920_D7_Pin GPIO_PIN_15
#endif

// API (no chip-selects used for this board)
void st7920_init(void);
void st7920_enter_graphics_mode(void);
void st7920_write_command(uint8_t cmd);
void st7920_write_data(uint8_t data);
void st7920_clear(void);
void st7920_set_cursor(int row, int col);  // Position cursor: row (0-3), col (0-15)
void st7920_print(const char *s);
void st7920_print_line(int row, const char *str);  // Print exactly 16 chars on row (0 or 1), pad with spaces
void st7920_update_display(const char *line0, const char *line1);  // Safe update of both lines
void st7920_bus_test(void);

// Framebuffer API for 128x64 graphics mode
#define ST7920_FB_WIDTH  128
#define ST7920_FB_HEIGHT 64

void st7920_fb_clear(void);              // Clear framebuffer to 0
void st7920_set_pixel(int x, int y, int on);  // Set pixel on/off (on=1, off=0)
int st7920_get_pixel(int x, int y);      // Get pixel state
void st7920_paint(void);                 // Write framebuffer to display (slow)

#endif // __ST7920_H
