/* st7920.c - 8-bit parallel ST7920 driver with framebuffer support (128x64 graphics)
   HAL-based, assumes CubeMX GPIO initialization.
   Framebuffer: 128x64 / 8 = 1024 bytes, each byte = 8 pixels (MSB = leftmost).
*/

#include "st7920.h"
#include "main.h"
#include <string.h>

// Framebuffer: 128 pixels wide, 64 pixels high => 16 bytes per row, 64 rows
#define FB_STRIDE  (ST7920_FB_WIDTH / 8)  // 16 bytes per row
#define FB_SIZE    (FB_STRIDE * ST7920_FB_HEIGHT)  // 1024 bytes
static uint8_t framebuffer[FB_SIZE];  // 1 KB framebuffer

static void st7920_write_bus(uint8_t v)
{
  HAL_GPIO_WritePin(ST7920_D0_GPIO_Port, ST7920_D0_Pin, (v & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ST7920_D1_GPIO_Port, ST7920_D1_Pin, (v & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ST7920_D2_GPIO_Port, ST7920_D2_Pin, (v & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ST7920_D3_GPIO_Port, ST7920_D3_Pin, (v & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ST7920_D4_GPIO_Port, ST7920_D4_Pin, (v & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ST7920_D5_GPIO_Port, ST7920_D5_Pin, (v & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ST7920_D6_GPIO_Port, ST7920_D6_Pin, (v & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ST7920_D7_GPIO_Port, ST7920_D7_Pin, (v & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void st7920_pulse_e(void)
{
  HAL_GPIO_WritePin(ST7920_E_GPIO_Port, ST7920_E_Pin, GPIO_PIN_SET);
  for (volatile int i = 0; i < 80; ++i) __NOP();
  HAL_GPIO_WritePin(ST7920_E_GPIO_Port, ST7920_E_Pin, GPIO_PIN_RESET);
  for (volatile int i = 0; i < 40; ++i) __NOP();
}

void st7920_write_command(uint8_t cmd)
{
  HAL_GPIO_WritePin(ST7920_RS_GPIO_Port, ST7920_RS_Pin, GPIO_PIN_RESET); // RS = 0 -> command
  HAL_GPIO_WritePin(ST7920_WR_GPIO_Port, ST7920_WR_Pin, GPIO_PIN_RESET); // RW = 0 -> write
  st7920_write_bus(cmd);
  st7920_pulse_e();
  HAL_GPIO_WritePin(ST7920_WR_GPIO_Port, ST7920_WR_Pin, GPIO_PIN_SET);
  // small settle
  for (volatile int i = 0; i < 200; ++i) __NOP();
}

void st7920_write_data(uint8_t data)
{
  HAL_GPIO_WritePin(ST7920_RS_GPIO_Port, ST7920_RS_Pin, GPIO_PIN_SET); // RS = 1 -> data
  HAL_GPIO_WritePin(ST7920_WR_GPIO_Port, ST7920_WR_Pin, GPIO_PIN_RESET); // RW = 0 -> write
  st7920_write_bus(data);
  st7920_pulse_e();
  HAL_GPIO_WritePin(ST7920_WR_GPIO_Port, ST7920_WR_Pin, GPIO_PIN_SET);
  for (volatile int i = 0; i < 100; ++i) __NOP();
}

void st7920_clear(void)
{
  // Clear display instruction (may act on text area)
  st7920_write_command(0x01);
  HAL_Delay(2);
}

void st7920_print(const char *s)
{
  while (*s) {
    st7920_write_data((uint8_t)*s++);
  }
}

// --- Framebuffer functions (fast pixel access) ---

void st7920_fb_clear(void)
{
  memset(framebuffer, 0, FB_SIZE);
}

void st7920_set_pixel(int x, int y, int on)
{
  if (x < 0 || x >= ST7920_FB_WIDTH || y < 0 || y >= ST7920_FB_HEIGHT)
    return;  // out of bounds
  
  int byte_idx = (y * FB_STRIDE) + (x / 8);
  int bit_pos = 7 - (x % 8);  // MSB = leftmost pixel
  
  if (on)
    framebuffer[byte_idx] |= (1U << bit_pos);
  else
    framebuffer[byte_idx] &= ~(1U << bit_pos);
}

int st7920_get_pixel(int x, int y)
{
  if (x < 0 || x >= ST7920_FB_WIDTH || y < 0 || y >= ST7920_FB_HEIGHT)
    return 0;
  
  int byte_idx = (y * FB_STRIDE) + (x / 8);
  int bit_pos = 7 - (x % 8);
  
  return (framebuffer[byte_idx] >> bit_pos) & 1;
}

void st7920_paint(void)
{
  // Write framebuffer to ST7920 GDRAM (graphics RAM)
  // ST7920 GDRAM: 0x00-0x7F for each of 2 vertical halves (64 rows / 2 = 32 + 32)
  // For each row y: set row address, then write 16 bytes of pixel data
  
  for (int y = 0; y < ST7920_FB_HEIGHT; ++y) {
    // Set vertical address
    if (y < 32) {
      st7920_write_command(0x80 + y);  // Set Y address (0x80-0x9F for upper half)
    } else {
      st7920_write_command(0x80 + (y - 32));  // 0x80-0x9F for lower half
    }
    // Set horizontal address (X=0)
    st7920_write_command(0x80);  // 0x80 = X address 0
    
    // Write 16 bytes for this row
    int row_offset = y * FB_STRIDE;
    for (int i = 0; i < FB_STRIDE; ++i) {
      st7920_write_data(framebuffer[row_offset + i]);
    }
  }
}


void st7920_init(void)
{
  // Assume CubeMX `MX_GPIO_Init()` configured LCD pins. Here only set safe
  // default pin states and send init commands.

  // set PSB high for parallel mode, and default control pin states (WR high, RS low, E low)
  HAL_GPIO_WritePin(ST7920_PSB_GPIO_Port, ST7920_PSB_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ST7920_WR_GPIO_Port, ST7920_WR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ST7920_RS_GPIO_Port, ST7920_RS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ST7920_E_GPIO_Port, ST7920_E_Pin, GPIO_PIN_RESET);

  HAL_Delay(10);

  // Init sequence for graphics mode (128x64)
  st7920_write_command(0x30); // Basic instruction set (not extended)
  HAL_Delay(1);
  st7920_write_command(0x0C); // Display on, cursor off, blink off
  HAL_Delay(1);
  st7920_write_command(0x34); // Extended instruction set + graphics mode enable
  HAL_Delay(1);
  
  // Clear framebuffer and display
  st7920_fb_clear();
  st7920_paint();
  HAL_Delay(1);
}

// Diagnostic: drive patterns to data bus and pulse E so you can probe signals
void st7920_bus_test(void)
{
  const uint8_t patterns[] = { 0xFF, 0x00, 0xAA, 0x55 };
  for (int p = 0; p < (int)(sizeof(patterns)); ++p) {
    uint8_t v = patterns[p];
    // Drive both halves sequentially so you can observe CS selection
    st7920_write_bus(v);
    // write twice so you can observe activity on E and data lines
    st7920_write_data(v);
    HAL_Delay(200);
    st7920_write_data(v);
    HAL_Delay(200);
  }
}
