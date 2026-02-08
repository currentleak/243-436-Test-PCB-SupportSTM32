/* st7920.c - 8-bit parallel ST7920 driver with framebuffer support (128x64 graphics)
   HAL-based, assumes CubeMX GPIO initialization.
   Framebuffer: 128x64 / 8 = 1024 bytes, each byte = 8 pixels (MSB = leftmost).
*/

#include <string.h>
#include <stdint.h>
#include "st7920.h"
#include "main.h"

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
  for (volatile int i = 0; i < 200; ++i) __NOP();
  HAL_Delay(1);  // Add delay for LCD settle
}

void st7920_clear(void)
{
  // Clear GDRAM: write 0x00 to all 128x64 pixels
  // GDRAM has 64 rows, each row = 16 bytes (128 bits)
  // Layout: first 32 rows (0x00-0x1F), then another 32 rows (0x20-0x3F)
  
  // Must be in extended mode (RE=1) and graphics on (G=1)
  // For TEXT mode, just use the clear display command
  st7920_write_command(0x01);  // Clear display (text mode)
  HAL_Delay(5);  // Generous delay for clear
}

void st7920_set_cursor(int row, int col)
{
  // Position cursor for text mode (row 0-3, col 0-15)
  // DDRAM address calculation: 0x80 + row*0x10 + col
  if (row < 0 || row > 3 || col < 0 || col > 15)
    return;  // out of range
  
  uint8_t address = 0x80 + (row * 0x10) + col;
  st7920_write_command(address);
  HAL_Delay(2);  // Increased delay for cursor positioning
}

void st7920_print_line(int row, const char *str)
{
  // Print exactly 16 characters on a line (row 0 or 1), padded with spaces
  if (row < 0 || row > 1)
    return;  // only lines 0 and 1 are visible on 2-line display
  
  // Position cursor at start of line
  st7920_set_cursor(row, 0);
  // Extra delay after cursor positioning before writing data
  HAL_Delay(3);
  
  // Print up to 16 characters, or pad with spaces if shorter
  char buffer[17];
  int i = 0;
  // Copy string up to 16 chars
  while (i < 16 && str && str[i]) {
    buffer[i] = str[i];
    i++;
  }
  // Pad with spaces if needed
  while (i < 16) {
    buffer[i] = ' ';
    i++;
  }
  buffer[16] = '\0';
  
  // Print the exactly 16-char string
  st7920_print(buffer);
  HAL_Delay(1);  // Settle after line write
}

void st7920_update_display(const char *line0, const char *line1)
{
  // Safe update of both lines with proper timing
  // Reset display state
  st7920_write_command(0x0C);  // Display on, cursor off
  HAL_Delay(2);
  
  // Write line 0
  st7920_print_line(0, line0);
  HAL_Delay(3);  // Settle between lines
  
  // Write line 1
  st7920_print_line(1, line1);
  HAL_Delay(2);  // Settle after last line
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
  // Write framebuffer to ST7920 GDRAM
  // 64 rows total: first write sets upper half (rows 0-31), auto-wraps for lower (rows 32-63)
  
  for (int row = 0; row < 32; ++row) {
    // Set vertical address (0x80-0x9F for upper half)
    st7920_write_command(0x80 + row);
    // Set horizontal address (0x80 = column 0)
    st7920_write_command(0x80);
    
    // Write 16 bytes for this row (upper half)
    int offset = row * FB_STRIDE;
    for (int col = 0; col < FB_STRIDE; ++col) {
      st7920_write_data(framebuffer[offset + col]);
    }
  }
  
  // Repeat for lower half (rows 32-63)
  // Address automatically wraps, but we repeat the command sequence to be safe
  for (int row = 0; row < 32; ++row) {
    st7920_write_command(0x80 + row);
    st7920_write_command(0x80);
    
    int offset = (32 + row) * FB_STRIDE;
    for (int col = 0; col < FB_STRIDE; ++col) {
      st7920_write_data(framebuffer[offset + col]);
    }
  }
  
  HAL_Delay(2);
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

  HAL_Delay(15);

  // Init sequence for TEXT mode (basic mode, not graphics)
  // Step 1: Function set - basic instruction set (RE=0)
  st7920_write_command(0x30);
  HAL_Delay(2);
  
  // Step 2: Display control (on, cursor off, blink off)
  st7920_write_command(0x0C);
  HAL_Delay(2);
  
  // Step 3: Entry mode set (increment, no shift)
  st7920_write_command(0x06);
  HAL_Delay(1);
  
  // Step 4: Clear display
  st7920_write_command(0x01);
  HAL_Delay(5);
}

void st7920_enter_graphics_mode(void)
{
  // Exit text mode and enter graphics mode
  // Must follow proper sequence to avoid corruption
  
  // Step 1: Switch to extended instruction set (RE=1, G=0)
  st7920_write_command(0x30);  // Basic set first (if already in extended, this goes back to basic)
  HAL_Delay(1);
  st7920_write_command(0x34);  // 0011 0100 = RE=1, G=0 (extended, graphics OFF)
  HAL_Delay(5);
  
  // Step 2: Enable graphics mode (RE=1, G=1)
  st7920_write_command(0x36);  // 0011 0110 = RE=1, G=1 (extended, graphics ON)
  HAL_Delay(5);
  
  // Step 3: Clear all GDRAM
  st7920_clear();
  HAL_Delay(2);
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
