// 8x8 font for ASCII characters 32-127
// Each character is 8 bytes, each byte is a row (MSB = leftmost pixel)
#ifndef FONT8X8_H
#define FONT8X8_H

#include <stdint.h>

extern const uint8_t font8x8_basic[96][8];

#endif // FONT8X8_H
