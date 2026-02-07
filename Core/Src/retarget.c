/* Retarget low-level I/O to SWO/ITM for printf support */
#include "stm32f4xx.h"

int __io_putchar(int ch)
{
  /* Send character to ITM (SWO) — requires debugger SWO enabled */
  ITM_SendChar(ch);
  return ch;
}

int __io_getchar(void)
{
  return 0;
}
