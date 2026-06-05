#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <Arduino.h>

// i.MX RT1062 WDOG1 registers (base 0x400B8000)
#define WDOG1_WCR   (*(volatile uint16_t *)0x400B8000)
#define WDOG1_WSR   (*(volatile uint16_t *)0x400B8002)

// ARM Cortex-M7 SCB_AIRCR for system reset
#define SCB_AIRCR   (*(volatile uint32_t *)0xE000ED0C)

void watchdogInit();
void watchdogFeed();
void watchdogCheck();

#endif
