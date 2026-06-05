#include "watchdog.h"

static unsigned long lastFeedTime = 0;

void watchdogInit()
{
    WDOG1_WSR = 0x5555;
    WDOG1_WSR = 0xAAAA;
    WDOG1_WCR = (3 << 8) | (1 << 2);
}

void watchdogFeed()
{
    lastFeedTime = millis();
    WDOG1_WSR = 0x5555;
    WDOG1_WSR = 0xAAAA;
}

void watchdogCheck()
{
    if(millis() - lastFeedTime > 1000)
    {
        SCB_AIRCR = 0x05FA0004;
        while(1);
    }
}
