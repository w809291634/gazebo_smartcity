#include <sys/time.h>
#include <stdint.h>
#include <stdio.h>

uint32_t millis(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint32_t t = tv.tv_sec*1000+tv.tv_usec/1000;
    return t;
}

void msleep(int ms)
{
	struct timeval delay;
	delay.tv_sec = ms/1000;
	delay.tv_usec = (ms%1000) * 1000; 
	select(0, NULL, NULL, NULL, &delay);
}

int timeoutMs(uint32_t ms)
{
    if (ms == 0) return 1;
    uint32_t ct = millis();
    int r = (int32_t)ct - (int32_t)ms;
    return r >= 0;
}
