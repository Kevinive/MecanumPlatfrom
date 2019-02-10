#ifndef _led_H
#define _led_H
#include "stm32f10x.h"
#define LED GPIO_Pin_1

void led_display(void);
void LED_Init(void);
void delay(u32);

#endif
