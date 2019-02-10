#include "led.h"
void delay(u32 i){
	while(i--);
}

void LED_Init(){	//LED³õÊ¼»¯º¯Êý
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = LED;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	SystemInit();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_Init(GPIOC,&GPIO_InitStructure);
}

void led_display(){
	GPIO_SetBits(GPIOC,LED);
	delay(6000000);
	GPIO_ResetBits(GPIOC,LED);
	delay(6000000);
}
