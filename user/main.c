#include "public.h"
#include "led.h"
#include "platform.h"
#include "gui.h"

unsigned char str[] = { 1, 2, 3, 4, 5, 6, 7, 8 };

int main() {
	TFT_Init();	  //TFT������ʼ��
	LED_Init();	 //LED��ʼ��
	TFT_ClearScreen(BLACK);	 //����
	GUI_Show12ASCII(80, 130, "hello world!", YELLOW, BLACK);
	delay_ms(500);
	TFT_ClearScreen(WHITE);
	platform_Init();
	GUI_Show12ASCII(0, 0, "Platform_Inited!", BLACK, WHITE);
	GUI_Show12ASCII(0, 16, "RXMessage", BLACK, WHITE);
	GUI_Show12ASCII(0, 48, "TXMessage", BLACK, WHITE);
	while (1) {
		led_display();	//LED��˸
		GUI_Show12ASCII(0, 32, serialRxBuffer.raw, BLUE, WHITE);
		GUI_Show12ASCII(0, 64, serialTxBuffer.raw, BLUE, WHITE);
	}
}
