/*
 * platform.c
 *
 *  Created on: 2019年1月29日
 *      Author: liuch
 */
#include "platform.h"
#include "can.h"
#include "systick.h"
#include <string.h>

platform_velo targetVelo = { 0 };
wheelSpeed targetWS = { 0 };
wheelPos currentPos[4] = { 0 };	//leftfront,leftback,rightfront,rightback

uint8_t msgRecievedFlag = 0;

uint8_t canTxBuffer[8] = { 0 };
serialBuffer serialTxBuffer = { 0 };
serialBuffer serialRxBuffer = { 0 };

void timer_Init() {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;	 //声明一个结构体变量，用来初始化GPIO

	/* 开启定时器3时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	 //清除TIMx的中断待处理位:TIM 中断源
	TIM_TimeBaseInitStructure.TIM_Period = 2 * DRIVER_REPORT_CVP_PERIOD;//设置自动重装载寄存器周期的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 35999;//设置用来作为TIMx时钟频率预分频值，100Khz计数频率
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	TIM_Cmd(TIM3, ENABLE); //使能或者失能TIMx外设
	/* 设置中断参数，并打开中断 */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);	//使能或者失能指定的TIM中断

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);	 //清除TIMx的中断待处理位:TIM 中断源
	TIM_TimeBaseInitStructure.TIM_Period = 0xffff;	//设置自动重装载寄存器周期的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 35999;//设置用来作为TIMx时钟频率预分频值，100Khz计数频率
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
	TIM_Cmd(TIM4, ENABLE); //使能或者失能TIMx外设

}

void usart_init() {
	GPIO_InitTypeDef GPIO_InitStructure;	//声明一个结构体变量，用来初始化GPIO
	USART_InitTypeDef USART_InitStructure;	  //串口结构体定义
	DMA_InitTypeDef DMA_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	 //打开时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/*  配置GPIO的模式和IO口 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	 //TX			   //串口输出PA9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	    //复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure); /* 初始化串口输入IO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	    //RX			 //串口输入PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		  //模拟输入
	GPIO_Init(GPIOA, &GPIO_InitStructure); /* 初始化GPIO */

	USART_InitStructure.USART_BaudRate = 115200;   //波特率设置为9600	//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据长8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//1位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;				//无效验
	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None; //失能硬件流
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	 //开启发送和接受模式
	USART_Init(USART1, &USART_InitStructure); /* 初始化USART1 */
	USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
	USART_Cmd(USART1, ENABLE); /* 使能USART1 */
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);	 //使能或者失能指定的USART中断 接收中断
	USART_ClearFlag(USART1, USART_FLAG_IDLE);	 //清除USARTx的待处理标志位

	//TX
	DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) &USART1->DR;	 //DMA外设地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32) serialTxBuffer.raw;//DMA内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	 //外设作为数据传输的目标
	DMA_InitStructure.DMA_BufferSize = sizeof(serialTxBuffer);//指定DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	 //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//外设数据宽度16
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	//存储数据宽度8
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	 //工作在普通模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;	 //DMA通道x拥有高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;	 //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);	//ADC1在DMA1通道1内
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Channel4, DISABLE);

	//RX
	DMA_DeInit(DMA1_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) &USART1->DR;	 //DMA外设地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32) &serialRxBuffer.raw;//DMA内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	 //外设作为数据传输的来源
	DMA_InitStructure.DMA_BufferSize = sizeof(serialTxBuffer);//指定DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	 //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据宽度8
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	//存储数据宽度8
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	 //工作在普通模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;	 //DMA通道x拥有高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;	 //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);	//ADC1在DMA1通道1内
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Channel5, ENABLE);	//使能DMA1
}

void drivers_Init() {
	memset(canTxBuffer, 0x55, sizeof(canTxBuffer));
	CAN1_SendMesg(0x000, 8, canTxBuffer);	//reset drivers(group 0 broadcast)
	delay_ms(500);
	canTxBuffer[0] = DRIVER_MODE_V;
	CAN1_SendMesg(0x001, 8, canTxBuffer);	//set to V mode(group 0 broadcast)
	delay_ms(500);
	canTxBuffer[0] = DRIVER_REPORT_CVP_PERIOD;
	canTxBuffer[1] = 0x00;
	CAN1_SendMesg(0x00A, 8, canTxBuffer);//set report period(group 0 broadcast)
}

void NVIC_init() {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //打开TIM3_IRQn的全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	//抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //响应优先级为1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//使能
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn; //打开TIM3_IRQn的全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //响应优先级为1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//使能
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn; //打开TIM3_IRQn的全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //响应优先级为1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//使能
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 	   //打开USART1的全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 	 //抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 			//响应优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			 //使能
	NVIC_Init(&NVIC_InitStructure);

}

void platform_Init() {
	NVIC_init();
	usart_init();
	CAN1_Config(CAN_Mode_Normal);
	CAN1_Config16BitFilter(0xB, 0XF);	//receive special pack only
	drivers_Init();
	timer_Init();
}

void calcSpeed() {
	targetWS.left_front = (int16_t) (PARAM_A * (targetVelo.x + targetVelo.y)
			- PARAM_B * targetVelo.rotation);
	targetWS.left_back = (int16_t) (PARAM_A * (-targetVelo.x + targetVelo.y)
			- PARAM_B * targetVelo.rotation);
	targetWS.right_front = (int16_t) (PARAM_A * (-targetVelo.x + targetVelo.y)
			+ PARAM_B * targetVelo.rotation);
	targetWS.right_back = (int16_t) (PARAM_A * (targetVelo.x + targetVelo.y)
			+ PARAM_B * targetVelo.rotation);
}

void calcMove(float *fmeta) {
	fmeta[0] =
	PARAM_C
			* (currentPos[0].deltaPos - currentPos[1].deltaPos
					- currentPos[2].deltaPos + currentPos[3].deltaPos)
			* 60/ DRIVER_PULSE_PP;
	fmeta[1] =
	PARAM_C
			* (currentPos[0].deltaPos + currentPos[1].deltaPos
					+ currentPos[2].deltaPos + currentPos[3].deltaPos)
			* 60/ DRIVER_PULSE_PP;
	fmeta[2] =
	PARAM_D
			* (-currentPos[0].deltaPos - currentPos[1].deltaPos
					+ currentPos[2].deltaPos + currentPos[3].deltaPos)
			* 60/ DRIVER_PULSE_PP;
	fmeta[3] =
	PARAM_D
			* (-currentPos[0].deltaPos - currentPos[1].deltaPos
					+ currentPos[2].deltaPos + currentPos[3].deltaPos)
			* 60/ DRIVER_PULSE_PP;
	fmeta[4] =
	PARAM_D
			* (-currentPos[0].deltaPos - currentPos[1].deltaPos
					+ currentPos[2].deltaPos + currentPos[3].deltaPos)
			* 60/ DRIVER_PULSE_PP;
	fmeta[5] =
	PARAM_D
			* (-currentPos[0].deltaPos - currentPos[1].deltaPos
					+ currentPos[2].deltaPos + currentPos[3].deltaPos)
			* 60/ DRIVER_PULSE_PP;

}

uint8_t decodeUartData() {
	if (serialRxBuffer.idat[0])
		return 0;			//two 0x00 for authorize
	if ((~serialRxBuffer.idat[4]) | (~serialRxBuffer.idat[5])
			| (~serialRxBuffer.idat[6]) | (~serialRxBuffer.idat[7]))
		return 0;		//four 0xff for authorize
	targetVelo.x = serialRxBuffer.fdat[1];
	targetVelo.y = serialRxBuffer.fdat[2];
	targetVelo.rotation = serialRxBuffer.fdat[3];
	return 1;
}

void encodeUartData(float *f) {
	serialTxBuffer.fdat[1] = f[0];
	serialTxBuffer.fdat[2] = f[1];
	serialTxBuffer.fdat[3] = f[2];
	serialTxBuffer.idat[0] = 0;
	serialTxBuffer.fdat[4] = f[3];
	serialTxBuffer.fdat[5] = f[4];
	serialTxBuffer.fdat[6] = f[5];
	serialTxBuffer.idat[7] = 0xFFFFFFFF;
}

void readPosFromCan() {
	uint32_t tempPos;
	uint16_t tempVel;
	uint8_t *data = CanRxMessage.Data;
	uint8_t i;
	uint16_t tim = TIM_GetCounter(TIM4);
	if (CanRxMessage.StdId & 0xF == 0xB) {
		tempPos = ((uint32_t) data[4] << 24) | ((uint32_t) data[5] << 16)
				| ((uint32_t) data[6] << 8) | data[7];
		tempVel = ((uint16_t) data[2] << 8) | data[3];
		switch (CanRxMessage.StdId & 0xF0) {
		case DRIVER_ID_LF:
			i = 0;
			break;
		case DRIVER_ID_LB:
			i = 1;
			break;
		case DRIVER_ID_RF:
			i = 2;
			break;
		case DRIVER_ID_RB:
			i = 3;
			break;
		default:
			break;
		}
		currentPos[i].deltaPos = tempPos - currentPos[i].pos;
		currentPos[i].pos = tempPos;
		currentPos[i].timestamp = tim;
		currentPos[i].vel = tempVel;
	}
}

void TIM3_IRQHandler() {
	uint8_t i;
	uint16_t tmptime = TIM_GetCounter(TIM4);
	float fdat[6];
	if (TIM_GetITStatus(TIM3, TIM_IT_Update))
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	for (i = 0; i < 4; i++) {
		if (tmptime - currentPos[i].timestamp > DRIVER_REPORT_CVP_PERIOD * 2)
			break;
	}
	if (i == 4) {
		calcMove(fdat);
		// handle the data
		encodeUartData(fdat);

		DMA1_Channel4->CNDTR = sizeof(serialTxBuffer);
		DMA1_Channel4->CMAR = (u32) serialTxBuffer.raw;
		DMA_Cmd(DMA1_Channel4, ENABLE);	//使能DMA1
	}
}

//TX
void DMA1_Channel4_IRQHandler() {
	DMA_Cmd(DMA1_Channel4, DISABLE);
	if (DMA_GetFlagStatus(DMA1_FLAG_TC4) == SET)
		DMA_ClearFlag(DMA1_FLAG_TC4);
}

//RX
void DMA1_Channel5_IRQHandler() {
	uint8_t i;
	DMA_Cmd(DMA1_Channel5, DISABLE);
	if (DMA_GetFlagStatus(DMA1_FLAG_TC5) == SET)
		DMA_ClearFlag(DMA1_FLAG_TC5);

	//处理接收到的数据
	if (decodeUartData()) {
		calcSpeed();

		memset(canTxBuffer, 0x55, sizeof(canTxBuffer));
		canTxBuffer[0] = (DRIVER_LIMIT_PWM >> 8) & 0XFF;
		canTxBuffer[1] = DRIVER_LIMIT_PWM & 0XFF;
		canTxBuffer[2] = (targetWS.left_front >> 8) & 0XFF;
		canTxBuffer[3] = targetWS.left_front & 0XFF;
		CAN1_SendMesg((DRIVER_GROUP | DRIVER_ID_LF | (DRIVER_MODE_V + 1)), 8,
				canTxBuffer);
		canTxBuffer[2] = (targetWS.left_back >> 8) & 0XFF;
		canTxBuffer[3] = targetWS.left_back & 0XFF;
		CAN1_SendMesg((DRIVER_GROUP | DRIVER_ID_LB | (DRIVER_MODE_V + 1)), 8,
				canTxBuffer);
		canTxBuffer[2] = (targetWS.right_front >> 8) & 0XFF;
		canTxBuffer[3] = targetWS.right_front & 0XFF;
		CAN1_SendMesg((DRIVER_GROUP | DRIVER_ID_RF | (DRIVER_MODE_V + 1)), 8,
				canTxBuffer);
		canTxBuffer[2] = (targetWS.right_back >> 8) & 0XFF;
		canTxBuffer[3] = targetWS.right_back & 0XFF;

		// A wait is needed, because of only three mailbox.
		while ((CAN1->TSR & CAN_TSR_TME2) != CAN_TSR_TME2)
			;
		CAN1_SendMesg((DRIVER_GROUP | DRIVER_ID_RB | (DRIVER_MODE_V + 1)), 8,
				canTxBuffer);
	}
	DMA1_Channel5->CNDTR = sizeof(serialRxBuffer);
	DMA1_Channel5->CMAR = (u32) serialRxBuffer.raw;
	DMA_Cmd(DMA1_Channel5, ENABLE);	//使能DMA1*/
}

// if a frame is not full, abandon it!
void USART1_IRQHandler() {
	if (USART_GetFlagStatus(USART1, USART_FLAG_IDLE) == SET) {
		USART_ReceiveData(USART1);
		DMA_Cmd(DMA1_Channel5, DISABLE);
		DMA1_Channel5->CNDTR = sizeof(serialRxBuffer);
		DMA1_Channel5->CMAR = (u32) serialRxBuffer.raw;
		DMA_Cmd(DMA1_Channel5, ENABLE);	//使能DMA1*/
	}
}
