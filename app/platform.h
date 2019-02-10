/*
 * platform.h
 *
 *  Created on: 2019年1月29日
 *      Author: liuch
 */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#include "stm32f10x.h"

#define PLATFORM_X					//wheel distance in X axis(mm)
#define PLATFORM_Y					//wheel distance in Y axis(mm)
#define PLATFORM_A			PLATFORM_X/2
#define PLATFORM_B			PLATFORM_Y/2

#define DRIVER_ID_LF		0x010
#define DRIVER_ID_LB		0x020
#define DRIVER_ID_RF		0x030
#define DRIVER_ID_RB		0x040
#define DRIVER_GROUP		0X000

#define DRIVER_MODE_O		0X01
#define DRIVER_MODE_C		0X02
#define DRIVER_MODE_V		0X03
#define DRIVER_MODE_P		0X04
#define DRIVER_MODE_VP		0X05
#define DRIVER_MODE_CV		0X06
#define DRIVER_MODE_CP		0X07
#define DRIVER_MODE_CVP		0X08

#define DRIVER_REPORT_CVP_PERIOD		100
#define DRIVER_LIMIT_PWM	5000
#define DRIVER_PULSE_PP		512

#define PARAM_A				0.12532
#define PARAM_B				28.1967
#define PARAM_C				1.9949
#define PARAM_D				0.0088663

typedef struct PlatformVelocity{
	float x;
	float y;
	float rotation;
}platform_velo;

typedef struct rotSpeed{
	int16_t left_front;
	int16_t left_back;
	int16_t right_front;
	int16_t right_back;
}wheelSpeed;

typedef struct position{
	uint16_t timestamp;
	uint32_t pos;
	uint32_t deltaPos;
	uint16_t vel;
}wheelPos;

typedef union buf{
	uint8_t raw[32];
	uint32_t idat[8];
	float fdat[8];
}serialBuffer;

extern platform_velo targetVelo;
extern wheelSpeed targetWS;
extern wheelPos currentPos[4];		// 顺序：左前 左后 右前 右后
extern uint8_t canTxBuffer[8];
extern serialBuffer serialTxBuffer;
extern serialBuffer serialRxBuffer;

extern void platform_Init();
extern void readPosFromCan();


#endif /* _PLATFORM_H_ */
