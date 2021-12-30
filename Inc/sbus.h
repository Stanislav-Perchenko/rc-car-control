/*
 * sbus.h
 *
 *  Created on: 10 нояб. 2021 г.
 *      Author: stanislav.perchenko
 */

#ifndef __SBUS_H_
#define __SBUS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
	uint16_t servo_channels[16];
	uint8_t ch17;
	uint8_t ch18;
	uint8_t frame_lost;
	uint8_t failsafe;
} SBUS_Frame_TypeDef;

void SBUS_Init(void);
uint8_t SBUS_OnData_Received(uint8_t data);
uint8_t SBUS_IsData_Ready(void);
uint32_t SBUS_TimeLastFrame_Received(void);
uint8_t SBUS_GetData(SBUS_Frame_TypeDef *sbus_frame);

#ifdef __cplusplus
}
#endif


#endif /* __SBUS_H_ */
