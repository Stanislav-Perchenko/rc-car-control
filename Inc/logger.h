/*
 * logger.h
 *
 *  Created on: 9 нояб. 2021 г.
 *      Author: stanislav.perchenko
 */

#ifndef __LOGGER_H__
#define __LOGGER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f1xx.h"

#define LOG_BUFFER_SIZE		((uint16_t) 460)

void LOG_ResetLogger(USART_TypeDef *usart);

uint8_t LOG_IsInProgress(void);

uint32_t LOG_GetTimeLastLogFinished(void);

void LOG_SendLog(uint8_t *data, uint16_t offsed, uint16_t n_bytes);

void LOG_EvaluateDataSend(void);


#ifdef __cplusplus
}
#endif


#endif /* __LOGGER_H__ */
