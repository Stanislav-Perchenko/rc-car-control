/*
 * logger.c
 *
 *  Created on: 9 нояб. 2021 г.
 *      Author: stanislav.perchenko
 */
#include "logger.h"
#include "systick.h"
#include "stm32f1xx_ll_usart.h"

#define USART_LOG USART2

static USART_TypeDef *usart_log;
static uint8_t log_tx_buffer[LOG_BUFFER_SIZE];

static uint16_t data_length;
static uint16_t start_index;

static uint32_t log_tx_time_finished;

static void AddLogDataByte(uint8_t data_byte);


void LOG_ResetLogger(USART_TypeDef *usart)
{
	usart_log = usart;
	start_index = 0;
	data_length = 0;
	log_tx_time_finished = SYS_GetTick() + 1;
}

uint8_t LOG_IsInProgress(void)
{
	return (data_length > 0);
}


uint32_t LOG_GetTimeLastLogFinished(void)
{
	return log_tx_time_finished;
}


void LOG_SendLog(uint8_t *data, uint16_t offset, uint16_t n_bytes)
{
	if (!usart_log || (n_bytes == 0) || (data_length >= LOG_BUFFER_SIZE)) return;

	LL_USART_DisableIT_TXE(usart_log);


	do {
		AddLogDataByte(data[offset ++]);
		n_bytes --;
	} while((n_bytes > 0) && (data_length < LOG_BUFFER_SIZE));


	LOG_EvaluateDataSend();
	LL_USART_EnableIT_TXE(usart_log);


}

static void AddLogDataByte(uint8_t data_byte)
{
	uint16_t next_index;
	if (data_length < LOG_BUFFER_SIZE)
	{
		next_index = start_index + data_length;
		if (next_index >= LOG_BUFFER_SIZE) {
			next_index -= LOG_BUFFER_SIZE;
		}

		log_tx_buffer[next_index] = data_byte;
		data_length ++;
	}
}


void LOG_EvaluateDataSend(void)
{
	uint8_t byte_to_send;

	if (!usart_log || (data_length == 0) || !LL_USART_IsActiveFlag_TXE(usart_log))
	{
		return;
	}


	byte_to_send = log_tx_buffer[start_index ++];
	if (start_index == LOG_BUFFER_SIZE) {
		start_index = 0;
	}
	data_length --;

	LL_USART_TransmitData8(usart_log, byte_to_send);

	if (data_length == 0) {
		log_tx_time_finished = SYS_GetTick();
	}
}


