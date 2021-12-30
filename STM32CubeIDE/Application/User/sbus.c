/*
 * sbus.c
 *
 *  Created on: 10 нояб. 2021 г.
 *      Author: stanislav.perchenko
 */
#include "sbus.h"
#include "systick.h"

#define MIN_FRAME_PROTECTION_INTERVAL	5
#define RAW_SBUS_FRAME_LENGTH			25
#define SBUS_FRAME_HEADER				0x0F
#define SBUS_FRAME_FOOTER				0x00
#define CH17_MASK			0x01
#define CH18_MASK			0x20
#define FRAME_LOST_MASK		0x04
#define FAILSAFE_MASK		0x08

static uint16_t CH1, CH2, CH3, CH4, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16;
static uint8_t binCH17, binCH18;
static uint8_t frame_lost;
static uint8_t failsafe;


static uint8_t fl_data_ready;
static uint8_t fl_receive_in_progress;
static uint32_t t_last_frame_received;
static uint8_t next_frame_byte_index;
static uint32_t t_last_byte_received;
static uint8_t raw_data_buff[RAW_SBUS_FRAME_LENGTH];

static void ParseSBUS_Frame(uint8_t *raw);
static uint8_t ReverseBiots(uint8_t b);


void SBUS_Init(void)
{
	CH1=CH2=CH3=CH4=CH5=CH6=CH7=CH8=CH9=CH10=CH11=CH12=CH13=CH14=CH15=CH16 = 0;
	binCH17 = binCH18 = 0;
	frame_lost = 0;
	failsafe = 0;

	fl_data_ready = 0;
	fl_receive_in_progress = 0;
	t_last_frame_received = 0;
	next_frame_byte_index = 0;
	t_last_byte_received = 0;
}

uint8_t SBUS_IsData_Ready(void)
{
	return fl_data_ready;
}

uint32_t SBUS_TimeLastFrame_Received(void)
{
	return t_last_frame_received;
}

uint8_t SBUS_GetData(SBUS_Frame_TypeDef *sbus_frame)
{
	sbus_frame->servo_channels[0] = CH1;
	sbus_frame->servo_channels[1] = CH2;
	sbus_frame->servo_channels[2] = CH3;
	sbus_frame->servo_channels[3] = CH4;
	sbus_frame->servo_channels[4] = CH5;
	sbus_frame->servo_channels[5] = CH6;
	sbus_frame->servo_channels[6] = CH7;
	sbus_frame->servo_channels[7] = CH8;
	sbus_frame->servo_channels[8] = CH9;
	sbus_frame->servo_channels[9] = CH10;
	sbus_frame->servo_channels[10] = CH11;
	sbus_frame->servo_channels[11] = CH12;
	sbus_frame->servo_channels[12] = CH13;
	sbus_frame->servo_channels[13] = CH14;
	sbus_frame->servo_channels[14] = CH15;
	sbus_frame->servo_channels[15] = CH16;
	sbus_frame->ch17 = binCH17;
	sbus_frame->ch18 = binCH18;
	sbus_frame->frame_lost = frame_lost;
	sbus_frame->failsafe = failsafe;
	if (fl_data_ready) {
		fl_data_ready = 0;
		return 1;
	} else {
		return 0;
	}
}


uint8_t SBUS_OnData_Received(uint8_t data)
{

	//data = ReverseBiots(data);

	uint32_t t_now = SYS_GetTick();
	if ((t_now - t_last_byte_received) > MIN_FRAME_PROTECTION_INTERVAL) {
		fl_receive_in_progress = 0;
		next_frame_byte_index = 0;
	}
	t_last_byte_received = t_now;



	if (!fl_receive_in_progress)
	{
		if (data == SBUS_FRAME_HEADER) {
			raw_data_buff[0] = SBUS_FRAME_HEADER;
			next_frame_byte_index = 1;
			fl_receive_in_progress = 1;
		}
	}
	else
	{
		raw_data_buff[next_frame_byte_index ++] = data;
		if (next_frame_byte_index == RAW_SBUS_FRAME_LENGTH) {
			fl_receive_in_progress = 0;
			next_frame_byte_index = 0;
			if (data == SBUS_FRAME_FOOTER)
			{
				ParseSBUS_Frame(raw_data_buff);
				fl_data_ready = 1;
				t_last_frame_received = SYS_GetTick();
				return 1;
			}
		}
	}

	return 0;
}

static uint8_t ReverseBiots(uint8_t b) {
	static const uint8_t lookup[16] = {
			0x00, 0x08, 0x04, 0x0C, 0x02, 0x0A, 0x06, 0x0E,
			0x01, 0x09, 0x05, 0x0D, 0x03, 0x0B, 0x07, 0x0F
	};

	return (lookup[b & 0x0F] << 4) | lookup[(b >> 4) & 0x0F];
}


static void ParseSBUS_Frame(uint8_t *buf_)
{
	uint16_t v;

	if ((buf_[0] != SBUS_FRAME_HEADER) || (buf_[24] != SBUS_FRAME_FOOTER)) {
		return;
	}

	// Channel 1
	v = buf_[1];
	v |= (((uint16_t) buf_[2] << 8) & 0x7FF);
	CH1 = v;

	// Channel 2
	v = buf_[2] >> 3;
	v |= (((uint16_t) buf_[3] << 5) & 0x7FF);
	CH2 = v;

	// Channel 3
	v = buf_[3] >> 6;
	v |= (((uint16_t) buf_[4] << 2)  & 0x7FF);
	v |= (((uint16_t) buf_[5] << 10) & 0x7FF);
	CH3 = v;

	// Channel 4
	v = buf_[5] >> 1;
	v |= (((uint16_t) buf_[6] << 7) & 0x7FF);
	CH4 = v;

	// Channel 5
	v = buf_[6] >> 4;
	v |= (((uint16_t) buf_[7] << 4) & 0x7FF);
	CH5 = v;

	// Channel 6
	v = buf_[7] >> 7;
	v |= (((uint16_t) buf_[8] << 1) & 0x7FF);
	v |= (((uint16_t) buf_[9] << 9) & 0x7FF);
	CH6 = v;

	// Channel 7
	v = buf_[9] >> 2;
	v |= (((uint16_t) buf_[10] << 6) & 0x7FF);
	CH7 = v;

	// Channel 8
	v = buf_[10] >> 5;
	v |= (((uint16_t) buf_[11] << 3) & 0x7FF);
	CH8 = v;

	// Channel 9
	v = buf_[12];
	v |= (((uint16_t) buf_[13] << 8) & 0x7FF);
	CH9 = v;

	// Channel 10
	v = buf_[13] >> 3;
	v |= (((uint16_t) buf_[14] << 5) & 0x7FF);
	CH10 = v;

	// Channel 11
	v = buf_[14] >> 6;
	v |= (((uint16_t) buf_[15] << 2) & 0x7FF);
	v |= (((uint16_t) buf_[16] << 10) & 0x7FF);
	CH11 = v;

	// Channel 12
	v = buf_[16] >> 1;
	v |= (((uint16_t) buf_[17] << 7) & 0x7FF);
	CH12 = v;

	// Channel 13
	v = buf_[17] >> 4;
	v |= (((uint16_t) buf_[18] << 4) & 0x7FF);\
	CH13 = v;

	// Channel 14
	v = buf_[18] >> 7;
	v |= (((uint16_t) buf_[19] << 1) & 0x7FF);
	v |= (((uint16_t) buf_[20] << 9) & 0x7FF);
	CH14 = v;

	// Channel 15
	v = buf_[20] >> 2;
	v |= (((uint16_t) buf_[21] << 6) & 0x7FF);\
	CH15 = v;

	// Channel 16
	v = buf_[21] >> 5;
	v |= (((uint16_t) buf_[2] << 3) & 0x7FF);\
	CH16 = v;

	// Channel 17
	binCH17 = (buf_[23] & CH17_MASK) != 0;

	// Channel 18
	binCH18 = (buf_[23] & CH18_MASK) != 0;

	// Frame lost
	frame_lost = (buf_[23] & FRAME_LOST_MASK) != 0;

	// FailSafe
	failsafe = (buf_[23] & FAILSAFE_MASK) != 0;
}



