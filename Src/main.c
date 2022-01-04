/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "systick.h"
#include "logger.h"
#include "eeprom.h"
#include "sbus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	uint16_t rudder;
	uint16_t throttle;
	uint16_t cam_yaw;
	uint16_t cam_pitch;
	uint16_t out1;
} MixedControls_TypeDef;

typedef struct {
	uint8_t rudder_ch_num;
	uint8_t throttle_ch_num;
	uint8_t cam_yaw_ch_num;
	uint8_t cam_pitch_ch_num;
	uint8_t out1_ch_num;
} ControlsMixerParams_TypeDef;

typedef struct {
	uint16_t rudder_min;
	uint16_t rudder_center;
	uint16_t rudder_max;
	uint16_t throttle_min;
	uint16_t throttle_center;
	uint16_t throttle_max;
	uint16_t cam_yaw_min;
	uint16_t cam_yaw_center;
	uint16_t cam_yaw_max;
	uint16_t cam_pitch_min;
	uint16_t cam_pitch_center;
	uint16_t cam_pitch_max;
	uint16_t out1_min;
	uint16_t out1_lo_thr;
	uint16_t out1_hi_thr;
	uint16_t out1_max;
} ControlsClipParams_TypeDef;

typedef struct {
	int8_t rudder;
	int8_t throttle;
	int8_t cam_yaw;
	int8_t cam_pitch;
	uint8_t out1;
} AbstractControls_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TOTAL_EEPROM_DATA_SIZE	16

#define EE_ADDR_RUDDER_MIN		((uint16_t) 0x0100)
#define EE_ADDR_RUDDER_CENT		((uint16_t) 0x0101)
#define EE_ADDR_RUDDER_MAX		((uint16_t) 0x0102)
#define EE_ADDR_THROT_MIN		((uint16_t) 0x0103)
#define EE_ADDR_THROT_CENT		((uint16_t) 0x0104)
#define EE_ADDR_THROT_MAX		((uint16_t) 0x0105)
#define EE_ADDR_CAMYAW_MIN		((uint16_t) 0x0106)
#define EE_ADDR_CAMYAW_CENT		((uint16_t) 0x0107)
#define EE_ADDR_CAMYAW_MAX		((uint16_t) 0x0108)
#define EE_ADDR_CAMPIT_MIN		((uint16_t) 0x0109)
#define EE_ADDR_CAMPIT_CENT		((uint16_t) 0x010A)
#define EE_ADDR_CAMPIT_MAX		((uint16_t) 0x010B)
#define EE_ADDR_OUT1_MIN		((uint16_t) 0x010C)
#define EE_ADDR_OUT1_LOTH		((uint16_t) 0x010D)
#define EE_ADDR_OUT1_HITH		((uint16_t) 0x010E)
#define EE_ADDR_OUT1_MAX		((uint16_t) 0x010F)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t EepromAllVirtAddrTable[TOTAL_EEPROM_DATA_SIZE] = {	EE_ADDR_RUDDER_MIN, EE_ADDR_RUDDER_CENT, EE_ADDR_RUDDER_MAX, EE_ADDR_THROT_MIN,  EE_ADDR_THROT_CENT,  EE_ADDR_THROT_MAX,
															EE_ADDR_CAMYAW_MIN, EE_ADDR_CAMYAW_CENT, EE_ADDR_CAMYAW_MAX, EE_ADDR_CAMPIT_MIN, EE_ADDR_CAMPIT_CENT, EE_ADDR_CAMPIT_MAX,
															EE_ADDR_OUT1_MIN, EE_ADDR_OUT1_LOTH, EE_ADDR_OUT1_HITH, EE_ADDR_OUT1_MAX};

static uint8_t log_buffer[120];

static ControlsMixerParams_TypeDef controls_mix_params = {
		SBUS_CH_NUM_RUDDER,
		SBUS_CH_NUM_THROTTLE,
		SBUS_CH_NUM_CAM_YAW,
		SBUS_CH_NUM_CAM_PITCH,
		SBUS_CH_NUM_OUT1
};
static ControlsClipParams_TypeDef controls_clip_params = {0};
static SBUS_Frame_TypeDef raw_sbus_frame = {0};
static MixedControls_TypeDef mixed_controls_raw = {0};
static MixedControls_TypeDef mixed_controls_clipped = {0};
static AbstractControls_TypeDef abstract_controls = {0};



/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t EepromAllVirtAddrTable[TOTAL_EEPROM_DATA_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/**
 * @brief  Reads calibration data from EEPROM and save it to the dst parameter
 * @retval int 1 - calibration data was found; 0 - calibration data was not found
 */
static uint8_t RestoreControlClipParams(ControlsClipParams_TypeDef *dst);
static void SaveControlClipParams(ControlsClipParams_TypeDef *clip_params);
static void MixControlsFromSBUS(SBUS_Frame_TypeDef *sbus_frame_in, MixedControls_TypeDef *raw_controls_out, ControlsMixerParams_TypeDef *mix_params);
static void ClipMixedControls(MixedControls_TypeDef *new_controls, MixedControls_TypeDef *clipped_out, ControlsClipParams_TypeDef *clip_params);
static uint16_t ClipValue(uint16_t x, uint16_t min, uint16_t cent, uint16_t max, uint16_t protect_percent);
static void UpdateCalibrationWithNewData(MixedControls_TypeDef *raw_controls);
static void MapClippedControlsToAbstractInterval(MixedControls_TypeDef *src, ControlsClipParams_TypeDef *clip_params, AbstractControls_TypeDef *dst);
static int8_t MapValueToAbstract(uint16_t x, uint16_t min, uint16_t center, uint16_t max);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SysTick->CTRL  &= ~SysTick_CTRL_CLKSOURCE_Msk;
  SYS_ClearTick();
  SYS_IncTick();
  SYS_IncTick();
  SYS_ResumeTick();
  FLASH_Unlock();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  EE_Init(EepromAllVirtAddrTable, TOTAL_EEPROM_DATA_SIZE);		// EEPROM Init
  LOG_ResetLogger(USART_LOG);									// Logger Init
  SBUS_Init();
  GPIO_BOARD_Led_OFF();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t t_now;
  uint16_t n_bytes;
  uint32_t main_frame_log_last_time = SYS_GetTick();

  uint8_t is_in_calibration_mode = !RestoreControlClipParams(&controls_clip_params) || GPIO_IsCalibrationON();
  uint8_t is_first_sbus_frame = 1;

  if (is_in_calibration_mode) {
	  n_bytes = sprintf((char *)log_buffer, "\r\nStart calibration mode\r\n");
	  LOG_SendLog(log_buffer, 0, n_bytes);
  }

  while (1)
  {
	  t_now = SYS_GetTick();

	  if (SBUS_IsData_Ready())
	  {
		  SBUS_GetData(&raw_sbus_frame);

		  MixControlsFromSBUS(&raw_sbus_frame, &mixed_controls_raw, &controls_mix_params);

		  if (is_in_calibration_mode)
		  {
			  UpdateCalibrationWithNewData(&mixed_controls_raw);
		  }
		  else
		  {

			  ClipMixedControls(&mixed_controls_raw, &mixed_controls_clipped, &controls_clip_params);

			  MapClippedControlsToAbstractInterval(&mixed_controls_clipped, &controls_clip_params, &abstract_controls);



			  if ((t_now - main_frame_log_last_time) > 400)
			  {
				  main_frame_log_last_time = t_now;
				  n_bytes = sprintf((char *)log_buffer, "\r\n\n\n----------------------------------------------------------------\r\nthrot\t\trud\t\tyaw\t\tpitch\t\tout1\r\n");
				  LOG_SendLog(log_buffer, 0, n_bytes);
				  n_bytes = sprintf((char *)log_buffer, "%d\t\t%d\t\t%d\t\t%d\t\t%d\r\n", mixed_controls_raw.throttle, mixed_controls_raw.rudder, mixed_controls_raw.cam_yaw, mixed_controls_raw.cam_pitch, mixed_controls_raw.out1);
				  LOG_SendLog(log_buffer, 0, n_bytes);
				  n_bytes = sprintf((char *)log_buffer, "%d\t\t%d\t\t%d\t\t%d\t\t%d\r\n", mixed_controls_clipped.throttle, mixed_controls_clipped.rudder, mixed_controls_clipped.cam_yaw, mixed_controls_clipped.cam_pitch, mixed_controls_clipped.out1);
				  LOG_SendLog(log_buffer, 0, n_bytes);
				  n_bytes = sprintf((char *)log_buffer, "%d\t\t%d\t\t%d\t\t%d\t\t%d\r\n", abstract_controls.throttle, abstract_controls.rudder, abstract_controls.cam_yaw, abstract_controls.cam_pitch, abstract_controls.out1);
				  LOG_SendLog(log_buffer, 0, n_bytes);
			  }

			  //TODO Implement Further !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		  }

		  is_first_sbus_frame = 0;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  MX_IWDG_Init();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_6);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_4);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(6000000);
  LL_SetSystemCoreClock(48000000);
}

/* USER CODE BEGIN 4 */
void USART1_RX_Callback(void)
{
	uint8_t data = LL_USART_ReceiveData8(USART1);
	SBUS_OnData_Received(data);
}

void USART2_TX_Callback(void)
{
	LOG_EvaluateDataSend();
	if(!LOG_IsInProgress()) {
		LL_USART_DisableIT_TXE(USART_LOG);
	}
}

static void MixControlsFromSBUS(SBUS_Frame_TypeDef *sbus_frame_in, MixedControls_TypeDef *raw_controls_out, ControlsMixerParams_TypeDef *mix_params)
{
	raw_controls_out->rudder = sbus_frame_in->servo_channels[mix_params->rudder_ch_num - 1];
	raw_controls_out->throttle = sbus_frame_in->servo_channels[mix_params->throttle_ch_num - 1];
	raw_controls_out->cam_yaw = sbus_frame_in->servo_channels[mix_params->cam_yaw_ch_num - 1];
	raw_controls_out->cam_pitch = sbus_frame_in->servo_channels[mix_params->cam_pitch_ch_num - 1];
	raw_controls_out->out1 = sbus_frame_in->servo_channels[mix_params->out1_ch_num - 1];
}

static void ClipMixedControls(MixedControls_TypeDef *new_controls, MixedControls_TypeDef *clipped_out, ControlsClipParams_TypeDef *clip_params)
{
	uint16_t clipped_value;

	clipped_value = ClipValue(new_controls->rudder, clip_params->rudder_min, clip_params->rudder_center, clip_params->rudder_max, CLIP_CENTER_PROTECTION_INTERVAL_PERCENT);
	clipped_out->rudder = clipped_value;

	clipped_value = ClipValue(new_controls->throttle, clip_params->throttle_min, clip_params->throttle_center, clip_params->throttle_max, CLIP_CENTER_PROTECTION_INTERVAL_PERCENT);
	clipped_out->throttle = clipped_value;

	clipped_value = ClipValue(new_controls->cam_yaw, clip_params->cam_yaw_min, clip_params->cam_yaw_center, clip_params->cam_yaw_max, CLIP_CENTER_PROTECTION_INTERVAL_PERCENT);
	clipped_out->cam_yaw = clipped_value;

	clipped_value = ClipValue(new_controls->cam_pitch, clip_params->cam_pitch_min, clip_params->cam_pitch_center, clip_params->cam_pitch_max, CLIP_CENTER_PROTECTION_INTERVAL_PERCENT);
	clipped_out->cam_pitch = clipped_value;

	if (new_controls->out1 > clip_params->out1_hi_thr)
	{
		clipped_value = clip_params->out1_max;
	}
	else if (new_controls->out1 < clip_params->out1_lo_thr)
	{
		clipped_value = clip_params->out1_min;
	}
	else if (clipped_out->out1 > 0)
	{
		if (clipped_out->out1 > clip_params->out1_hi_thr) {
			clipped_value = clip_params->out1_max;
		}
		else if (clipped_out->out1 < clip_params->out1_lo_thr)
		{
			clipped_value = clip_params->out1_min;
		}
	}
	else {
		clipped_value = clip_params->out1_min;
	}
	clipped_out->out1 = clipped_value;
}


static uint16_t ClipValue(uint16_t x, uint16_t min, uint16_t cent, uint16_t max, uint16_t protect_percent)
{
	uint32_t i_delta = (max - min) * protect_percent / 100;
	uint32_t i_th_lo = cent - i_delta;
	uint32_t i_th_hi = cent + i_delta;

	uint32_t y;
	if (x > max) {

		y = max;

	} else if (x > i_th_hi) {

		y = cent + (x - i_th_hi) * (max - cent) / (max - i_th_hi);

	} else if (x < min) {

		y = min;

	}  else if (x < i_th_lo) {

		y = min + (x - min) * (cent - min) / (i_th_lo - min);

	} else {

		y = cent;

	}

	return (uint16_t) y;
}


static void UpdateCalibrationWithNewData(MixedControls_TypeDef *raw_controls)
{
	static uint32_t n_frames = 0;
	static ControlsClipParams_TypeDef clip_params = {0};

	static uint32_t last_log = 0;

	uint16_t n_bytes;
	uint32_t t_now = SYS_GetTick();

	uint8_t upd_cent;


	if ((raw_controls->rudder < ABS_MIN_CHANNEL_VALUE_FOR_CALIBRATION) || (raw_controls->rudder > ABS_MAX_CHANNEL_VALUE_FOR_CALIBRATION)) {
		return;
	} else if ((raw_controls->throttle < ABS_MIN_CHANNEL_VALUE_FOR_CALIBRATION) || (raw_controls->throttle > ABS_MAX_CHANNEL_VALUE_FOR_CALIBRATION)) {
		return;
	} else if ((raw_controls->cam_yaw < ABS_MIN_CHANNEL_VALUE_FOR_CALIBRATION) || (raw_controls->cam_yaw > ABS_MAX_CHANNEL_VALUE_FOR_CALIBRATION)) {
		return;
	} else if ((raw_controls->cam_pitch < ABS_MIN_CHANNEL_VALUE_FOR_CALIBRATION) || (raw_controls->cam_pitch > ABS_MAX_CHANNEL_VALUE_FOR_CALIBRATION)) {
		return;
	} else if ((raw_controls->out1 < ABS_MIN_CHANNEL_VALUE_FOR_CALIBRATION) || (raw_controls->out1 > ABS_MAX_CHANNEL_VALUE_FOR_CALIBRATION)) {
		return;
	}

	if (n_frames == 0) {

		clip_params.rudder_min = 	raw_controls->rudder;
		clip_params.rudder_center =	raw_controls->rudder;
		clip_params.rudder_max = 	raw_controls->rudder;
		clip_params.throttle_min = 		raw_controls->throttle;
		clip_params.throttle_center =	raw_controls->throttle;
		clip_params.throttle_max = 		raw_controls->throttle;
		clip_params.cam_yaw_min = 		raw_controls->cam_yaw;
		clip_params.cam_yaw_center =	raw_controls->cam_yaw;
		clip_params.cam_yaw_max = 		raw_controls->cam_yaw;
		clip_params.cam_pitch_min = 	raw_controls->cam_pitch;
		clip_params.cam_pitch_center =	raw_controls->cam_pitch;
		clip_params.cam_pitch_max = 	raw_controls->cam_pitch;
		clip_params.out1_min = 		raw_controls->out1;
		clip_params.out1_lo_thr =	raw_controls->out1;
		clip_params.out1_hi_thr =	raw_controls->out1;
		clip_params.out1_max = 		raw_controls->out1;

	} else {

		//---- Rudder ----
		upd_cent = 0;
		if (clip_params.rudder_min > raw_controls->rudder) {
			clip_params.rudder_min = raw_controls->rudder;
			upd_cent = 1;
		}
		if (clip_params.rudder_max < raw_controls->rudder) {
			clip_params.rudder_max = raw_controls->rudder;
			upd_cent = 1;
		}
		if (upd_cent) {
			clip_params.rudder_center = (clip_params.rudder_min + clip_params.rudder_max) >> 1;
		}

		//---- Throttle ----
		upd_cent = 0;
		if (clip_params.throttle_min > raw_controls->throttle) {
			clip_params.throttle_min = raw_controls->throttle;
			upd_cent = 1;
		}
		if (clip_params.throttle_max < raw_controls->throttle) {
			clip_params.throttle_max = raw_controls->throttle;
			upd_cent = 1;
		}
		if (upd_cent) {
			clip_params.throttle_center = (clip_params.throttle_min + clip_params.throttle_max) >> 1;
		}

		//---- Camera YAW ----
		upd_cent = 0;
		if (clip_params.cam_yaw_min > raw_controls->cam_yaw) {
			clip_params.cam_yaw_min = raw_controls->cam_yaw;
			upd_cent = 1;
		}
		if (clip_params.cam_yaw_max < raw_controls->cam_yaw) {
			clip_params.cam_yaw_max = raw_controls->cam_yaw;
			upd_cent = 1;
		}
		if (upd_cent) {
			clip_params.cam_yaw_center = (clip_params.cam_yaw_min + clip_params.cam_yaw_max) >> 1;
		}

		//---- Camera PITCH ----
		upd_cent = 0;
		if (clip_params.cam_pitch_min > raw_controls->cam_pitch) {
			clip_params.cam_pitch_min = raw_controls->cam_pitch;
			upd_cent = 1;
		}
		if (clip_params.cam_pitch_max < raw_controls->cam_pitch) {
			clip_params.cam_pitch_max = raw_controls->cam_pitch;
			upd_cent = 1;
		}
		if (upd_cent) {
			clip_params.cam_pitch_center = (clip_params.cam_pitch_min + clip_params.cam_pitch_max) >> 1;
		}


		//---- OUT1 ----
		upd_cent = 0;
		if (clip_params.out1_min > raw_controls->out1) {
			clip_params.out1_min = raw_controls->out1;
			upd_cent = 1;
		}
		if (clip_params.out1_max < raw_controls->out1) {
			clip_params.out1_max = raw_controls->out1;
			upd_cent = 1;
		}
		if (upd_cent) {
			uint16_t center = (clip_params.out1_min + clip_params.out1_max) >> 1;
			uint16_t delta  = (clip_params.out1_max + clip_params.out1_min) / 10;
			clip_params.out1_lo_thr = center - delta;
			clip_params.out1_hi_thr = center + delta;
		}
	}

	if ((last_log == 0) || ((t_now - last_log) >= 900)) {
		n_bytes = sprintf((char *)log_buffer, "throttle = %d,  rudder = %d, cam_yaw = %d, cam_pitch = %d\r\n", raw_controls->throttle, raw_controls->rudder, raw_controls->cam_yaw, raw_controls->cam_pitch);
		LOG_SendLog(log_buffer, 0, n_bytes);

		n_bytes = sprintf((char *)log_buffer, "TH: %d,\t%d,\t%d;\tRUD %d,\t%d,\t%d;\tYAW %d,\t%d,\t%d;\tPIT %d,\t%d,\t%d;\r\n\n\n",
				clip_params.throttle_min, clip_params.throttle_center, clip_params.throttle_max,
				clip_params.rudder_min, clip_params.rudder_center, clip_params.rudder_max,
				clip_params.cam_yaw_min, clip_params.cam_yaw_center, clip_params.cam_yaw_max,
				clip_params.cam_pitch_min, clip_params.cam_pitch_center, clip_params.cam_pitch_max);
		LOG_SendLog(log_buffer, 0, n_bytes);


		if (last_log > 0) {
			SaveControlClipParams(&clip_params);
		}
		last_log = t_now;
	}

	n_frames ++;
}






static uint8_t RestoreControlClipParams(ControlsClipParams_TypeDef *dst)
{
	uint16_t ee_result_code;

	ee_result_code = EE_ReadVariable(EE_ADDR_RUDDER_MIN, &(dst->rudder_min));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_RUDDER_CENT, &(dst->rudder_center));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_RUDDER_MAX, &(dst->rudder_max));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_THROT_MIN, &(dst->throttle_min));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_THROT_CENT, &(dst->throttle_center));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_THROT_MAX, &(dst->throttle_max));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_CAMYAW_MIN, &(dst->cam_yaw_min));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_CAMYAW_CENT, &(dst->cam_yaw_center));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_CAMYAW_MAX, &(dst->cam_yaw_max));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_CAMPIT_MIN, &(dst->cam_pitch_min));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_CAMPIT_CENT, &(dst->cam_pitch_center));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_CAMPIT_MAX, &(dst->cam_pitch_max));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_OUT1_MIN, &(dst->out1_min));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_OUT1_LOTH, &(dst->out1_lo_thr));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_OUT1_HITH, &(dst->out1_hi_thr));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	ee_result_code = EE_ReadVariable(EE_ADDR_OUT1_MAX, &(dst->out1_max));
	if (ee_result_code != EE_VAR_FOUND) return 0;
	return 1;
}




static void SaveControlClipParams(ControlsClipParams_TypeDef *clip_params)
{
	EE_WriteVariable(EE_ADDR_RUDDER_MIN, clip_params->rudder_min);
	EE_WriteVariable(EE_ADDR_RUDDER_CENT, clip_params->rudder_center);
	EE_WriteVariable(EE_ADDR_RUDDER_MAX, clip_params->rudder_max);
	EE_WriteVariable(EE_ADDR_THROT_MIN, clip_params->throttle_min);
	EE_WriteVariable(EE_ADDR_THROT_CENT, clip_params->throttle_center);
	EE_WriteVariable(EE_ADDR_THROT_MAX, clip_params->throttle_max);
	EE_WriteVariable(EE_ADDR_CAMYAW_MIN, clip_params->cam_yaw_min);
	EE_WriteVariable(EE_ADDR_CAMYAW_CENT, clip_params->cam_yaw_center);
	EE_WriteVariable(EE_ADDR_CAMYAW_MAX, clip_params->cam_yaw_max);
	EE_WriteVariable(EE_ADDR_CAMPIT_MIN, clip_params->cam_pitch_min);
	EE_WriteVariable(EE_ADDR_CAMPIT_CENT, clip_params->cam_pitch_center);
	EE_WriteVariable(EE_ADDR_CAMPIT_MAX, clip_params->cam_pitch_max);
	EE_WriteVariable(EE_ADDR_OUT1_MIN, clip_params->out1_min);
	EE_WriteVariable(EE_ADDR_OUT1_LOTH, clip_params->out1_lo_thr);
	EE_WriteVariable(EE_ADDR_OUT1_HITH, clip_params->out1_hi_thr);
	EE_WriteVariable(EE_ADDR_OUT1_MAX, clip_params->out1_max);
}


static void MapClippedControlsToAbstractInterval(MixedControls_TypeDef *src, ControlsClipParams_TypeDef *clip_params, AbstractControls_TypeDef *dst)
{
	dst->rudder = MapValueToAbstract(src->rudder, clip_params->rudder_min, clip_params->rudder_center, clip_params->rudder_max);
	dst->throttle = MapValueToAbstract(src->throttle, clip_params->throttle_min, clip_params->throttle_center, clip_params->throttle_max);
	dst->cam_yaw = MapValueToAbstract(src->cam_yaw, clip_params->cam_yaw_min, clip_params->cam_yaw_center, clip_params->cam_yaw_max);
	dst->cam_pitch = MapValueToAbstract(src->cam_pitch, clip_params->cam_pitch_min, clip_params->cam_pitch_center, clip_params->cam_pitch_max);
	dst->out1 = (src->out1 > clip_params->out1_hi_thr);
}

static int8_t MapValueToAbstract(uint16_t x, uint16_t min, uint16_t center, uint16_t max)
{
	int8_t y;
	if (x > max)
	{
		y = 100;
	}
	else if (x > center)
	{
		y = (x - center) * 100 / (max - center);
	}
	else if (x < min)
	{
		y = -100;
	}
	else if (x < center)
	{
		y = (x - min) * 100 / (center - min) - 100;
	}
	else
	{
		y = 0;
	}

	return y;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
