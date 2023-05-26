#include "ZS05.h"

#define WAIT_FOR_PIN(X)                                              \
	while ((ZS05_Sample_Input() == X) && (ZS05_Get_Time_us() < 300)) \
	{                                                                \
	};                                                               \
	ZS05_Reset_Time();

#define WAIT_FOR_PIN_NO_RST(X)                                       \
	while ((ZS05_Sample_Input() == X) && (ZS05_Get_Time_us() < 300)) \
	{                                                                \
	};

extern TIM_HandleTypeDef htim3;
uint8_t time_arr[50] = {
	0,
};
uint8_t time_arr_ctr = 0;
uint8_t tmp_data[5] = {
	0,
};

void ZS05_Time_Start()
{
	HAL_TIM_Base_Start(&htim3);
}

void ZS05_Time_Stop()
{
	HAL_TIM_Base_Stop(&htim3);
}

void ZS05_Reset_Time()
{
	__HAL_TIM_SET_COUNTER(&htim3, 0);
}

uint32_t ZS05_Get_Time_us()
{
	return (uint32_t)__HAL_TIM_GET_COUNTER(&htim3);
}

void ZS05_Delay_ms(uint32_t delay_time)
{
	HAL_Delay(delay_time);
}

uint8_t ZS05_Sample_Input()
{
	return (HAL_GPIO_ReadPin(ZS05_DATA_GPIO_Port, ZS05_DATA_Pin));
}

void ZS05_Set_Input()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = ZS05_DATA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(ZS05_DATA_GPIO_Port, &GPIO_InitStruct);
}

void ZS05_Set_Input_Interrupt()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = ZS05_DATA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(ZS05_DATA_GPIO_Port, &GPIO_InitStruct);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
}

void ZS05_Set_Output_PushPull()
{
	HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = ZS05_DATA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(ZS05_DATA_GPIO_Port, &GPIO_InitStruct);
	ZS05_Set_Output_Level(1);
}

void ZS05_Set_Output_Level(uint8_t level)
{
	HAL_GPIO_WritePin(ZS05_DATA_GPIO_Port, ZS05_DATA_Pin, level);
}

uint8_t ZS05_Get_Data()
{
	tmp_data[0] = 0;
	tmp_data[1] = 0;
	tmp_data[2] = 0;
	tmp_data[3] = 0;
	tmp_data[4] = 0;

	ZS05_Time_Start();
	ZS05_Set_Output_PushPull();

	ZS05_Set_Output_Level(1); // setting high
	ZS05_Delay_ms(20);		  // waiting 20ms
	ZS05_Set_Output_Level(0); // setting low
	ZS05_Delay_ms(20);		  // waiting 20ms

	ZS05_Set_Input(); // setting as input
	ZS05_Reset_Time();

	WAIT_FOR_PIN(1); // waiting for ZS05 to pull down the line
	WAIT_FOR_PIN(0); //~80us low ZS05 response signal
	WAIT_FOR_PIN(1); //~80us high ZS05 response signal

	// data starts now, 40 bits
	for (uint8_t i = 0; i < 40; i++)
	{
		WAIT_FOR_PIN(0);		// 50us low
		WAIT_FOR_PIN_NO_RST(1); // high for 27us = 0, high for 70us = 1
		uint16_t stop_time = ZS05_Get_Time_us();
		tmp_data[i / 8] |= ((stop_time > 60) ? 1 : 0) << (7 - (i % 8)); // decoding, using 35us as threshold
		time_arr[i] = stop_time;
	}

	ZS05_Time_Stop();
	ZS05_Set_Output_PushPull(); // setting to output
	ZS05_Set_Output_Level(1);	// setting line as high

	return HAL_OK;
}

uint8_t ZS05_Get_Data_Interrupt()
{
	time_arr_ctr = 0;

	tmp_data[0] = 0;
	tmp_data[1] = 0;
	tmp_data[2] = 0;
	tmp_data[3] = 0;
	tmp_data[4] = 0;

	ZS05_Time_Start();
	ZS05_Set_Output_PushPull();

	ZS05_Set_Output_Level(1); // setting high
	ZS05_Delay_ms(30);		  // waiting 20ms
	ZS05_Set_Output_Level(0); // setting low
	ZS05_Delay_ms(30);		  // waiting 20ms

	ZS05_Set_Input_Interrupt();
	while( __HAL_TIM_GET_COUNTER(&htim3) < 1000){};
	ZS05_Set_Output_PushPull();
	HAL_TIM_Base_Stop(&htim3);
	for(uint8_t i = 0; i < 40; i++)
	{
		tmp_data[i / 8] |= ((time_arr[i+2] > 90) ? 1 : 0) << (7 - (i % 8)); // decoding, using 35us as threshold
	}

	return HAL_OK;

}

uint8_t ZS05_Get_Temperature(ZS05_Data *zs05_data)
{
	// uint8_t status = ZS05_Get_Data();
	bzero(time_arr, sizeof(time_arr));
	bzero(tmp_data, sizeof(tmp_data));
	uint8_t status = ZS05_Get_Data_Interrupt();

	if ((tmp_data[4] == (uint8_t)(tmp_data[0] + tmp_data[1] + tmp_data[2] + tmp_data[3])) && (status == HAL_OK) && (tmp_data[0] != 0 && tmp_data[2] != 0))
	{
		zs05_data->humidity = (tmp_data[0]);
		zs05_data->temperature = (tmp_data[2]) * ((tmp_data[3] & (1 << 7)) ? (-1) : (1));
		status = HAL_OK;
	}
	else
	{
		zs05_data->humidity = 0;
		zs05_data->temperature = 0;
		status = HAL_ERROR;
	}

	return status;
}