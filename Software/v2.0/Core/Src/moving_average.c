/*
 * moving_average.c
 *
 *  Created on: 30.08.2021
 *      Author: Thomas
 */

/* Includes ------------------------------------------------------------------*/
#include "moving_average.h"

/**
 * @brief  This function initializes filter's data structure.
 * @param  filter_struct : Data structure
 * @retval None.
 */
void Moving_Average_Init(FilterTypeDef *filter_struct, uint32_t size)
{
	filter_struct->Sum = 0;
	filter_struct->WindowPointer = 0;
	filter_struct->WindowLength = size;

	filter_struct->History = calloc(filter_struct->WindowLength, sizeof(int));

	for (uint32_t i = 0; i < filter_struct->WindowLength; i++)
	{
		filter_struct->History[i] = 0;
	}
}

/**
 * @brief  This function filters data with moving average filter.
 * @param  raw_data : input raw sensor data.
 * @param  filter_struct : Data structure
 * @retval Filtered value.
 */
int Moving_Average_Compute(int raw_data, FilterTypeDef *filter_struct)
{
	filter_struct->Sum += raw_data;
	filter_struct->Sum -= filter_struct->History[filter_struct->WindowPointer];
	filter_struct->History[filter_struct->WindowPointer] = raw_data;
	if (filter_struct->WindowPointer < filter_struct->WindowLength - 1)
	{
		filter_struct->WindowPointer += 1;
	}
	else
	{
		filter_struct->WindowPointer = 0;
	}
	return filter_struct->Sum / filter_struct->WindowLength;
}

