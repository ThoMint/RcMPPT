/*
 * moving_average.h
 *
 *  Created on: 30.08.2021
 *      Author: Thomas
 */

#ifndef INC_MOVING_AVERAGE_H_
#define INC_MOVING_AVERAGE_H_

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdlib.h"

/* Definitions ---------------------------------------------------------------*/
//#define WindowLength 20

/* TypeDefs ------------------------------------------------------------------*/
typedef struct
{
	int WindowLength;
	int Sum; /* Sum of filter window's elements*/
	uint32_t WindowPointer; /* Pointer to the first element of window*/
	int *History; /*Array to store values of filter window*/
} FilterTypeDef;

/* Function prototypes -------------------------------------------------------*/
void Moving_Average_Init(FilterTypeDef *filter_struct, uint32_t size);
int Moving_Average_Compute(int raw_data, FilterTypeDef *filter_struct);

#endif /* INC_MOVING_AVERAGE_H_ */
