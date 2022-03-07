/**********************************************************************
*
* @file		crc.cpp
* @brief	Contains the definitions of crc function
* @version	v1.00f00
* @date		12. Dec. 2020
* @author	CECARELLI, Federico (fededc88@gmail.com)
*			MOYA, Martin		(moyamartin1@gmail.com)
*			SANTOS, Lucio		(lusho2206@gmail.com)
* @copyright GPL license, all text here must be included in any redistribution.
**********************************************************************/
#include "crcLUT.h"

uint8_t calculate_crc(uint8_t * buffer, size_t buffer_length, 
                      const uint8_t * crc_lut)
{
	uint8_t crc = 0, tmp = 0;
	for(size_t i = 0; i < buffer_length; ++i){
		tmp = (uint8_t) (crc ^ buffer[i]);
		crc = crc_lut[tmp];
	}
    return crc;
}
