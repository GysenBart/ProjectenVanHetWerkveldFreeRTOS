/*
 * global.c
 *
 *  Created on: May 8, 2023
 *      Author: bart-
 */


#include <FreeRTOS.h>
#include <global_setters_getters.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>

#include "main.h"




uint8_t ETHstatus;
extern SemaphoreHandle_t Mutex_ETHstate;
//extern osMutexId_t Mutex_ETHstateHandle;

uint16_t ROTvalue;
extern SemaphoreHandle_t Mutex_ROTvalue;
//extern osMutexId_t Mutex_ROTvalueHandle;

extern uint16_t Rot_cnt;

void SetETHstate(uint8_t status)
{
	if (xSemaphoreTake(Mutex_ETHstate, 250))
	{
		ETHstatus = status;
		xSemaphoreGive(Mutex_ETHstate);
	}
}

uint8_t GetETHstate()
{
	uint8_t status;
	if (xSemaphoreTake(Mutex_ETHstate, 250))
	{
		status = ETHstatus;
		xSemaphoreGive(Mutex_ETHstate);
	}
	return status;
}

void SetROTvalue(uint16_t value)
{
	if (xSemaphoreTake(Mutex_ROTvalue, 250))
	{
		ROTvalue = value;
		xSemaphoreGive(Mutex_ROTvalue);
	}
}

uint16_t GetROTvalue()
{
	uint16_t value;
	if (xSemaphoreTake(Mutex_ROTvalue, 250))
	{
		value = ROTvalue;
		xSemaphoreGive(Mutex_ROTvalue);
	}
	return value;
}

