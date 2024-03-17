/*
 * global.h
 *
 *  Created on: May 8, 2023
 *      Author: bart-
 */

#ifndef INC_GLOBAL_SETTERS_GETTERS_H_
#define INC_GLOBAL_SETTERS_GETTERS_H_

/* SOCKET DEFINES */
#define SOCKET_PORT 1883
#define SOCKET_HOST "192.168.1.164"
#define INIT_OK 1
#define INIT_NOK 0
#define DEBUGGING

extern uint8_t ETHstatus;
//extern SemaphoreHandle_t Mutex_ETHstate;
extern uint16_t ROTvalue;
//extern SemaphoreHandle_t Mutex_ROTvalue;

void SetETHstate(uint8_t status);
uint8_t GetETHstate();
void SetROTvalue(uint16_t value);
uint16_t GetROTvalue();

#endif /* INC_GLOBAL_SETTERS_GETTERS_H_ */
