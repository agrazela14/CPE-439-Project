/*
 * sf_coms.h
 *
 * Authors: Tristan Lennertz and Alex Grazela
 */

#ifndef SRC_SF_COMS_H_
#define SRC_SF_COMS_H_

#include "FreeRTOS.h"
#include "semphr.h"

#define IMU_ADDR 0x28
#define EEPROM_ADDR 0x50

void sf_init_coms();

void sf_iic_send(u8 *out, BaseType_t numBytes, u16 slaveAddr);

void sf_iic_receive(u8 *in, BaseType_t numBytes, u16 slaveAddr);

u32 sf_uart_send(u8 *out, BaseType_t numBytes);

void sf_uart_receive(u8 *in, BaseType_t numBytes);

extern SemaphoreHandle_t uartSendDone, uartRecDone;
extern SemaphoreHandle_t iicSendDone, iicRecDone;

#endif /* SRC_SF_COMS_H_ */
