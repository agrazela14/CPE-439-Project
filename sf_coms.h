/*
 * sf_coms.h
 *
 *  Created on: Nov 2, 2016
 */

#ifndef SRC_SF_COMS_H_
#define SRC_SF_COMS_H_
#include "FreeRTOS.h"

void sf_uart_init();

void sf_iic_init();

void sf_uart_write(u8 *out, BaseType_t numBytes);


#endif /* SRC_SF_COMS_H_ */
