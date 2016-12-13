/*
 * sf_dma.h
 *
 * Authors: Tristan Lennertz and Alex Grazela
 */

#ifndef SF_DMA_H
#define SF_DMA_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "sf_coms.h"

/* The buffers for Transmitting and Receiving to the hardware accelerator.
 * Fill Tx up, start transceive (blocks), and then read Rx! */
extern float *sf_dma_TxBuffer;
extern float *sf_dma_RxBuffer;

/* Sizes of the buffers from the caller's perspective */
#define TX_BUFFER_LENGTH 50
#define RX_BUFFER_LENGTH 50

/* Transmit buffer to device through DMA, then receive buffer back through DMA
 * Transfers are always of size TX_BUFFER_LENGTH and RX_BUFFER_LENGTH.
 * XST_SUCCESS on success, XST_FAILURE otherwise. */
int sf_dma_transceive(void);

/* Initializes DMA engine. XST_SUCCESS on success, XST_FAILURE otherwise */
int sf_init_dma(void);

#endif
