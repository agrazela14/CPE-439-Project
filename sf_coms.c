/*
 * sf_coms.c
 *
 *  Created on: Nov 2, 2016
 *
 */

#include "FreeRTOS.h"
#include "xuartps.h"
#include "xscugic.h"
#include "xiicps.h"
#include "xsemaphr.h"
#include "sf_coms.h"

#define IIC_CLK_FREQ 100000
#define uartWriteBlockTime 100000

extern XScuGic xInterruptController;

/* UART_0 instance */
static XUartPs xuart;

/* IIC_1 instance */
static XIicPs xiic;

static xSemaphore uartSema;
static xSemaphore iicSema;

static int rec = 0;
static int snd = 0;

void IIC_Handler(void *callBackRef, u32 Event) {
    if ((Event & XIICPS_EVENT_COMPLETE_RECV) != 0) {
        rec = 1; //semaphore instead
    }
    else if ((Event & XIICPS_EVENT_COMPLETE_SEND) != 0) {
        snd = 1; //semaphore instead
    }
}

void UART_Handler(void *callBackRef, u32 Event) {
	/* All of the data has been sent */
	if (Event == XUARTPS_EVENT_SENT_DATA) {
		/* semaphore */
	}

	/* All of the data has been received */
	if (Event == XUARTPS_EVENT_RECV_DATA) {
		/* semaphore */
	}
}

void sf_init_ints() {
	/* Interrupt controller driver assumed to be initialized already */

	/*
	 * Connect the interrupt controller interrupt handler to the
	 * hardware interrupt handling logic in the processor.
	 */
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
            (Xil_InterruptHandler)(XScuGic_InterruptHandler),
            &xInterruptController);

    /* Connect IIC_1 device driver handler */
    XScuGic_Connect(&xInterruptController, XPAR_XIICPS_0_INTR,
            (Xil_InterruptHandler) (XIicPs_MasterInterruptHandler),
            (void *)&xiic);

    /* Connect UART_0 device driver handler */
    XScuGic_Connect(&xInterruptController, XPAR_XUARTPS_0_INTR,
            (Xil_InterruptHandler) (XUartPs_InterruptHandler),
            (void *)&xuart);

    /* Enable Interrupts for IIC_1 */
    XScuGic_Enable(&xInterruptController, XPAR_XIICPS_0_INTR);

    /* Enable Interrupts for UART_0 */
    XScuGic_Enable(&xInterruptController, XPAR_XUARTPS_0_INTR);

    Xil_ExceptionEnable();

    /* Set callback Handler for UART_0 */
    XUartPs_SetHandler(&xuart, UART_Handler, (void *)(&xuart));

    /* Set callback Handler for IIC_1 */
    XIicPs_SetStatusHandler(&xiic, (void *)(&xiic), IIC_Handler);
}

void sf_iic_init() {
    XIicPs_Config *pxConfigPtrIIC;
    BaseType_t xStatusIIC;

    //Initialize I2C
    pxConfigPtrIIC = XIicPs_LookupConfig(XPAR_PS7_I2C_1_DEVICE_ID);
    xStatusIIC = XIicPs_CfgInitialize(&xiic, pxConfigPtrIIC, pxConfigPtrIIC->BaseAddress);
    configASSERT( xStatusIIC == XST_SUCCESS );
    (void) xStatusIIC;

    XIicPs_SetSClk(&xiic, IIC_CLK_FREQ);
}

void sf_iic_send(u8 *out, BaseType_t numBytes, u16 slaveAddr) {

    XIicPs_MasterSend(&xiic, out, numBytes, slaveAddr);

}

void sf_uart_init() {
    XUartPs_Config *pxConfigPtrUART;
    BaseType_t xStatusUART;

    //Initialize UART
    pxConfigPtrUART = XUartPs_LookupConfig(XPAR_PS7_UART_0_DEVICE_ID);
    xStatusUART = XUartPs_CfgInitialize(&xuart, pxConfigPtrUART, pxConfigPtrUART->BaseAddress);
    configASSERT( xStatusUART == XST_SUCCESS );
    (void) xStatusUART;

    //set UART options
    XUartPs_SetBaudRate(&xuart, 9600);
    //Want to also put in interrupt mode later
}

//self note: Semaphore Macros:
// xSemaphoreTake(xSemaphore, xBlockTime) lowers it for a specified amount of time before going anyways (I think)
// xSemaphoreGive(xSemaphore) Ups the semaphore

u32 sf_uart_write(u8 *out, BaseType_t numBytes) {
    u32 bytesSent;
    xSemaphoreTake(uartSema, uartWriteBlockTime); 
    bytesSent = XUartPs_Send(&xuart, out, numBytes);
    return bytesSent;
}

u32 sf_uart_read(u8 *in, BaseType_t numBytes) {
    u32 bytesRecv = XUartPs_Recv(&xuart, in, numBytes);
    xSemaphoreGive(uartSema); 
    return bytesRecv;
}
