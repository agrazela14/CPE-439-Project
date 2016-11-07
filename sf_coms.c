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
#include "sf_coms.h"

#define IIC_CLK_FREQ 100000
#define IMU_ADDR 0x28
#define EEPROM_ADDR 0x50

extern XScuGic xInterruptController;

/* UART_0 instance */
static XUartPs xuart;

/* IIC_1 instance */
static XIicPs xiic;

static int rec = 0;
static int snd = 0;

void IIC_Handler(void *callBackRef, u32 Event) {
    if ((Event & XIICPS_EVENT_COMPLETE_RECV) != 0) {
        rec = 1;
    }
    else if ((Event & XIICPS_EVENT_COMPLETE_SEND) != 0) {
        snd = 1;
    }
}

void sf_init_ints() {
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
            (Xil_InterruptHandler)(XScuGic_InterruptHandler),
            &xInterruptController);

    XScuGic_Connect(&xInterruptController, XPAR_XIICPS_0_INTR,
            (Xil_InterruptHandler) (XIicPs_MasterInterruptHandler),
            (void *)&xiic);

    XScuGic_Enable(&xInterruptController, XPAR_XIICPS_0_INTR);

    Xil_ExceptionEnable();

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

void sf_iic_send(u8 *out, BaseType_t numBytes) {

    XIicPs_MasterSend(&xiic, out, numBytes, IMU_ADDR);

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
    //XUartPs_SetBaudRate(&xuart, 115200);
    //Want to also put in interrupt mode later
}

void sf_uart_write(u8 *out, BaseType_t numBytes) {

    XUartPs_Send(&xuart, out, numBytes);

}
