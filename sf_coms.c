/*
 * sf_coms.c
 *
 * Authors: Tristan Lennertz and Alex Grazela
 */

#include "FreeRTOS.h"
#include "xuartps.h"
#include "xscugic.h"
#include "xiicps.h"
#include "semphr.h"
#include "sf_coms.h"

#define IIC_CLK_FREQ 100000
#define RX_QUEUE_LEN 1024

/* Interrupt Controller Initialized in main */
extern XScuGic xInterruptController;

/* UART_0 instance */
static XUartPs xuart;

/* IIC_1 instance */
static XIicPs xiic;

/* External indicators that a transmission or reception has finished.
 * This is separate from the internal indicators, which implement blocking
 * within the send/receive functions to protect against multiple reads
 * and writes too quickly. These allow caller to block on a call without making
 * another call.
 */
SemaphoreHandle_t uartSendDone, uartRecDone;
SemaphoreHandle_t iicSendDone, iicRecDone;

/* For receiving constant stream of data from GPS */
static QueueHandle_t xuartRxQueue = NULL;

/* Internal indicators to stop multiple reads and writes from ruining each other */
static SemaphoreHandle_t uartSemaSend, uartSemaRec;
static SemaphoreHandle_t iicSemaSend, iicSemaRec;

/* forward declarations for this file */
void sf_init_ints();
void sf_init_iic();
void sf_init_uart();

void IIC_Handler(void *callBackRef, u32 Event) {
	(void) callBackRef; //Removes compiler warning for not using callBackRef
	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* Receive has been completed */
    if ((Event & XIICPS_EVENT_COMPLETE_RECV) != 0) {
        xSemaphoreGiveFromISR(iicRecDone, &xHigherPriorityTaskWoken);
        xSemaphoreGiveFromISR(iicSemaRec, &xHigherPriorityTaskWoken);
    }
    else if ((Event & XIICPS_EVENT_COMPLETE_SEND) != 0) {
        xSemaphoreGiveFromISR(iicSendDone, &xHigherPriorityTaskWoken);
        xSemaphoreGiveFromISR(iicSemaSend, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void UART_Handler(void *callBackRef, u32 Event, u32 EventData) {
	(void) callBackRef; //Removes compiler warning for not using callBackRef
	(void) EventData; 	//... or EventData

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	uint32_t ulChannelStatusRegister;
	char cChar;

	/* All of the data has been sent */
	if ((Event == XUARTPS_EVENT_SENT_DATA)) {
		xSemaphoreGiveFromISR(uartSendDone, &xHigherPriorityTaskWoken);
        xSemaphoreGiveFromISR(uartSemaSend, &xHigherPriorityTaskWoken);
	}

	/* Data has been received */
	if ((Event == XUARTPS_EVENT_RECV_DATA)  | (Event == XUARTPS_EVENT_RECV_TOUT) ) {
		/* Read the Channel Status Register to determine if there is any data in
		the RX FIFO. */
		ulChannelStatusRegister = XUartPs_ReadReg( XPAR_PS7_UART_0_BASEADDR, XUARTPS_SR_OFFSET );

		/* Move data from the Rx FIFO to the Rx queue.  NOTE THE COMMENTS AT THE
		TOP OF THIS FILE ABOUT USING QUEUES FOR THIS PURPSOE. */
		while( ( ulChannelStatusRegister & XUARTPS_SR_RXEMPTY ) == 0 )
		{
			cChar =	XUartPs_ReadReg( XPAR_PS7_UART_0_BASEADDR, XUARTPS_FIFO_OFFSET );

			/* If writing to the queue unblocks a task, and the unblocked task
			has a priority above the currently running task (the task that this
			interrupt interrupted), then xHigherPriorityTaskWoken will be set
			to pdTRUE inside the xQueueSendFromISR() function.
			xHigherPriorityTaskWoken is then passed to portYIELD_FROM_ISR() at
			the end of this interrupt handler to request a context switch so the
			interrupt returns directly to the (higher priority) unblocked
			task. */
			xQueueSendFromISR( xuartRxQueue, &cChar, &xHigherPriorityTaskWoken );
			ulChannelStatusRegister = XUartPs_ReadReg( XPAR_PS7_UART_0_BASEADDR, XUARTPS_SR_OFFSET );
		}
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void sf_init_coms() {
	sf_init_iic();
	sf_init_uart();
	sf_init_ints();
}

void sf_init_ints() {
	/* Interrupt controller driver assumed to be initialized already */

    /* Connect IIC_1 device driver handler */
    XScuGic_Connect(&xInterruptController, XPAR_XIICPS_1_INTR,
            (Xil_InterruptHandler) (XIicPs_MasterInterruptHandler),
            (void *)&xiic);

    /* Connect UART_0 device driver handler */
    XScuGic_Connect(&xInterruptController, XPAR_XUARTPS_0_INTR,
            (Xil_InterruptHandler) (XUartPs_InterruptHandler),
            (void *)&xuart);

    /* Enable Interrupts for IIC_1 */
    XScuGic_Enable(&xInterruptController, XPAR_XIICPS_1_INTR);

    /* Enable Interrupts for UART_0 */
    XScuGic_Enable(&xInterruptController, XPAR_XUARTPS_0_INTR);

    /* Set callback Handler for UART_0 */
    XUartPs_SetHandler(&xuart, UART_Handler, (void *)(&xuart));

    /* Set callback Handler for IIC_1 */
    XIicPs_SetStatusHandler(&xiic, (void *)(&xiic), IIC_Handler);

    /* Interrupts to enable for UART_0
     * Only RX trigger interrupt is cared about here */
	u32 UARTIntrMask =
			XUARTPS_IXR_TOUT |
			XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_RXFULL |
			XUARTPS_IXR_RXOVR;


    XUartPs_SetInterruptMask(&xuart, UARTIntrMask);
}

void sf_init_iic() {
    XIicPs_Config *pxConfigPtrIIC;
    BaseType_t xStatusIIC;

    //Initialize I2C
    pxConfigPtrIIC = XIicPs_LookupConfig(XPAR_PS7_I2C_1_DEVICE_ID);
    xStatusIIC = XIicPs_CfgInitialize(&xiic, pxConfigPtrIIC, pxConfigPtrIIC->BaseAddress);
    configASSERT( xStatusIIC == XST_SUCCESS );
    (void) xStatusIIC;

    XIicPs_SetSClk(&xiic, IIC_CLK_FREQ);

    //Initialize semaphores to indicate the end of sending and receiving
    iicSemaSend = xSemaphoreCreateBinary();
    iicSemaRec = xSemaphoreCreateBinary();
    iicSendDone = xSemaphoreCreateBinary();
    iicRecDone = xSemaphoreCreateBinary();

    //All inititialized to 'taken', so give at the start
    xSemaphoreGive(iicSemaSend);
    xSemaphoreGive(iicSemaRec);
    xSemaphoreGive(iicSendDone);
    xSemaphoreGive(iicRecDone);
}

void sf_init_uart() {
	xuartRxQueue = xQueueCreate( RX_QUEUE_LEN , sizeof( char ) );

    XUartPs_Config *pxConfigPtrUART;
    BaseType_t xStatusUART;

    //Initialize UART
    pxConfigPtrUART = XUartPs_LookupConfig(XPAR_PS7_UART_0_DEVICE_ID);
    xStatusUART = XUartPs_CfgInitialize(&xuart, pxConfigPtrUART, pxConfigPtrUART->BaseAddress);
    configASSERT( xStatusUART == XST_SUCCESS );
    (void) xStatusUART;

    //set UART options
    XUartPs_SetBaudRate(&xuart, 9600);

    //Initialize semaphores to indicate the end of sending and receiving
    uartSemaSend = xSemaphoreCreateBinary();
    uartSemaRec = xSemaphoreCreateBinary();
    uartSendDone = xSemaphoreCreateBinary();
    uartRecDone = xSemaphoreCreateBinary();

    //All inititialized to 'taken', so give at the start
    xSemaphoreGive(uartSemaSend);
    xSemaphoreGive(uartSemaRec);
    xSemaphoreGive(uartSendDone);
    xSemaphoreGive(uartRecDone);
}

/* Will initialize a send of numBytes of data from *out buffer, to
 * device with address slaveAddr. This will indicate the end of
 * transmission by giving the iicSemaSend BinarySemaphore back
 * (it is taken before starting the transmission).
 *
 * If a previous send is
 */
void sf_iic_send(u8 *out, BaseType_t numBytes, u16 slaveAddr) {
	if(uxSemaphoreGetCount(iicSendDone)) //make sure external indicator cleared
		xSemaphoreTake(iicSendDone, portMAX_DELAY); //shouldn't ever need to block...

    XIicPs_MasterSend(&xiic, out, numBytes, slaveAddr);
}

void sf_iic_receive(u8 *in, BaseType_t numBytes, u16 slaveAddr) {
	if(uxSemaphoreGetCount(iicRecDone)) //make sure external indicator cleared
		xSemaphoreTake(iicRecDone, portMAX_DELAY); //shouldn't ever need to block...

	XIicPs_MasterRecv(&xiic, in, numBytes, slaveAddr);
}

u32 sf_uart_send(u8 *out, BaseType_t numBytes) {
	xSemaphoreTake(uartSemaSend, portMAX_DELAY); //block on previous call to finish

	if(uxSemaphoreGetCount(uartSendDone)) //make sure external indicator cleared
		xSemaphoreTake(uartSendDone, 1); //shouldn't ever need to block...

    u32 bytesSent = XUartPs_Send(&xuart, out, numBytes);
    return bytesSent;
}

void sf_uart_receive(u8 *in, BaseType_t numBytes) {
	int i;
	for (i = numBytes; i > 0; i--)
		xQueueReceive( xuartRxQueue, in + (numBytes - i), portMAX_DELAY );
}
