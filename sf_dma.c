#include "xaxidma.h"
#include "xparameters.h"
#include "xil_exception.h"
#include "xdebug.h"
#include "xscugic.h"
#include "sf_dma.h"

/* Timeout loop counter for reset */
#define RESET_TIMEOUT_COUNTER	10000

/* Instance of the XAxiDma */
static XAxiDma AxiDma;

/* Zynq interrupt controller instance (for setup) */
extern XScuGic xInterruptController;

/* The buffers for Transmitting and Receiving to the hardware accelerator,
 * from the perspective of the caller. Fill Tx up, start transceive (blocks),
 * and then read Rx! This is actually an offset pointer from the real
 * buffer, read the comment over the offset declaration */
float *sf_dma_TxBuffer;
float *sf_dma_RxBuffer;

/* DMA works consistently, but has incorrect values for the first 7 float
 * transfers. We haven't had the time to resolve it, but we think it is an
 * AXI timing issue. This is reliable behavior, however, and we are working
 * around it by offsetting the start of the buffer that the caller uses
 * by 7 float, so that to them the world is a happy place absent of pain */
#define DMA_BUFFER_OFFSET 7

/* Array length and the number of bytes to transfer */
#define TX_ARRAY_LENGTH		(TX_BUFFER_LENGTH + DMA_BUFFER_OFFSET)
#define RX_ARRAY_LENGTH		(RX_BUFFER_LENGTH + DMA_BUFFER_OFFSET)

float real_tx_buffer[TX_ARRAY_LENGTH];
float real_rx_buffer[RX_ARRAY_LENGTH];

#define BYTES_TO_TRANSFER_TX	(4 * TX_ARRAY_LENGTH)
#define BYTES_TO_TRANSFER_RX	(4 * RX_ARRAY_LENGTH)

/*  Device hardware build related constants. */
#define DMA_DEV_ID		XPAR_AXIDMA_0_DEVICE_ID

#define RX_INTR_ID		XPAR_FABRIC_AXI_DMA_0_S2MM_INTROUT_INTR
#define TX_INTR_ID		XPAR_FABRIC_AXI_DMA_0_MM2S_INTROUT_INTR

#define INTC_DEVICE_ID  XPAR_SCUGIC_SINGLE_DEVICE_ID

#define INTC			XScuGic
#define INTC_HANDLER	XScuGic_InterruptHandler

/* Flags interrupt handlers use to notify the application context the events. */
volatile int TxDone;
volatile int RxDone;
volatile int Error;

/* Internal Function prototypes */
static void TxIntrHandler(void *Callback);
static void RxIntrHandler(void *Callback);

static int SetupIntrSystem(INTC * IntcInstancePtr,
			   XAxiDma * AxiDmaPtr, u16 TxIntrId, u16 RxIntrId);
static void DisableIntrSystem(INTC * IntcInstancePtr,
					u16 TxIntrId, u16 RxIntrId);

/* Initializes DMA engine. XST_SUCCESS on success, XST_FAILURE otherwise */
int sf_init_dma(void) {
	/* apply buffer offset */
	sf_dma_TxBuffer = real_tx_buffer + DMA_BUFFER_OFFSET;
	sf_dma_RxBuffer = real_rx_buffer + DMA_BUFFER_OFFSET;

	int Status;
	XAxiDma_Config *Config;

	Config = XAxiDma_LookupConfig(DMA_DEV_ID);
	if (!Config) {
		return XST_FAILURE;
	}

	/* Initialize DMA engine */
	Status = XAxiDma_CfgInitialize(&AxiDma, Config);

	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	if(XAxiDma_HasSg(&AxiDma)){
		return XST_FAILURE;
	}

	/* Set up Interrupt system  */
	Status = SetupIntrSystem(&xInterruptController, &AxiDma, TX_INTR_ID, RX_INTR_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Disable all interrupts before setup */

	XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK,
						XAXIDMA_DMA_TO_DEVICE);

	XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK,
				XAXIDMA_DEVICE_TO_DMA);

	/* Enable all interrupts */
	XAxiDma_IntrEnable(&AxiDma, XAXIDMA_IRQ_ALL_MASK,
							XAXIDMA_DMA_TO_DEVICE);


	XAxiDma_IntrEnable(&AxiDma, XAXIDMA_IRQ_ALL_MASK,
							XAXIDMA_DEVICE_TO_DMA);

	return XST_SUCCESS;
}


int sf_dma_transceive() {
	int Status;

	/* Initialize flags before start transfer */
	TxDone = 0;
	RxDone = 0;
	Error = 0;

	/* Flush the SrcBuffer before the DMA transfer, in case the Data Cache is enabled */
	Xil_DCacheFlushRange((u32)real_tx_buffer, BYTES_TO_TRANSFER_TX);

	Status = XAxiDma_SimpleTransfer(&AxiDma,(u32) real_tx_buffer, BYTES_TO_TRANSFER_TX, XAXIDMA_DMA_TO_DEVICE);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Status = XAxiDma_SimpleTransfer(&AxiDma,(u32) real_rx_buffer, BYTES_TO_TRANSFER_RX, XAXIDMA_DEVICE_TO_DMA);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Wait TX done and RX done */
	while (!TxDone && !RxDone && !Error) { /* NOP */ }

	if (Error) {
		return XST_FAILURE;
	}

	/* Invalidate the DestBuffer before checking the data, in case the Data Cache is enabled */
	Xil_DCacheInvalidateRange((u32)real_rx_buffer, BYTES_TO_TRANSFER_RX);

	return XST_SUCCESS;
}

/* Tx interrupt handler function */
static void TxIntrHandler(void *Callback)
{
	u32 IrqStatus;
	int TimeOut;
	XAxiDma *AxiDmaInst = (XAxiDma *)Callback;

	/* Read pending interrupts */
	IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DMA_TO_DEVICE);

	/* Acknowledge pending interrupts */


	XAxiDma_IntrAckIrq(AxiDmaInst, IrqStatus, XAXIDMA_DMA_TO_DEVICE);

	/*
	 * If no interrupt is asserted, we do not do anything
	 */
	if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {

		return;
	}

	/*
	 * If error interrupt is asserted, raise error flag, reset the
	 * hardware to recover from the error, and return with no further
	 * processing.
	 */
	if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK)) {

		Error = 1;

		/*
		 * Reset should never fail for transmit channel
		 */
		XAxiDma_Reset(AxiDmaInst);

		TimeOut = RESET_TIMEOUT_COUNTER;

		while (TimeOut) {
			if (XAxiDma_ResetIsDone(AxiDmaInst)) {
				break;
			}

			TimeOut -= 1;
		}

		return;
	}

	/*
	 * If Completion interrupt is asserted, then set the TxDone flag
	 */
	if ((IrqStatus & XAXIDMA_IRQ_IOC_MASK)) {

		TxDone = 1;
	}
}

/* Rx interrupt handler function */
static void RxIntrHandler(void *Callback)
{
	u32 IrqStatus;
	int TimeOut;
	XAxiDma *AxiDmaInst = (XAxiDma *)Callback;

	/* Read pending interrupts */
	IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DEVICE_TO_DMA);

	/* Acknowledge pending interrupts */
	XAxiDma_IntrAckIrq(AxiDmaInst, IrqStatus, XAXIDMA_DEVICE_TO_DMA);

	/*
	 * If no interrupt is asserted, we do not do anything
	 */
	if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
		return;
	}

	/*
	 * If error interrupt is asserted, raise error flag, reset the
	 * hardware to recover from the error, and return with no further
	 * processing.
	 */
	if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK)) {

		Error = 1;

		/* Reset could fail and hang
		 * NEED a way to handle this or do not call it??
		 */
		XAxiDma_Reset(AxiDmaInst);

		TimeOut = RESET_TIMEOUT_COUNTER;

		while (TimeOut) {
			if(XAxiDma_ResetIsDone(AxiDmaInst)) {
				break;
			}

			TimeOut -= 1;
		}

		return;
	}

	/*
	 * If completion interrupt is asserted, then set RxDone flag
	 */
	if ((IrqStatus & XAXIDMA_IRQ_IOC_MASK)) {

		RxDone = 1;
	}
}


/* This function setups the interrupt system so interrupts can occur for the
 * DMA, it assumes INTC component exists in the hardware system. */
static int SetupIntrSystem(INTC * IntcInstancePtr,
			   XAxiDma * AxiDmaPtr, u16 TxIntrId, u16 RxIntrId)
{
	int Status;

	XScuGic_SetPriorityTriggerType(IntcInstancePtr, TxIntrId, 0xA0, 0x3);

	XScuGic_SetPriorityTriggerType(IntcInstancePtr, RxIntrId, 0xA0, 0x3);

	 /* Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device. */
	Status = XScuGic_Connect(IntcInstancePtr, TxIntrId,
				(Xil_InterruptHandler)TxIntrHandler,
				AxiDmaPtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	Status = XScuGic_Connect(IntcInstancePtr, RxIntrId,
				(Xil_InterruptHandler)RxIntrHandler,
				AxiDmaPtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	XScuGic_Enable(IntcInstancePtr, TxIntrId);
	XScuGic_Enable(IntcInstancePtr, RxIntrId);

	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

/* This function disables the interrupts for DMA engine. */
static void DisableIntrSystem(INTC * IntcInstancePtr,
					u16 TxIntrId, u16 RxIntrId)
{
#ifdef XPAR_INTC_0_DEVICE_ID
	/* Disconnect the interrupts for the DMA TX and RX channels */
	XIntc_Disconnect(IntcInstancePtr, TxIntrId);
	XIntc_Disconnect(IntcInstancePtr, RxIntrId);
#else
	XScuGic_Disconnect(IntcInstancePtr, TxIntrId);
	XScuGic_Disconnect(IntcInstancePtr, RxIntrId);
#endif
}
