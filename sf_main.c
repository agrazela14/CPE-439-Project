/*
 * sf_main.c
 *
 * Authors: Tristan Lennertz and Alex Grazela
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* C lib includes */
#include <string.h>
#include <stdio.h>

/* Sensor Fusion Project Includes */
#include "sf_coms.h"
#include "sf_imu.h"
#include "sf_sdcard.h"
#include "sf_gps.h"
#include "sf_dma.h"

/* temporary serial include */
#include "serial.h"

#define RECEIVE_BUFFER_SIZE 256
#define SENTENCE_NAM_LEN 6
#define SD_QUEUE_SIZE 16

/* Priorities at which the tasks are created. */
#define		mainTASK_PRIORITY_SD			( tskIDLE_PRIORITY + 4 )
#define 	mainTASK_PRIORITY_GPS			( tskIDLE_PRIORITY + 2 )
#define		mainTASK_PRIORITY_IMU			( tskIDLE_PRIORITY + 3 )


/* Quantum of time to fetch measurements from IMU */
#define mainIMU_FETCH_FREQ			( 1000 / portTICK_PERIOD_MS )

/* Time to wait between switching different modes in IMU */
#define mainIMU_SWITCH_TIME			( 19 / portTICK_PERIOD_MS )

/* Task Forward Declarations */
void vGPSReceiveTask(void *pvParameters);
void vIMUFetchTask(void *pvParameters);
void vSDWriteTask(void *pvParameters);

/* Queue for writing finished GPS points out to SD card */
static QueueHandle_t SDWriteQueue = NULL;

void sf_main(void) {
	/* initialize instances of devices and their interrupt Handlers */
	sf_init_coms();

	/* Configure IMU */
	sf_imu_init();

	/* Inititialize SD card & its receive queue
	 * Commented out because it was stalling program initially */
	/*int Res = sf_init_sdcard();
	if(Res)
		vSerialPutString(NULL, (signed char *)"Failed to open SD card\r\n", strlen("Failed to open SD card\r\n"));
	else
		vSerialPutString(NULL, (signed char *)"SD card opened\r\n", strlen("SD card opened\r\n"));

	SDWriteQueue = xQueueCreate( SD_QUEUE_SIZE , sizeof( gps_t )); */

	xTaskCreate( vGPSReceiveTask,					// The function that implements the task.
				"GPS Parse", 						// The text name assigned to the task - for debug only as it is not used by the kernel.
				4096, 								// The size of the stack to allocate to the task.
				NULL, 								// The parameter passed to the task - not used in this case.
				mainTASK_PRIORITY_GPS, 				// The priority assigned to the task.
				NULL );								// The task handle is not required, so NULL is passed.

	xTaskCreate( vIMUFetchTask,						// The function that implements the task.
				"IMU Fetch", 						// The text name assigned to the task - for debug only as it is not used by the kernel.
				4096, 								// The size of the stack to allocate to the task.
				NULL, 								// The parameter passed to the task - not used in this case.
				mainTASK_PRIORITY_IMU, 				// The priority assigned to the task.
				NULL );								// The task handle is not required, so NULL is passed.

	xTaskCreate( vSDWriteTask,						// The function that implements the task.
				"SD Write", 						// The text name assigned to the task - for debug only as it is not used by the kernel.
				4096, 								// The size of the stack to allocate to the task.
				NULL, 								// The parameter passed to the task - not used in this case.
				mainTASK_PRIORITY_SD, 				// The priority assigned to the task.
				NULL );								// The task handle is not required, so NULL is passed.

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was either insufficient FreeRTOS heap memory available for the idle
	and/or timer tasks to be created, or vTaskStartScheduler() was called from
	User mode.  See the memory management section on the FreeRTOS web site for
	more details on the FreeRTOS heap http://www.freertos.org/a00111.html.  The
	mode from which main() is called is set in the C start up code and must be
	a privileged mode (not user mode). */
	for( ;; )
		;
}

void vSDWriteTask(void *pvParameters) {
	char dataPrintBuff[256];
	(void) pvParameters;
	//gps_t toWrite;

	for(;;) {
		int whole, dec;

		/* Announce TX */
		vSerialPutString(NULL, (signed char *)"=\r\n", strlen("=\r\n"));

		int i;
		for (i = 0; i < TX_BUFFER_LENGTH; i++) {
			sf_dma_TxBuffer[i] = i * 1.22;
			whole = sf_dma_TxBuffer[i];
			dec = ((sf_dma_TxBuffer[i] - whole) * 1000);
			snprintf(dataPrintBuff, 256, "Tx[%d]= %d.%d\r\n", i, whole, dec);
			vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));
		}

		for (i = 0; i < RX_BUFFER_LENGTH; i++)
			sf_dma_RxBuffer[i] = 0;

		sf_dma_transmit();

		/* Announce RX */
		for (i = 0; i < RX_BUFFER_LENGTH; i++) {
			whole = sf_dma_RxBuffer[i];
			dec = ((sf_dma_RxBuffer[i] - whole) * 1000);
			snprintf(dataPrintBuff, 256, "Rx[%d]= %d.%d\r\n", i, whole, dec);
			vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));
		}

		/* Block until something is received in queue to write out to file */
		//xQueueReceive( SDWriteQueue, &toWrite, portMAX_DELAY );
	}
}

/* right now just full of test stuff */
void vIMUFetchTask(void *pvParameters) {
	(void) pvParameters;
	char dataPrintBuff[256];

	/* Initialise xNextWakeTime - this only needs to be done once. */
	TickType_t xNextWakeTime = xTaskGetTickCount();

	/* make sure the IMU has had enough time to switch out of config mode */
	vTaskDelayUntil( &xNextWakeTime, mainIMU_SWITCH_TIME);

	for( ;; )
	{
		/* This quantum of delay time will be a core of the dead reckoning algorithm */
		vTaskDelayUntil( &xNextWakeTime, mainIMU_FETCH_FREQ );

		u32 data = sf_imu_get_calib();

		snprintf(dataPrintBuff, 256, "Calibration Status: %X\r\n", (int)data);
		vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));

		data = sf_imu_get_heading();

		snprintf(dataPrintBuff, 256, "Raw Heading: %d\r\n", (int)data);
		vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));

		data = sf_imu_get_acc_z();

		snprintf(dataPrintBuff, 256, "Z: %d\r\n", (int)data);
		vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));

		data = sf_imu_get_acc_y();

		snprintf(dataPrintBuff, 256, "Y: %d\r\n", (int)data);
		vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));

		data = sf_imu_get_acc_x();

		snprintf(dataPrintBuff, 256, "X: %d\r\n", (int)data);
		vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));
	}
}

/* right now just parses out and then prints something when valid */
void vGPSReceiveTask(void *pvParameters) {
	(void) pvParameters; //to eliminate compiler warnings for nonuse
	char recBuff[RECEIVE_BUFFER_SIZE], *buffPtr;
	buffPtr = recBuff;

	for(;;) {
		sf_uart_receive(buffPtr, 1); /* Start interrupt-driven receive (does not poll wait) */
		//xSemaphoreTake(uartRecDone, portMAX_DELAY); /* block until something received */

		/* Check for NMEA sentence start delimiter */
		if (*buffPtr++ == '$') {
			sf_uart_receive(buffPtr, SENTENCE_NAM_LEN); 	/* Start another receive for next part of */
			//xSemaphoreTake(uartRecDone, portMAX_DELAY);	/* sentence, block until done */

			/* Check for correct sentence type and validity */
			if (!strncmp(buffPtr, "GPRMC,", SENTENCE_NAM_LEN)) {
				buffPtr += SENTENCE_NAM_LEN;

				/* Run through timestamp field */
				do {
					sf_uart_receive(buffPtr, 1);
				} while (*buffPtr++ != ',');

				/* Collect Validity bit char and check */
				sf_uart_receive(buffPtr, 2);

				if (*buffPtr == 'A' || buffPtr == 'V') { /* 'A' is valid, 'V' is not */
					buffPtr += 2;
					char *fieldStart = buffPtr;
					int fieldLength = 0;

					/* Receive the Latitude field */
					do {
						sf_uart_receive(buffPtr, 1);
						fieldLength++;
					} while (*buffPtr++ != ',');

					/* Receive the N/S value of latitude */
					sf_uart_receive(buffPtr, 2);
					buffPtr += 2;

					/* Receive the longitude */
					do {
						sf_uart_receive(buffPtr, 1);
						fieldLength++;
					} while (*buffPtr++ != ',');

					/* Receive the E/W value of longitude */
					sf_uart_receive(buffPtr, 2);
					buffPtr += 2;

					/* Receive speed in Knots */
					do {
						sf_uart_receive(buffPtr, 1);
						fieldLength++;
					} while (*buffPtr++ != ',');

					/* Receive true course */
					do {
						sf_uart_receive(buffPtr, 1);
						fieldLength++;
					} while (*buffPtr++ != ',');

					/* Receive date stamp */
					do {
						sf_uart_receive(buffPtr, 1);
						fieldLength++;
					} while (*buffPtr++ != ',');

					/* Receive variation */
					do {
						sf_uart_receive(buffPtr, 1);
						fieldLength++;
					} while (*buffPtr++ != ',');

					/* Receive the E/W value */
					sf_uart_receive(buffPtr, 2);
					buffPtr += 2;

					/* Receive checksum */
					sf_uart_receive(buffPtr, 2);
					buffPtr += 2;

					/* Receive Ending characters */
					sf_uart_receive(buffPtr, 2);
					buffPtr += 2;

					/* terminate string */
					*buffPtr++ = '\n';

					*buffPtr = NULL;

					vSerialPutString(NULL, (signed char *)recBuff, strlen(recBuff));
				}
			}
		}

		buffPtr = recBuff;
	}
}
