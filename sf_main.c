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

/* Sensor Fusion Project Includes */
#include "sf_coms.h"

/* temporary serial include */
#include "serial.h"

#define RECEIVE_BUFFER_SIZE 255
#define SENTENCE_NAM_LEN 6

/* The following defines are there for convenience, and should be modified
 * to fit desired specifics of functionality
 */

/* Priorities at which the tasks are created. */
#define 	mainTASK_PRIORITY_GPS			( tskIDLE_PRIORITY + 1 )
//#define	mainTASK_PRIORITY_2			( tskIDLE_PRIORITY + 2 )

/* How to define time from ticks */
//#define mainFREQUENCY_MS_1			( 420 / portTICK_PERIOD_MS )

void sf_main(void) {
	vSerialPutString(NULL, (signed char *)"Hello, starting up...\n", 22);

	/* initialize instances of devices and their interrupt Handlers */
	sf_init_coms();

	xTaskCreate( vGPSReceiveTask,					/* The function that implements the task. */
				"GPS Receive and Parse", 			/* The text name assigned to the task - for debug only as it is not used by the kernel. */
				configMINIMAL_STACK_SIZE, 			/* The size of the stack to allocate to the task. */
				NULL, 								/* The parameter passed to the task - not used in this case. */
				mainTASK_PRIORITY_GPS, 				/* The priority assigned to the task. */
				NULL );								/* The task handle is not required, so NULL is passed. */

	vSerialPutString(NULL, (signed char *)"Starting scheduler...\n", 22);

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

vGPSReceiveTask(void *pvParameters) {
	(void) pvParameters; //to eliminate compiler warnings for nonuse
	char recBuff[RECEIVE_BUFFER_SIZE], *buffPtr;
	buffPtr = recBuff;

	for(;;) {
		sf_uart_receive(buffPtr, 1); /* Start interrupt-driven receive (does not poll wait) */
		xSemaphoreTake(uartRecDone, portMAX_DELAY); /* block until something received */

		/* Check for NMEA sentence start delimiter */
		if (*buffPtr++ == '$') {
			sf_uart_receive(buffPtr, SENTENCE_NAM_LEN); 	/* Start another receive for next part of */
			xSemaphoreTake(uartRecDone, portMAX_DELAY);	/* sentence, block until done */

			/* Check for correct sentence type and validity */
			if (!strcmp(buffPtr, "GPRMC,")) {
				buffPtr += SENTENCE_NAM_LEN;

				/* Run through timestamp field */
				do {
					sf_uart_receive(buffPtr, 1);
				} while (*buffPtr++ != ',');

				/* Collect Validity bit char and check */
				sf_uart_receive(buffPtr, 2);

				if (*buffPtr == 'A') { /* 'A' is valid, 'V' is not */
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

					int gpsstrlen = ((int)(buffPtr - buffRec)) + 1;

					vSerialPutString(NULL, (signed char *)recBuff, gpsstrlen);
				}
			}
		}

		buffPtr = recBuff;
	}
}
