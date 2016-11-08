/*
 * sf_main.c
 *
 * Authors: Tristan Lennertz and Alex Grazela
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Sensor Fusion Project Includes */
#include "sf_coms.h"

/* temporary serial include */
#include "serial.h"

/* The following defines are there for convenience, and should be modified
 * to fit desired specifics of functionality
 */

/* Priorities at which the tasks are created. */
//#define mainTASK_PRIORITY_1			( tskIDLE_PRIORITY + 2 )
//#define	mainTASK_PRIORITY_2			( tskIDLE_PRIORITY + 1 )

/* How to define time from ticks */
//#define mainFREQUENCY_MS_1			( 420 / portTICK_PERIOD_MS )

void sf_main(void) {
	/*CODE*/


	/* Below is there for convenience when we are implementing scheduling */

	//xTaskCreate( prvQueueReceiveTask,				/* The function that implements the task. */
	//			"Rx", 								/* The text name assigned to the task - for debug only as it is not used by the kernel. */
	//			configMINIMAL_STACK_SIZE, 			/* The size of the stack to allocate to the task. */
	//			NULL, 								/* The parameter passed to the task - not used in this case. */
	//			mainQUEUE_RECEIVE_TASK_PRIORITY, 	/* The priority assigned to the task. */
	//			NULL );								/* The task handle is not required, so NULL is passed. */

	/* Start the tasks and timer running. */
	//vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was either insufficient FreeRTOS heap memory available for the idle
	and/or timer tasks to be created, or vTaskStartScheduler() was called from
	User mode.  See the memory management section on the FreeRTOS web site for
	more details on the FreeRTOS heap http://www.freertos.org/a00111.html.  The
	mode from which main() is called is set in the C start up code and must be
	a privileged mode (not user mode). */
	for( ;; )
		vSerialPutString(NULL, "dick", 4);
}
