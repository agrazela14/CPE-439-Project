/*
 * sf_main.c
 *
 * Authors: Tristan Lennertz and Alex Grazela
 */

// This important macro defines whether it's running the full thing (0)
// Or just a test of the sensors (Not 0)
#define TEST 0

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* C lib includes */
#include <string.h>
#include <stdlib.h>
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
#define GPS_QUEUE_SIZE 64 
#define MILLION 100000

/* Priorities at which the tasks are created. */
#define 	mainTASK_PRIORITY_TEST			( tskIDLE_PRIORITY + 5 )
#define     mainTASK_PRIORITY_SD            ( tskIDLE_PRIORITY + 4 )
#define     mainTASK_PRIORITY_GPS           ( tskIDLE_PRIORITY + 3 )
#define     mainTASK_PRIORITY_IMU           ( tskIDLE_PRIORITY + 4 )
#define     mainTASK_PRIORITY_DATAPROC      ( tskIDLE_PRIORITY + 3 )
#define 	mainTASK_PRIORITY_CALIB			( tskIDLE_PRIORITY + 6 )



/* Quantum of time to fetch measurements from IMU */
#define mainIMU_FETCH_FREQ          ( 1000 / portTICK_PERIOD_MS )

/* Time to wait between switching different modes in IMU */
#define mainIMU_SWITCH_TIME         ( 19 / portTICK_PERIOD_MS )

/* Number of Items the queue can hold, setting it to 1 since we want to receive them immediately */
#define mainQUEUE_LENGTH 1

/* Task Forward Declarations */
void vGPSReceiveTask(void *pvParameters);
void vIMUFetchTask(void *pvParameters);
void vSDWriteTask(void *pvParameters);
void vDataProcessWriteTask(void *pvParameters);
void vtestTask(void *pvParameters);
void calibrationTask(void *pvParameters);

//Use these Queues to get the data we care about into a "data processing" task,
//Then that data processing, which will also run the dma, has a queue out to the sd write task
static QueueHandle_t GPSDataQueue = NULL;
static QueueHandle_t IMUDataQueue = NULL;
static QueueHandle_t SDWriteQueue = NULL;

/* Used to stall the other data tasks until calibration has been sufficiently satisfied */
SemaphoreHandle_t waketestTask;

/* Declaration of data log file */
static FIL datafile;
#define filename "datalog"

/* Declaration of plot file */
static FIL gps_plot_file;
#define gps_plot_filename "plot.csv"

void sf_main(void) {
    /* initialize instances of devices and their interrupt Handlers */
    sf_init_coms();

    /* Configure IMU */
    sf_imu_init();

	/* Initialize DMA Engine */
	sf_init_dma();
    
	/* Create the queues for data passing between tasks */
    SDWriteQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof(gps_t));
	GPSDataQueue = xQueueCreate( mainQUEUE_LENGTH, GPS_QUEUE_SIZE);
    IMUDataQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof(float));

    /* create semaphores for stalling test task until calibration done */
    waketestTask = xSemaphoreCreateBinary();

#if TEST == 0
	xTaskCreate( vGPSReceiveTask,					// The function that implements the task.
				"GPS Parse", 						// The text name assigned to the task 
				4096, 								// The size of the stack to allocate to the task.
				NULL, 								// The parameter passed to the task - not used in this case.
				mainTASK_PRIORITY_GPS, 				// The priority assigned to the task.
				NULL );								// The task handle is not required, so NULL is passed.

	xTaskCreate( vIMUFetchTask,						// The function that implements the task.
				"IMU Fetch", 						// The text name assigned to the task 
				4096, 								// The size of the stack to allocate to the task.
				NULL, 								// The parameter passed to the task - not used in this case.
				mainTASK_PRIORITY_IMU, 				// The priority assigned to the task.
				NULL );								// The task handle is not required, so NULL is passed.

	xTaskCreate( vSDWriteTask,						// The function that implements the task.
				"SD Write", 						// The text name assigned to the task
				4096, 								// The size of the stack to allocate to the task.
				NULL, 								// The parameter passed to the task - not used in this case.
				mainTASK_PRIORITY_SD, 				// The priority assigned to the task.
				NULL );								// The task handle is not required, so NULL is passed.

	xTaskCreate(vDataProcessWriteTask,					// The function that implements the task.
				"Process Data", 					// The text name assigned to the task 
				4096, 								// The size of the stack to allocate to the task.
				NULL, 								// The parameter passed to the task - not used in this case.
				mainTASK_PRIORITY_DATAPROC, 		// The priority assigned to the task.
				NULL );								// The task handle is not required, so NULL is passed.
#else
	xTaskCreate(vtestTask,							// The function that implements the task.
				"Test", 							// The text name assigned to the task
				4096, 								// The size of the stack to allocate to the task.
				NULL, 								// The parameter passed to the task - not used in this case.
				mainTASK_PRIORITY_TEST, 			// The priority assigned to the task.
				NULL );								// The task handle is not required, so NULL is passed.

	xTaskCreate(calibrationTask,					// The function that implements the task.
				"Calibration", 						// The text name assigned to the task .
				4096, 								// The size of the stack to allocate to the task.
				NULL, 								// The parameter passed to the task - not used in this case.
				mainTASK_PRIORITY_CALIB, 			// The priority assigned to the task.
				NULL );								// The task handle is not required, so NULL is passed.
#endif 

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

/* Only used if TEST is defined as 1. This is the first state of TEST mode.
 * The IMU basically outputs a calibration status until the user is satisfied
 * with the calibration status, then presses any key. That exits the calibration
 * state and allows it to enter the test state, which is just running the test
 * task, outputting the valid GPS data, IMU acceleration, and compass heading. */
void calibrationTask(void *pvParameters) {
	(void) pvParameters;
	char dataPrintBuff[256];
	char dataRecBuff[256];
	int calibrated = 0;

    /* Initialise xNextWakeTime - this only needs to be done once. */
    TickType_t xNextWakeTime = xTaskGetTickCount();

    /* Will exit calibration once a key is pressed */
	while(!calibrated) {
		/* wait for a char for (mainIMU_FETCH_FREQ) amount of time */
		int res = xSerialGetChar(NULL, dataRecBuff, mainIMU_FETCH_FREQ);

		/* any key stroke,  at all? */
        if(res == pdTRUE) {
        	xSemaphoreGive(waketestTask);
        	calibrated = 1;
        }
        else {
        	/* read calibration status from IMU */
			u8 data = sf_imu_get_calib();

			/* format and print the calibration status */
			snprintf(dataPrintBuff, 256, "Calibration Status: %X\r\n", (int)data);
			vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));
        }
	}

	/* Once exited, just delay forever */
	vTaskDelayUntil( &xNextWakeTime, portMAX_DELAY);
}

/* Only runs if TEST is defined as 1. Will loop, waking up every mainIMU_FETCH_FREQ
 * in order to read the various subsystem data streams and output it to the console */
void vtestTask(void *pvParameters) {
	(void) pvParameters;
	char dataPrintBuff[256];
	u16 data;

    /* Initialise xNextWakeTime - this only needs to be done once. */
    TickType_t xNextWakeTime = xTaskGetTickCount();

    /* Block on receiving OKAY from calibration task */
    xSemaphoreTake(waketestTask, portMAX_DELAY);

	for(;;) {
		/* wake up every mainIMU_FETCH_FREQ to output all this stuff */
		vTaskDelayUntil( &xNextWakeTime, portMAX_DELAY);

		/* Fill in the data buffer for a dead reckoning calculation.
		 * The order of the input parameters is defined in the Vivado HLS C file */
		sf_dma_TxBuffer[0] = 6.9;
		sf_dma_TxBuffer[1] = 7.9;
		sf_dma_TxBuffer[2] = 2.1;
		sf_dma_TxBuffer[3] = 0.5;
		sf_dma_TxBuffer[4] = 1069.3;
		sf_dma_TxBuffer[5] = 30.0002;
		sf_dma_TxBuffer[6] = 1.2;
		sf_dma_TxBuffer[7] = 0.69;
		sf_dma_TxBuffer[8] = 0.5;

		int j;

		/* Fill the rest of the transfer buffer with 0s. Don't actually need to,
		 * but helps to account for every part of it in your head. The buffer is
		 * purposely larger than our needs for the dead reckoning. This is because
		 * DMA is relatively quick and gives as plenty of space for easily adding
		 * functionality in something like testing or the passing of more data */
		for(j = 9; j < 50; j++)
			sf_dma_TxBuffer[j] = 0;

		/* Write zeros to the receive buffer so that we're not looking at values from a
		 * previous transceive, or general junk data */
		for (j = 0; j < 50; j++) {
			sf_dma_RxBuffer[j] = 0;
		}

		/* initiate DMA transceive. Blocks here in the logical flow until the receive is done */
		sf_dma_transceive();

		/* FreeRTOS lacks %f printing functionality, so these will help us in printing floats with
		 * clever number manipulation */
		int whole, dec, i;

		/* Output the parts of the transmit and receive buffers we want to see */
		for (i = 0; i < 8; i++) { /* TX first */
			/* formatting for a float print */
			if (sf_dma_TxBuffer[i] > 1 || sf_dma_TxBuffer[i] < -1)
				whole = sf_dma_TxBuffer[i];
			else
				whole = 0;

			dec = ((sf_dma_TxBuffer[i] - whole) * 100000);
			snprintf(dataPrintBuff, 256, "Tx[%d]= %d.%d\r\n", i, whole, dec);

			/* finally print */
			vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));
		}

		for (i = 0; i < 8; i++) { /* RX second */
			/* formatting for a float print */
			if (sf_dma_RxBuffer[i] > 1 || sf_dma_RxBuffer[i] < -1)
				whole = sf_dma_RxBuffer[i];
			else
				whole = 0;

			dec = ((sf_dma_RxBuffer[i] - whole) * 100000);
			snprintf(dataPrintBuff, 256, "Rx[%d]= %d.%d\r\n", i, whole, dec);

			/* finally print */
			vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));
		}

		/* Output compass heading */
		data = sf_imu_get_heading();
		snprintf(dataPrintBuff, 256, "Compass Heading: %d degrees\r\n", (int)(data / 16));
		vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));

		/* Output Z axis acc */
        data = sf_imu_get_acc_z();
        snprintf(dataPrintBuff, 256, "Z: %d\r\n", (int)(data));
        vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));

    	/* Output Y axis acc */
        data = sf_imu_get_acc_y();
        snprintf(dataPrintBuff, 256, "Y: %d\r\n", (int)data);
        vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));


    	/* Output X axis acc */
        data = sf_imu_get_acc_x();
        snprintf(dataPrintBuff, 256, "X: %d\r\n", (int)data);
        vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));
	}
}

/* This task takes in values from both the GPS and IMU tasks, then uses the functions in sf_gps.c to make them usable floats
   Those floats are then provided to the DMA via Xilinx functions, gets the value out of it, then gives it to the sd writer
   Values from GPS received as string in "degree minute second" form
   conversion out of this form is floatval = degree + minute/60 + second/3600
   N positive S negative (latitude)
   E positive W negative (longitude)
   lat comes before long */
 void vDataProcessWriteTask(void *pvParameters) {
    (void) pvParameters;
    
    gps_t gps;
    float imuXAcc;
    float imuYAcc;
    int heading;

    char gpsLatLong [GPS_QUEUE_SIZE];
    char gpsHeading [GPS_QUEUE_SIZE];
    
    for ( ;; ) {
    	/* Receive and convert GPS data */
        xQueueReceive(GPSDataQueue, gpsLatLong, portMAX_DELAY);  
        xQueueReceive(GPSDataQueue, gpsHeading, portMAX_DELAY);  
        heading = atol(gpsHeading);

        /* Receive and convert IMU data */
        xQueueReceive(IMUDataQueue, &imuYAcc, portMAX_DELAY);  
        xQueueReceive(IMUDataQueue, &imuXAcc, portMAX_DELAY);  
        convert_lat_long(gpsLatLong, &gps);
        convert_acc(heading, imuXAcc, imuYAcc, &gps);
    }  
}

/* Block on a queue receiving a complete gps position/acc data struct.
 * Once received, pass to hardware accelerator, then receive result back. Print result
 * to the SD card */
void vSDWriteTask(void *pvParameters) {
    char dataPrintBuff[256];
    char plotPrintBuff[256];

    (void) pvParameters;
    int lg_wl;
    int lg_dc;
    int lt_wl;
    int lt_dc;
    int lg_v_wl;
    int lg_v_dc;
    int lt_v_wl;
    int lt_v_dc;

    float longitude;
    float latitude;
    float long_v;
    float lat_v;
    
    int bytesWritten;

    gps_t gps; 

    /* Initialize writing to SD card */
    int Res = sf_init_sdcard();

	if(Res)
		vSerialPutString(NULL, (signed char *)"Failed to open SD card\r\n", strlen("Failed to open SD card\r\n"));
	else
		vSerialPutString(NULL, (signed char *)"SD card opened\r\n", strlen("SD card opened\r\n"));

	/* Open files on SD card */
	sf_open_file(&datafile, filename);
    sf_open_file(&gps_plot_file, gps_plot_filename);

    for (;;) {
    	/* wait on recieving something to write to SD card */
       xQueueReceive(SDWriteQueue, &gps, portMAX_DELAY);
       
       /* Fill DMA buffer
        * NOTE: hardware accelerator currently only configured for the dead reckoning
        * demonstrated in the TEST mode. Commented out here until hardware is
        * available to handle it */
       /*
		sf_dma_TxBuffer[0] = gps.longitude;
		sf_dma_TxBuffer[1] = gps.latitude;
		sf_dma_TxBuffer[2] = gps.acc_long;
		sf_dma_TxBuffer[3] = gps.acc_lat;
		sf_dma_TxBuffer[4] = gps.cur_long_p;
		sf_dma_TxBuffer[5] = gps.cur_lat_p;
		sf_dma_TxBuffer[6] = gps.cur_long_v;
		sf_dma_TxBuffer[7] = gps.cur_lat_v;
		sf_dma_TxBuffer[8] = gps.timeslice;
		*/

       /* Perform Transceive */
		/*sf_dma_transceive(); */

       /* Read from dma rx buffer */
       /*
		? = sf_dma_RxBuffer[0];
		? = sf_dma_RxBuffer[1];
		? = sf_dma_RxBuffer[2];
		? = sf_dma_RxBuffer[3];
		? = sf_dma_RxBuffer[4];
		? = sf_dma_RxBuffer[5];
		? = sf_dma_RxBuffer[6];
		? = sf_dma_RxBuffer[7];
		? = sf_dma_RxBuffer[8];
		*/
       
       /*
       longitude = *(sf_dma_RxBuffer); 
       latitude  = *(sf_dma_RxBuffer + 1); 
       long_v    = *(sf_dma_RxBuffer + 2); 
       lat_v     = *(sf_dma_RxBuffer + 3); 
       */

       /* For now, just print out original data from GPS */
       longitude = gps.longitude;
       latitude = gps.latitude;
       long_v = gps.cur_long_v;
	   lat_v = gps.cur_lat_v;
       
	   /* Convert from meters to decimal degrees, for proper GPS format */
       sf_return_to_decimal_degree(&longitude, &latitude);
        
       lg_wl = longitude;
       //make it positive for the decimal part
       if (longitude < 0) {
           longitude *= (-1);
       }
       lg_dc = (longitude - lg_wl) * MILLION; 
       lt_wl = latitude;
       if (latitude < 0) {
           latitude *= (-1);
       }
       lt_dc = (latitude - lt_wl) * MILLION; 
       lg_v_wl = long_v;
       if (long_v < 0) {
           long_v *= (-1);
       }
       lg_v_dc = (long_v - lg_v_wl) * MILLION; 
       lt_v_wl = lat_v;
       if (lat_v < 0) {
           lat_v *= (-1);
       }
       lt_v_dc = (lat_v - lt_v_wl) * MILLION; 

       sprintf(dataPrintBuff, "Longitude: %d.%d | Latitude: %d.%d | Longitudinal Velocity: %d.%d | Latitudinal Velocity %d.%d\n",
               lg_wl, lg_dc, lt_wl, lt_dc, lg_v_wl, lg_v_dc, lt_v_wl, lt_v_dc); 

       /* print to files */
       sf_write_file_cur_loc(&datafile, (void *)dataPrintBuff, 256, &bytesWritten);
        
       sprintf(plotPrintBuff, "%d.%d,%d.%d\n", lg_wl, lg_dc, lt_wl, lt_dc);
       sf_write_file_cur_loc(&gps_plot_file, (void *)plotPrintBuff, 256, &bytesWritten);
    }
}

/* Periodically wake up, fetch the required IMU data, and queue it
 * off to the data processing task. This wakeup time is the quanta of
 * our dead reckoning / Kalman Filter */
void vIMUFetchTask(void *pvParameters) {
    (void) pvParameters;
    char dataPrintBuff[256];
    u16 data;

    /* Initialise xNextWakeTime - this only needs to be done once. */
    TickType_t xNextWakeTime = xTaskGetTickCount();

    /* make sure the IMU has had enough time to switch out of config mode */
    vTaskDelayUntil( &xNextWakeTime, mainIMU_SWITCH_TIME);

    for( ;; )
    {
    	/* Wake up every mainIMU_FETCH_FREQ to fetch know IMU data */
    	vTaskDelayUntil( &xNextWakeTime, mainIMU_FETCH_FREQ);

    	/* get y axis acc */
        data = sf_imu_get_acc_y();

        /* send it to data processing queue */
        xQueueSend(IMUDataQueue, &data, 0U);

    	/* get x axis acc */
        data = sf_imu_get_acc_x();

        /* send it to the data processing queue */
        xQueueSend(IMUDataQueue, &data, 0U);
    }
}

/* Continuously Receive and parse GPS data, only keeping the valid packets,
 * queuing them off to be processed */
void vGPSReceiveTask(void *pvParameters) {
    (void) pvParameters; //to eliminate compiler warnings for nonuse
    char recBuff[RECEIVE_BUFFER_SIZE], *buffPtr;
    buffPtr = recBuff;

    for(;;) {
        sf_uart_receive(buffPtr, 1); /* Start interrupt-driven receive (does not poll wait) */
        //xSemaphoreTake(uartRecDone, portMAX_DELAY); /* block until something received */

        /* Check for NMEA sentence start delimiter */
        if (*buffPtr++ == '$') {
            sf_uart_receive(buffPtr, SENTENCE_NAM_LEN);     /* Start another receive for next part of */

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
                    char longlat[GPS_QUEUE_SIZE];
                    char heading[GPS_QUEUE_SIZE];
                    int longlatLen = 0;
                    int headingLen = 0;
                    int fieldLength = 0;

                    /* Receive the Latitude field */
                    // N positive, S negative 
                    do {
                        sf_uart_receive(buffPtr, 1);
                        fieldLength++;
                    } while (*buffPtr++ != ',');

                    /* Receive the N/S value of latitude */
                    sf_uart_receive(buffPtr, 2);
                    buffPtr += 2;
                    longlatLen += 2;

                    /* Receive the longitude */
                    // E positive, W negative 
                    do {
                        sf_uart_receive(buffPtr, 1);
                        fieldLength++;
                        longlatLen++;
                    } while (*buffPtr++ != ',');

                    /* Receive the E/W value of longitude */
                    sf_uart_receive(buffPtr, 2);
                    buffPtr += 2;
                    longlatLen += 2;
                    
                    //send the lat/long string (in degree minute second form) to the data processor
                    snprintf(longlat, longlatLen, "%s", buffPtr - longlatLen);
                    //xQueueSend(GPSDataQueue, (void *)longlat, 0U);

                    /* Receive speed in Knots */
                    do {
                        sf_uart_receive(buffPtr, 1);
                        fieldLength++;
                        headingLen++;
                    } while (*buffPtr++ != ',');
                    snprintf(heading, headingLen, "%s", buffPtr - headingLen);
                    //xQueueSend(GPSDataQueue, (void *)heading, 0U);

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
