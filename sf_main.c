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
#define GPS_QUEUE_SIZE 64 

/* Priorities at which the tasks are created. */
#define     mainTASK_PRIORITY_SD            ( tskIDLE_PRIORITY + 4 )
#define     mainTASK_PRIORITY_GPS           ( tskIDLE_PRIORITY + 2 )
#define     mainTASK_PRIORITY_IMU           ( tskIDLE_PRIORITY + 4 )
#define     mainTASK_PRIORITY_DATAPROC      ( tskIDLE_PRIORITY + 3 )


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

/* Queue for writing finished GPS points out to SD card */

//Use these Queues to get the data we care about into a "data processing" task,
//Then that data processing, which will also run the dma, has a queue out to the sd write task

static QueueHandle_t GPSDataQueue = NULL;
static QueueHandle_t IMUDataQueue = NULL;
static QueueHandle_t SDWriteQueue = NULL;

static FIL datafile;


void sf_main(void) {
    /* initialize instances of devices and their interrupt Handlers */
    sf_init_coms();

    /* Configure IMU */
    sf_imu_init();

    GPSDataQueue = xQueueCreate( mainQUEUE_LENGTH, GPS_QUEUE_SIZE);
    IMUDataQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof(float));
    SDWriteQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof(gps_t));

    /* Inititialize SD card & its receive queue
     * Commented out because it was stalling program initially */
    /*int Res = sf_init_sdcard();
    if(Res)
        vSerialPutString(NULL, (signed char *)"Failed to open SD card\r\n", strlen("Failed to open SD card\r\n"));
    else
        vSerialPutString(NULL, (signed char *)"SD card opened\r\n", strlen("SD card opened\r\n"));

    SDWriteQueue = xQueueCreate( SD_QUEUE_SIZE , sizeof( gps_t )); */

    xTaskCreate( vGPSReceiveTask,                   // The function that implements the task.
                "GPS Parse",                        // The text name assigned to the task - for debug only as it is not used by the kernel.
                4096,                               // The size of the stack to allocate to the task.
                NULL,                               // The parameter passed to the task - not used in this case.
                mainTASK_PRIORITY_GPS,              // The priority assigned to the task.
                NULL );                             // The task handle is not required, so NULL is passed.

    xTaskCreate( vIMUFetchTask,                     // The function that implements the task.
                "IMU Fetch",                        // The text name assigned to the task - for debug only as it is not used by the kernel.
                4096,                               // The size of the stack to allocate to the task.
                NULL,                               // The parameter passed to the task - not used in this case.
                mainTASK_PRIORITY_IMU,              // The priority assigned to the task.
                NULL );                             // The task handle is not required, so NULL is passed.

    xTaskCreate( vSDWriteTask,                      // The function that implements the task.
                "SD Write",                         // The text name assigned to the task - for debug only as it is not used by the kernel.
                4096,                               // The size of the stack to allocate to the task.
                NULL,                               // The parameter passed to the task - not used in this case.
                mainTASK_PRIORITY_SD,               // The priority assigned to the task.
                NULL );                             // The task handle is not required, so NULL is passed.

    xTaskCreate( vDataProcessTask,                  // The function that implements the task.
                "Process Data",                     // The text name assigned to the task - for debug only as it is not used by the kernel.
                4096,                               // The size of the stack to allocate to the task.
                NULL,                               // The parameter passed to the task - not used in this case.
                mainTASK_PRIORITY_DATAPROC,         // The priority assigned to the task.
                NULL );                             // The task handle is not required, so NULL is passed.

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

/* This task takes in values from both the GPS and IMU tasks, then uses the functions in sf_gps.c to make them usable floats
   Those floats are then provided to the DMA via Xilinx functions, gets the value out of it, then gives it to the sd writer
   
   Values from GPS received as string in "degree minute second" form
   conversion out of this form is floatval = degree + minute/60 + second/3600
   N positive S negative (latitude)
   E positive W negative (longitude)
   lat comes before long 
   
   IMU data comes in as an X or Y character, then the float value*/

void vDataProcessWriteTask(void *pvParameters) {
    void (pvParamters);
    
    gps_t gps;
    float imuXAcc;
    float imuYAcc;
    int heading;
    char gpsLatLong [GPS_QUEUE_SIZE];
    char gpsHeading [GPS_QUEUE_SIZE];
    
    
    for ( ;; ) {
        xQueueReceive(GPSDataQueue, gpsLatLong, portMAX_DELAY);  
        xQueueReceive(GPSDataQueue, gpsHeading, portMAX_DELAY);  
        heading = atol(gpsHeading);

        xQueueReceive(IMUDataQueue, &imuYAcc, portMAX_DELAY);  
        xQueueReceive(IMUDataQueue, &imuXAcc, portMAX_DELAY);  
        
        convert_lat_long(gpsrecv, &gps);
        convert_acc(heading, imuXAcc, imuYAcc, &gps);
        
    }  
}

/* Read out a gps_t from the dataprocessor, then put into the 
   sf_dma_TxBuffer, of length TX_BUFFER_LENGTH
   in the order of:
   longitude
   latitude
   acc_long
   acc_lat
   cur_long_p
   cur_lat_p
   cur_long_v
   cur_lat_v
   timeslice

   then call 
   sf_dma_transceive();
   to flush that buffer

   from the sf_dma_RxBuffer of length RX_BUFFER_LENGTH
   we will get
   new longitude
   new latitude
   new long_v
   new lat_v

   we will get things out in the same order
*/
    

void vSDWriteTask(void *pvParameters) {
    char dataPrintBuff[256];
    (void) pvParameters;

    gps_t gps; 

    for (;;) {
       xQueueReceive(SDWriteQueue, &gps, portMAX_DELAY);
       
       *sf_dma_TxBuffer = gps->longitude; 
       *(sf_dma_TxBuffer + 1) = gps->latitude; 
       *(sf_dma_TxBuffer + 2) = gps->acc_long; 
       *(sf_dma_TxBuffer + 3) = gps->acc_lat; 
       *(sf_dma_TxBuffer + 4) = gps->cur_long_p; 
       *(sf_dma_TxBuffer + 5) = gps->cur_lat_p; 
       *(sf_dma_TxBuffer + 6) = gps->cur_long_v; 
       *(sf_dma_TxBuffer + 7) = gps->cur_lat_v; 
       *(sf_dma_TxBuffer + 8) = gps->timeslice; 

       sf_dma_transceive();
        




    }
}
    //gps_t toWrite;

    /* This here is test code for the dma, not likely to need it ever again
    for(;;) {
        int whole, dec;

        // Announce TX 
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

        sf_dma_transceive();

        // Announce RX 
        for (i = 0; i < RX_BUFFER_LENGTH; i++) {
            whole = sf_dma_RxBuffer[i];
            dec = ((sf_dma_RxBuffer[i] - whole) * 1000);
            snprintf(dataPrintBuff, 256, "Rx[%d]= %d.%d\r\n", i, whole, dec);
            vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));
        }

        // Block until something is received in queue to write out to file
        //xQueueReceive( SDWriteQueue, &toWrite, portMAX_DELAY );
        */

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

        xQueueSend(IMUDataQueue, (void *)data, 0U); 

        snprintf(dataPrintBuff, 256, "Y: %d\r\n", (int)data);
        vSerialPutString(NULL, (signed char *)dataPrintBuff, strlen(dataPrintBuff));

        data = sf_imu_get_acc_x();

        xQueueSend(IMUDataQueue, (void *)data, 0U); 

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
            sf_uart_receive(buffPtr, SENTENCE_NAM_LEN);     /* Start another receive for next part of */
            //xSemaphoreTake(uartRecDone, portMAX_DELAY);   /* sentence, block until done */

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
                    xQueueSend(GPSDataQueue, (void *)longlat, 0U);

                    /* Receive speed in Knots */
                    do {
                        sf_uart_receive(buffPtr, 1);
                        fieldLength++;
                        headingLen++;
                    } while (*buffPtr++ != ',');
                    snprintf(heading, headingLen, "%s", buffPtr - headingLen);
                    xQueueSend(GPSDataQueue, (void *)heading, 0U);

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
