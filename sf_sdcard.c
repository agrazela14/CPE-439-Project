/* I have some doubts about the poly learn tutorial by now, so use this as a reference for FATfs
 * http://elm-chan.org/fsw/ff/00index_e.html
 * it seems to have differing ideas on how to use f_mount or needing f_mkfs for example
 */


#include "sf_coms.c"
#include "sf_sdcard.h"
#include <string.h>


SemaphoreHandle_t sdcardSendDone, sdcardRecDone;

/****Here's a big thing for file variables and sich****/
static FATFS fatfs;

/*
#ifdef _ICCARM_
#pragma datat alignment = 32
u8 destinationAddress[10*1024*1024]
u8 sourceAddress[10*1024*1024]
#pragma data_alignment = 4
#else
u8 destinationAddress[10*1024*1024] _attribute_ ((aligned(32)));
u8 sourceAddress[10*1024*1024] _attribute_ ((aligned(32)));
#endif

#define TEST 7
*/


int sf_init_sdcard(FIL *fil, char *SD_File) {
    FRESULT Res;
    u32 BuffCnt;
    u32 FileSize = (8*1024*1024);
    
    /* 
    for (BuffCnt = 0; BuffCnt < FileSize; BuffCnt++) {
        SourceAddress[BuffCnt] = TEST + BuffCnt;
    }
    */

    Res = f_mount(0, &fatfs);

    if (Res != FR_OK) {
        return XST_FAILURE;
    }    
    
    Res = f_open(fil, SD_File, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);

    if (Res) {
        return XST_FAILURE;
    }    

    return XST_SUCCESS;
}

int sf_write_sdcard_location(FIL *fil, void *SourceAddress, u32 FileSize, u32 *NumBytesWritten, u32 location) {
    int Res = f_lseek(fil, location);

    if (Res) {
        return XST_FAILURE;
    }

    Res = f_write(fil, SourceAddress, FileSize, NumBytesWritten);

    if (Res) {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

int sf_read_sdcard_location(FIL *fil, void *DestinationAddress, u32 FileSize, u32 *NumBytesWritten, u32 location) {
    int Res = f_lseek(fil, location);

    if (Res) {
        return XST_FAILURE;
    }

    Res = f_write(fil, DestinationAddress, FileSize, NumBytesWritten);

    if (Res) {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

int sf_close_sdcard_file(FIL *fil) {
    int Res = f_close(fil);
    
    if (Res) {
        return XST_FAILURE;
    }
    return XST_SUCCESS;
}

