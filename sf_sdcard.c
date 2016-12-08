/* I have some doubts about the poly learn tutorial by now, so use this as a reference for FATfs
 * http://elm-chan.org/fsw/ff/00index_e.html
 * it seems to have differing ideas on how to use f_mount or needing f_mkfs for example
 */


#include "sf_coms.h"
#include "sf_sdcard.h"
#include "xparameters.h"
#include "xsdps.h"
#include "xplatform_info.h"
#include "semphr.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "ff.h"
#include <string.h>


SemaphoreHandle_t sdcardSendDone, sdcardRecvDone;

/****Here's a big thing for file variables and sich****/
static FATFS fatfs;

#ifdef _ICCARM_
#pragma datat alignment = 32
//u8 destinationAddress[10*1024*1024]
//u8 sourceAddress[10*1024*1024]
#pragma data_alignment = 4
#else
//u8 destinationAddress[10*1024*1024] _attribute_ ((aligned(32)));
//u8 sourceAddress[10*1024*1024] _attribute_ ((aligned(32)));
#endif

#define TEST 7


int sf_init_sdcard() {
    //This is the way the documentation online says, xlinix disagrees
    //Res = f_mkfs("", FM_ANY, 0, work, sizeof work);
    FRESULT Res;
    
    //The f_mkfs in xilinx doesn't follow the online version 
    //I also Don't know if we need mkfs, it may already have one
    //Res = f_mkfs("0:/", 0, 0);
    //Res = f_mkfs(0, 0, 4*1024);
    
    //if (Res) {
    //   return XST_FAILURE;
    //}

    //Res = f_mount(0, &fatfs);
    Res = f_mount(&fatfs, "0:/", 0);

    if (Res) {
        return XST_FAILURE;
    }
    
    sdcardSendDone = xSemaphoreCreateBinary();
    sdcardRecvDone = xSemaphoreCreateBinary();
    
    xSemaphoreGive(sdcardRecvDone);
    xSemaphoreGive(sdcardSendDone);
    
    return XST_SUCCESS;
}

int sf_open_file(FIL *fil, char *SD_File) {
    FRESULT Res = f_open(fil, SD_File, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);

    if (Res) {
        return XST_FAILURE;
    }    
    return XST_SUCCESS;
}
    
int sf_write_file_cur_loc(FIL *fil, void *SourceAddress, u32 BytesToWrite, u32 *NumBytesWritten) {
    FRESULT Res = f_lseek(fil, SEEK_CUR);

    if (Res) {
        return XST_FAILURE;
    }

    Res = f_write(fil, SourceAddress, BytesToWrite, NumBytesWritten);

    if (Res) {
        return XST_FAILURE;
    }

    return XST_SUCCESS;

}

int sf_write_file_location(FIL *fil, void *SourceAddress, u32 BytesToWrite, u32 *NumBytesWritten, u32 location) {
    FRESULT Res = f_lseek(fil, location);

    if (Res) {
        return XST_FAILURE;
    }

    Res = f_write(fil, SourceAddress, BytesToWrite, NumBytesWritten);

    if (Res) {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

int sf_read_file_location(FIL *fil, void *DestinationAddress, u32 ReadSize, u32 *NumBytesRead, u32 location) {
    FRESULT Res = f_lseek(fil, location);

    if (Res) {
        return XST_FAILURE;
    }

    Res = f_read(fil, DestinationAddress, ReadSize, NumBytesRead);

    if (Res) {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

int sf_close_file(FIL *fil) {
    FRESULT Res = f_close(fil);
    
    if (Res) {
        return XST_FAILURE;
    }
    
    return XST_SUCCESS;
}

//I think this is the right way to do this
int sf_unregister_work_area() {
    FRESULT Res = f_mount(NULL, "0:/", 0);
    
    if (Res) {
        return XST_FAILURE;
    }
    return XST_SUCCESS;
}

//A test function to run after initing the File System, then check the micro SD card and see if you can see a file
int sf_test_file() {
    FRESULT Res;
    FIL testFile;
    char filename[16] = "Test";
    char received[16];
    u32 transferredBytes;
    

    Res = sf_open_file(&testFile, filename);

    if (Res) {
        return Res;
    }
    
    //Write 4 bytes from "Test" to location 0, place how many bytes written into transferredBytes
    Res = sf_write_file_location(&testFile, filename, 4, &transferredBytes, 0); 

    if (Res) {
        return Res;
    }
    
    //Read 4 bytes from the file and put them in recieved buffer, transferredBytes once again gets how many were actually read
    Res = sf_read_file_location(&testFile, received, 4, &transferredBytes, 0); 

    if (Res) {
        return Res;
    }

    Res = sf_close_file(&testFile);

    if (Res) {
        return Res;
    }

    return XST_SUCCESS;
}

