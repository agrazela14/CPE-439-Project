/*
 * sf_sdcard.h
 *
 * Authors: Tristan Lennertz and Alex Grazela
 */

#ifndef SF_SDCARD_H
#define SF_SDCARD_H

#include "ff.h"

extern SemaphoreHandle_t sdcardSendDone, sdcardRecvDone;

int sf_init_sdcard();

int sf_open_file(FIL *fil, char *SD_File);

int sf_write_file_cur_loc(FIL *fil, void *SourceAddress, u32 BytesToWrite, u32 *NumBytesWritten);

int sf_write_file_location(FIL *fil, void *SourceAddress, u32 BytesToWrite, u32 *NumBytesWritten, u32 location);

int sf_read_file_location(FIL *fil, void *DestinationAddress, u32 ReadSize, u32 *NumBytesRead, u32 location); 

int sf_close_file(FIL *fil);

int sf_unregister_work_area();

int sf_test_file();

#endif
