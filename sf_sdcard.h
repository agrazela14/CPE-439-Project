extern SemaphoreHandle_t sdcardSendDone, sdcardRecDone;

int sf_init_sdcard(char *SD_File, char *FileName, FIL *fil);

int sf_write_sdcard_location(FIL *fil, void *SourceAddress, u32 FileSize, u32 *NumBytesWritten, u32 location);

int sf_read_sdcard_location(FIL *fil, void *DestinationAddress, u32 FileSize, u32 *NumBytesWritten, u32 location);

int sf_close_sdcard_file(FIL *fil) ;
