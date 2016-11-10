#ifndef SF_GPS_SETTINGS_H
#define SF_GPS_SETTINGS_H

typedef struct {
    double longitude;
    double latitude;
    signed char NS; //negative 1 or positive 1, a mutliplier for latitude
    signed char EW; //negative 1 or positive 1, a mutliplier for longitude
} gpsData; 

void sf_gps_full_cold_start();

void sf_gps_cold_start();

void sf_gps_warm_start();
	
void sf_gps_hot_start();	
	
void sf_gps_set_baud(u32 baudRate, u8 baudByteLen);	
	
void sf_gps_set_fix_interval(u32 interval, u32 intervalByteLength);	
	
void sf_gps_set_nmea_rate(u8 GLL, u8 RMC, u8 VTG, u8 GGA, u8 GSA, u8 GSV);	

void sf_gps_checksum_calc(char *checksumBuf, char *messageString);

void sf_gps_read(char *readBuf, u32 numBytes);

void sf_gps_checksum_calc(char *checksumBuf, char *messageString);

void sf_gps_checksum_calc(char *checksumBuf, char *messageString);
	
#endif
