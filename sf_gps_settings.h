#ifndef SF_GPS_SETTINGS_H
#define SF_GPS_SETTINGS_H
void sf_gps_full_cold_start();

void sf_gps_cold_start();

void sf_gps_warm_start();
	
void sf_gps_hot_start();	
	
void sf_gps_set_baud(u32 baudRate, u8 baudByteLen);	
	
void sf_gps_set_fix_interval(u32 interval, u32 intervalByteLength);	
	
void sf_gps_set_nmea_rate(u8 GLL, u8 RMC, u8 VTG, u8 GGA, u8 GSA, u8 GSV);	
	
#endif