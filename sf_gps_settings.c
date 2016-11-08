#include "sf_coms.h"
#include <stdio.h>

/* From the Data Sheet:
0 NMEA_SEN_GLL, // GPGLL interval - Geographic Position - Latitude longitude
1 NMEA_SEN_RMC, // GPRMC interval - Recommended Minimum Specific GNSS Sentence
2 NMEA_SEN_VTG, // GPVTG interval - Course over Ground and Ground Speed
3 NMEA_SEN_GGA, // GPGGA interval - GPS Fix Data
4 NMEA_SEN_GSA, // GPGSA interval - GNSS DOPS and Active Satellites
5 NMEA_SEN_GSV, // GPGSV interval - GNSS Satellites in View
6 //Reserved
7 //Reserved
13 //Reserved
14 //Reserved
15 //Reserved
16 //Reserved
17 //Reserved
18 NMEA_SEN_MCHN, // PMTKCHN interval – GPS channel status 

*/

/*
The Datasheet for commands to send to this memer is availble at https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf
Also note, I'm not sure if the <CR><LF> is supposed to be two certain bytes or something?
Another note, the GPS is always spitting out it's data without the need to query it, or is that not right?
*/

#define messagePreambleLength 8
#define baudRateExtraCharactes 12
#define StartMessageLength 19
#define nmeaRateMessageLength 57

//This should restart it to full factory default, clearing anything we have set
void sf_gps_full_cold_start() {
    char fullColdStartMessage[StartMessageLength] = "$PMTK104*37<CR><LF>";
    sf_uart_write((u8 *)fullColdStartMessage, StartMessageLength);
}

void sf_gps_cold_start() {
    char coldStartMessage[StartMessageLength] = "$PMTK103*30<CR><LF>";
    sf_uart_write((u8 *)coldStartMessage, StartMessageLength);
}

void sf_gps_warm_start() {
    char warmStartMessage[StartMessageLength] = "$PMTK102*31<CR><LF>";
    sf_uart_write((u8 *)warmStartMessage, StartMessageLength);
}

void sf_gps_hot_start() {
    char *hotStartMessage = "$PMTK101*32<CR><LF>";
    sf_uart_write((u8 *)hotStartMessage, StartMessageLength);
}



void sf_gps_set_baud(u32 baudRate, u8 baudByteLen) {
    char baudRateBuf[baudByteLen];
    char baudRateMessage[messagePreambleLength + baudByteLen + baudRateExtraCharactes];
    char preamble[messagePreambleLength + 1];
    char checkSumBuf[2];
    
    snprintf(baudRateBuf, "%d", baudRate);
    sprintf(preamble, "$PMKT251,");
    //sprintf(intervalMessage, "%s%s*%s<CR><LF>", preamble, baudRateBuf, checkSumBuf);
    
    sf_uart_write((u8 *)baudRateMessage, baudByteLen + messagePreambleLength + baudRateExtraCharactes);
}

void sf_gps_set_fix_interval(u32 interval, u32 intervalByteLength) {
    char intervalBuf[intervalByteLength];
    char preamble[messagePreambleLength + 1];
    //char intervalMessage[intervalByteLength + messagePreambleLength + ];
    char checkSumBuf[2];
    
    sprintf(intervalBuf, "%d", interval);
    sprintf(preamble, "$PMKT220,");
    //sprintf(intervalMessage, "%s%s*%s<CR><LF>", preamble, intervalBuf, checkSumBuf);
    
    //This 12 comes from the comma, the *, the 2 bytes for the checksum, and 8 from <CR><LF>
    //sf_uart_write(intervalMessage, messagePreambleLength + intervalByteLength + 12);
}

//This will construct and send a message that sets each valid nmea sentence recieve rate based on 1 - 5 fixes
//Of course this means that each u8 passed in should only be in values 0 (invalid) or 1 - 5
//nmea Rate example $PMTK314,1,1,1,1,1,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2C<CR><LF> 

void sf_gps_set_nmea_rate(u8 GLL, u8 RMC, u8 VTG, u8 GGA, u8 GSA, u8 GSV) {
    //char nmeaRateMessage[nmeaRateMessageLenth];
    char GLLBuf[2];
    char RMCBuf[2];
    char VTGBuf[2];
    char GSABuf[2];
    char GSVBuf[2];
    char CRLFBuf[8] = "<CR><LF>";
    char checkSumBuf[2];
    char ReservedBuf[25];
    char preamble[messagePreambleLength];
    
    GLLBuf[1] = ','; 
    RMCBuf[1] = ','; 
    VTGBuf[1] = ','; 
    GSABuf[1] = ','; 
    GSVBuf[1] = ','; 
    
    snprintf(GLLBuf, "%d", GLL);
    snprintf(RMCBuf, "%d", RMC);
    snprintf(VTGBuf, "%d", VTG);
    snprintf(GSABuf, "%d", GSA);
    snprintf(GSVBuf, "%d", GSV);
        
    snprintf(ReservedBuf, "0,0,0,0,0,0,0,0,0,0,0,0,0", 25);
    snprintf(preamble, "$PMTK314,", messagePreambleLength + 1);
    
    //snprintf(nmeaRateMessage, "%s%s%s%s%s%s%s%s%s", preamble, GLLBuf, RMCBuf, VTGBuf, GSABuf, GSVBuf, checkSumBuf, ReservedBuf, CRLFBuf);
    //sf_uart_write((u8 *)nmeaRateMessage, nmeaRateMessageLength);
}


