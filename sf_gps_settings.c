#include "sf_coms.h"
#include <stdio.h>
#include <string.h>

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
Another note, the GPS is always spitting out it's data without the need to query it, or is that not right?
*/

#define messagePreambleLength 9
#define checksumLength 3
#define baudRateExtraCharacters 12
#define StartMessageLength 13
#define nmeaRateReservedLength 26
#define nmeaRateMessageLength 51

//This should restart it to full factory default, clearing anything we have set
void sf_gps_full_cold_start() {
    char fullColdStartMessage[StartMessageLength];
    snprintf(fullColdStartMessage, "$PMTK104*37\r\n");

    sf_uart_write((u8 *)fullColdStartMessage, StartMessageLength);
}

void sf_gps_cold_start() {
    char coldStartMessage[StartMessageLength];
    snprintf(coldStartMessage, "$PMTK103*30\r\n");

    sf_uart_write((u8 *)coldStartMessage, StartMessageLength);
}

void sf_gps_warm_start() {
    char warmStartMessage[StartMessageLength];
    snprintf(warmStartMessage, "$PMTK102*31\r\n");
    
    sf_uart_write((u8 *)warmStartMessage, StartMessageLength);
}

void sf_gps_hot_start() {
    char hotStartMessage[StartMessageLength];
    snprintf(hotStartMessage, "$PMTK101*32\r\n");

    sf_uart_write((u8 *)hotStartMessage, StartMessageLength);
}



void sf_gps_set_baud(u32 baudRate, u8 baudByteLen) {
    char baudRateBuf[baudByteLen];
    char baudRateMessage[messagePreambleLength + baudByteLen + baudRateExtraCharacters];
    char preamble[messagePreambleLength + 1];
    char checksumBuf[checksumLength];
    
    snprintf(baudRateBuf, "%d\0", baudRate);
    sprintf(preamble, "$PMKT251,\0");
    //sprintf(intervalMessage, "%s%s*%s\r\n", preamble, baudRateBuf, checksumBuf);

    sprintf(baudRateMessage, "%s%s*", preamble, baudRateBuf);
    sf_gps_checksum_calc(checksumBuf, baudRateMessage);
    sprintf(baudRateMessage, "%s\r\n", checksumBuf);

    
    sf_uart_write((u8 *)baudRateMessage, baudByteLen + messagePreambleLength + checksumLength + 4);
    //the +4 is a , * and \r\n
}

void sf_gps_set_fix_interval(u32 interval, u32 intervalByteLength) {
    char intervalBuf[intervalByteLength];
    char preamble[messagePreambleLength + 1];
    char intervalMessage[intervalByteLength + messagePreambleLength + checksumLength + 4];
    char checksumBuf[checksumLength];
    
    sprintf(intervalBuf, "%d\0", interval);
    sprintf(preamble, "$PMKT220,\0");
    //sprintf(intervalMessage, "%s%s*%s\r\n", preamble, intervalBuf, checksumBuf);

    sprintf(intervalMessage, "%s%s*", preamble, intervalBuf);
    sf_gps_checksum_calc(checksumBuf, intervalMessage);
    sprintf(intervalMessage, "%s\r\n", checksumBuf);
    
    sf_uart_write(intervalMessage, messagePreambleLength + intervalByteLength + 4);
    //the +4 is a , * and \r\n
}

//This will construct and send a message that sets each valid nmea sentence recieve rate based on 1 - 5 fixes
//Of course this means that each u8 passed in should only be in values 0 (invalid) or 1 - 5
//nmea Rate example $PMTK314,1,1,1,1,1,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2C<CR><LF> 

void sf_gps_set_nmea_rate(u8 GLL, u8 RMC, u8 VTG, u8 GGA, u8 GSA, u8 GSV) {
    char nmeaRateMessage[nmeaRateMessageLenth];
    char GLLBuf[3];
    char RMCBuf[3];
    char VTGBuf[3];
    char GSABuf[3];
    char GSVBuf[3];
    char CRLFBuf[3];
    char checksumBuf[3];
    char ReservedBuf[nmeaRateReservedLength];
    char preamble[messagePreambleLength];
    
    snprintf(CRLFBuf, "\r\n\0");
    snprintf(GLLBuf, "%d", GLL);
    snprintf(RMCBuf, "%d", RMC);
    snprintf(VTGBuf, "%d", VTG);
    snprintf(GSABuf, "%d", GSA);
    snprintf(GSVBuf, "%d", GSV);
        
    sprintf(GLLBuf + 1, ",\0");
    sprintf(RMCBuf + 1, ",\0");
    sprintf(RMCBuf + 1, ",\0");
    sprintf(VTGBuf + 1, ",\0");
    sprintf(GSABuf + 1, ",\0");
    sprintf(GSVBuf + 1, ",\0");

    snprintf(ReservedBuf, "0,0,0,0,0,0,0,0,0,0,0,0,0\0");
    snprintf(preamble, "$PMTK314,\0");
    
    snprintf(nmeaRateMessage, "%s%s%s%s%s%s%s*", preamble, GLLBuf, RMCBuf, VTGBuf, GSABuf, GSVBuf, ReservedBuf);
    sf_gps_checksum_calc(checksumBuf, nmeaRateMessage);

    snprintf(nmeaRateMessage + (messagePreambleLength + 10 + nmeaRateReservedLength), "%s%s", checksumBuf, CRLFBuf); 

    sf_uart_write((u8 *)nmeaRateMessage, nmeaRateMessageLength);
}

void sf_gps_checksum_calc(char *checksumBuf, char *messageString) {
   char *next = messageString[2];
   u16 checksum = messageString[1];
   
   while (*next != '*') {
      checksum ^= next++;
   }

   sprintf(checksumBuf, "%*X\0", 2, checksum);
}

void sf_gps_read(char *readBuf, u32 numBytes) {
   
}
