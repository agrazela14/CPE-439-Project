#ifndef SF_GPS_H
#define SF_GPS_H

/* The longitude and Latitude for the room we are running this in, to use as a starting point */

#define GPS_START_LONGITUDE -120.66176464
#define GPS_START_LATITUDE 35.30080247 

#define LAT_DEGREE_LEN 2
#define LONG_DEGREE_LEN 3
#define MINUTE_LEN 6

#define DEGREE_LONG_TO_METERS 82019.85
#define DEGREE_LAT_TO_METERS 31780.69

/* Struct for holding a gps position and long/lat accelerations.
 * meant to make passing all of these closely related and needed
 * values around easier */
typedef struct gps_t {
	float longitude;
	float latitude;
	float acc_long;
	float acc_lat;
    float cur_long_p;
    float cur_lat_p;
    float cur_long_v;
    float cur_lat_v;
    float timeslice;
} gps_t;

/* Takes the raw IMU compass bearing, x-axis linear acceleration, and
 * y-axis linear acceleration values and converts them into valid accelerations
 * in the longitudinal and latitudinal directions, placing them into their
 * respective fields of the gps_t struct pointed to by &gps
 *
 * Note that the passed compass bearing is (1 degree / 16 LSBs) and passed
 * accelerations are (1 m/s / 100 LSB) */
void convert_acc(int compass_bearing, float acc_x, float acc_y, gps_t *gps);

/*takes the string version of the latitude longitude from the gps, which is in degrees minutes seconds form
  and turns it into some actual floating points, which are then put into the fields
  gps->latitude
  gps->longitude
  Example of a string that might come in: 4807.038,N,01131.000,E
  48 degree 7.038 minutes North
  11 degree 31.000 minutes East
*/
void convert_lat_long(char *stringVers, gps_t *gps);

//Helper functions
void convert_degree_decimal_to_meter(float *longitude, float *latitude);

void sf_return_to_decimal_degree(float *longitude, float *latitude); 

/* The following lookup tables are for quick calculations of sin and cos,
 * down to a resolution of .25 degrees. Each entry is a step of .25 degrees,
 * starting at 0 and ending at 359.75 (index 1439) */
#define LUT_SIZE 1440

extern const float cos_LUT[LUT_SIZE];
extern const float sin_LUT[LUT_SIZE];

#endif
