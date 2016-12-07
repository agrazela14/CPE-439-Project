#ifndef SF_GPS_H
#define SF_GPS_H

/* Struct for holding a gps position and long/lat accelerations.
 * meant to make passing all of these closely related and needed
 * values around easier */
typedef struct gps_t {
	float longitude;
	float latittude;
	float acc_long;
	float acc_lat;
} gps_t;

/* Takes the raw IMU compass bearing, x-axis linear acceleration, and
 * y-axis linear acceleration values and converts them into valid accelerations
 * in the longitudinal and latitudinal directions, placing them into their
 * respective fields of the gps_t struct pointed to by &gps
 *
 * Note that the passed compass bearing is (1 degree / 16 LSBs) and passed
 * accelerations are (1 m/s / 100 LSB) */
void convert_acc(int compass_bearing, float acc_x, float acc_y, gps_t *gps);

/* The following lookup tables are for quick calculations of sin and cos,
 * down to a resolution of .25 degrees. Each entry is a step of .25 degrees,
 * starting at 0 and ending at 359.75 (index 1439) */
#define LUT_SIZE 1440

extern const float cos_LUT[LUT_SIZE];
extern const float sin_LUT[LUT_SIZE];

#endif
