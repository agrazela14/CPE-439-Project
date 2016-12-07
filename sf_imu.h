#ifndef SF_IMU_H
#define SF_IMU_H

/* Initializes IMU in NDOF mode */
void sf_imu_init();

/* Gets the Z component of acceleration, mostly for testing */
u16 sf_imu_get_acc_z();

u16 sf_imu_get_acc_y();

u16 sf_imu_get_acc_x();

/* Returns the value of the CALIB_STAT register of the IMU */
u8 sf_imu_get_calib();

/* Returns the compass heading of the imu */
u16 sf_imu_get_heading();

#endif
