/* BNO055 IMU communication library, utilizing the sf_coms protocols
 *
 * Note:	Make sure PS0 and PS1 pins are both pulled LOW to be in
 * 			I2C communication mode on the sensor. Also, verify that
 * 			the address pin is set correctly for what address you use
 */

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "semphr.h"

/* Project Includes */
#include "sf_coms.h"

/* BNO055 I2C address with COM3 pin LOW (ADDR=0x29 with COM4 HIGH) */
#define IMU_IIC_ADDR 0x28

/* IMU Register Address Definitions */
#define OPR_MODE 0x3D
#define UNIT_SEL 0x3B
#define ACC_DATA_Z_MSB 0x0D

/* IMU Mode Defines
 * Note: 7 ms switching from config mode to any other, 19 ms switching
 * 		 from any other mode to config mode.  */
#define IMU_MODE_CONFIG 0x00
#define IMU_MODE_ACC 0x01
#define IMU_MODE_IMU 0x08

/* only puts IMU in accelerometer mode right now for testing */
void sf_imu_init() {
	u16 toSend;

	/* Reg address to write to is first in buffer, followed by value to write */
	toSend = OPR_MODE << sizeof(u8);
	toSend |= IMU_MODE_ACC;

	/* Send off REG address and value to write to it */
	sf_iic_send((u8 *)(&toSend), 2, IMU_IIC_ADDR);
	//xSemaphoreTake(iicSendDone, portMAX_DELAY); /* block until transmission done */
}

/* Gets the Z component of acceleration, mostly for testing */
u16 sf_imu_get_acc_z() {
	u8 regAddr = ACC_DATA_Z_MSB; /* If we read two bytes starting here, will get LSB after MSB */
	u16 recBuff = 0;

	sf_iic_send((u8 *)(&regAddr), 1, IMU_IIC_ADDR);
	//xSemaphoreTake(iicSendDone, portMAX_DELAY); /* wait for above to send */

	sf_iic_receive((u8 *)(&recBuff), 2, IMU_IIC_ADDR);
	//xSemaphoreTake(iicRecDone, portMAX_DELAY); /* wait for receive to finish */

	return recBuff;
}
