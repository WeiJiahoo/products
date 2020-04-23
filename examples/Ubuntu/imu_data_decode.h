#ifndef __IMU_DATA_DECODE_H__
#define __IMU_DATA_DECODE_H__

#include <stdint.h>
#include <stdbool.h>

#define BIT_VALID_ID	(0)
#define BIT_VALID_ACC	(1)
#define BIT_VALID_GYR	(2)
#define BIT_VALID_MAG	(3)
#define BIT_VALID_EUL	(4)
#define BIT_VALID_QUAT	(5)

extern int frame_count;


typedef struct
{
	uint8_t id;
	uint8_t bitmap;		/* each bit indicate data valid or not: bit0:acc  bit1:gyr,  bit2:mag */
	float acc[3];
	float gyr[3];
	float mag[3];
	float eul[3];
	float quat[4];
}imt_data_t;

typedef enum 
{
    kItemID =                   0x90,   /* user programed ID   */
    kItemAccRaw =               0xA0,   /* raw acc             */
    kItemGyrRaw =               0xB0,   /* raw gyro            */  
    kItemMagRaw =               0xC0,   /* raw mag             */
    kItemRotationEul =          0xD0,   /* eular angle         */
    kItemRotationQuat =         0xD1,   /* att q               */
    kItemPressure =             0xF0,   /* pressure            */
    kItemEnd =                  0x00,   
	KItemIMUSOL =              0x91,   /* IMUSOL  */
	KItemGWSOL =               0x62,   /* RFSOL  */
}ItemID_t;

int imu_data_decode_init(void);
//int get_raw_acc(float* a);
//int get_frame_count(void);
//int get_raw_gyr(float* g);
//int get_raw_mag(float* m);
//int get_id(uint8_t *user_id);
//int get_eul(float* e);
//int get_quat(float* q);

int get_imu_data(imt_data_t *data);

#endif

 
