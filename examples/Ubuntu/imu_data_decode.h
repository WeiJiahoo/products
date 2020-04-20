#ifndef __IMU_DATA_DECODE_H__
#define __IMU_DATA_DECODE_H__

#include <stdint.h>
#include <stdbool.h>
extern int frame_count;
extern bool data_tag_flag;
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
	KItemIMU_SOL =              0x91,   /*   */
	KItemGW_SOL =               0x62,   /*   */
}ItemID_t;

int imu_data_decode_init(void);
int get_raw_acc(float* a);
int get_raw_gyr(float* g);
int get_raw_mag(float* m);
int get_id(uint8_t *user_id);
int get_eul(float* e);
int get_quat(float* q);

#endif

 
