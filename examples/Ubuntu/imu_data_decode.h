#ifndef __IMU_DATA_DECODE_H__
#define __IMU_DATA_DECODE_H__

#include <stdint.h>
#include <stdbool.h>

typedef enum 
{
    kItemID =                   0x90,   /* user programed ID    size: 1 */
    kItemAccRaw =               0xA0,   /* raw acc              size: 3x2 */
    kItemGyrRaw =               0xB0,   /* raw gyro             size: 3x2 */  
    kItemMagRaw =               0xC0,   /* raw mag              size: 3x2 */
    kItemRotationEul =        0xD0,   /* eular angle          size:3x2 */
    kItemRotationEul2 =       0xD9,   /* new eular angle      size:3x4 */
    kItemRotationQuat =         0xD1,   /* att q,               size:4x4 */
    kItemPressure =             0xF0,   /* pressure             size:1x4 */
    kItemEnd =                  0x00,   
}ItemID_t;

int imu_data_decode_init(void);
int get_raw_acc(int16_t* a);
int get_raw_gyr(int16_t* g);
int get_raw_mag(int16_t* m);
int get_id(uint8_t *user_id);
int get_eul(float* e);
int get_quat(float* q);

#endif

 
