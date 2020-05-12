#ifndef __IMU_DATA_H__
#define __IMU_DATA_H__

#include <stdint.h>
#include <stdbool.h>

typedef enum 
{
    kItemTest8F = 0x60,
    kItemIMU = 0x70,   /* 3 x 3 x sizeof(float) */
    kItemRFQuat =               0x71,   /* 4*16 float quat */
    kItemRFEul =                0x72,
    kItemRFAccCalibrated =      0x75,
    kItemRFGyrCalibrated =      0x78,
    kItemRFMagCalibrated =      0x7A,
    kItemRFTemperature =        0x7C,
    kItemExtend =               0x61,   /* extend data */
    kItemCPUTimeStamp =         0x8A,   /* uint32_t x 1  time stamp CPU tick time in ms */
    kItemID =                   0x90,   /* user programed ID    size: 1 */
    kItemAccRaw =               0xA0,   /* raw acc              size: 3x2 */
    kItemAccCalibrated =        0xA1,
    kItemAccFiltered =          0xA2,   
    kItemAccLinear =            0xA5,
    kItemAccGravity =           0xA6,
    kItemAccNorm =              0xA8,
    kItemGyrRaw =               0xB0,   /* raw gyro             size: 3x2 */  
    kItemGyrCalibrated =        0xB1,
    kItemGyrFiltered =          0xB2, 
    kItemGyrNorm =              0xB8,
    kItemMagRaw =               0xC0,   /* raw mag              size: 3x2 */
    kItemMagCalibrated =        0xC1,
    kItemMagFiltered =          0xC2,
    kItemRotationEular =        0xD0,   /* eular angle          size:3x2 */
    kItemRotationEular2 =       0xD9,   /* new eular angle      size:3x4 */
    kItemRotationQuat =         0xD1,   /* att q,               size:4x4 */
    kItemTemperature =          0xE0,   
    kItemPressure =             0xF0,   /* pressure             size:1x4 */
    kItemEnd =                  0x00,   
}ItemID_t;


typedef struct
{
    ItemID_t        item;
    const char      *name;
}imu_item_t;

const char *imu_data_get_name(ItemID_t item);

#endif

