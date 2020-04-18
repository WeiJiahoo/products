#ifndef __IMU_DATA_DECODE_H__
#define __IMU_DATA_DECODE_H__

#include <stdint.h>
#include <stdbool.h>


#define MAX_SLV_CNT 8

typedef struct
{
    uint8_t id;
    float       q[4];
    int16_t     acc[3];
    int16_t     gyr[3];
    int16_t     mag[3];
    float       eul[3];
    float       temperature;
    float       rf_quat[MAX_SLV_CNT][4];
    int16_t     rf_acc[MAX_SLV_CNT][3];
    int16_t     rf_gyr[MAX_SLV_CNT][3];
    int16_t     rf_mag[MAX_SLV_CNT][3];
    int16_t     rf_eul[MAX_SLV_CNT][3];
}imu_data_t;

static uint8_t rf_slave_cnt = 2;

void imu_data_parse(imu_data_t *imu, uint8_t *buf, uint32_t len);
int dump_imu_data(imu_data_t *imu);
int dump_rf_data(imu_data_t *imu);
int imu_data_decode_init(void);


#endif


