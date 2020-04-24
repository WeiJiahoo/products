#ifndef __IMU_DATA_DECODE_H__
#define __IMU_DATA_DECODE_H__

#include <stdint.h>
#include <stdbool.h>
#define MAX_LENGTH 16

#define BIT_VALID_ID	(0)
#define BIT_VALID_ACC	(1)
#define BIT_VALID_GYR	(2)
#define BIT_VALID_MAG	(3)
#define BIT_VALID_EUL	(4)
#define BIT_VALID_QUAT	(5)

extern int frame_count;

<<<<<<< HEAD
#define BIT_VALID_ID   (0)
#define BIT_VALID_ACC  (1)
#define BIT_VALID_GYR  (2)
#define BIT_VALID_MAG  (3)
#define BIT_VALID_EUL  (4)
#define BIT_VALID_QUAT (5)

typedef struct receive_imusol_packet_t {
	uint8_t tag;
	uint8_t id;
	uint8_t bitmap;
	uint8_t	resereve[10];

=======

typedef struct
{
	uint8_t id;
	uint8_t bitmap;		/* each bit indicate data valid or not: bit0:acc  bit1:gyr,  bit2:mag */
>>>>>>> fd06b4be97443ee993410c66ccfe70d59c711947
	float acc[3];
	float gyr[3];
	float mag[3];
	float eul[3];
	float quat[4];
<<<<<<< HEAD

}receive_imusol_packet_t;

typedef struct receive_gwsol_packet_t {
	uint8_t tag;
	uint8_t bitmap;
	uint8_t target_id;
	uint8_t node_total;
	uint8_t reserve[5];
	receive_imusol_packet_t receive_imusol[MAX_LENGTH];
}receive_gwsol_packet_t;
	
=======
}imt_data_t;

>>>>>>> fd06b4be97443ee993410c66ccfe70d59c711947
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
	KItemIMUSOL =               0x91,   /* IMUSOL  */
	KItemGWSOL =                0x62,   /* RFSOL  */
}ItemID_t;

int imu_data_decode_init(void);
<<<<<<< HEAD
int get_imu_data(receive_imusol_packet_t *data);
int get_gw_data(receive_gwsol_packet_t *data);
=======
//int get_raw_acc(float* a);
//int get_frame_count(void);
//int get_raw_gyr(float* g);
//int get_raw_mag(float* m);
//int get_id(uint8_t *user_id);
//int get_eul(float* e);
//int get_quat(float* q);

int get_imu_data(imt_data_t *data);
>>>>>>> fd06b4be97443ee993410c66ccfe70d59c711947

#endif

 
