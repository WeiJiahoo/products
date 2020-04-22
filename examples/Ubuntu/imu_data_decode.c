#include <string.h>
#include <stdio.h>

#include "packet.h"
#include "imu_data_decode.h"

bool acc_tag_flag = false;
bool gyr_tag_flag = false;
bool mag_tag_flag = false;
bool eul_tag_flag = false;
bool quat_tag_flag = false;
bool imu_tag_flag = false;
bool gw_tag_flag = false;


static Packet_t RxPkt; /* used for data receive */

static float acc[3];
static float gyr[3];
static float mag[3];

static float eul[3];
static float quat[4];
static uint8_t id;
int frame_count;
    
	
int get_frame_count(void)
{
	return frame_count;
}

int get_raw_acc(float* a)
{
    memcpy(a, acc, sizeof(acc));
    return 0;
}

int get_raw_gyr(float* g)
{
    memcpy(g, gyr, sizeof(gyr));
    return 0;
}

int get_raw_mag(float* m)
{
    memcpy(m, mag, sizeof(mag));
    return 0;
}

int get_eul(float* e)
{
    memcpy(e, eul, sizeof(eul));
    return 0;
}

int get_quat(float* q)
{
    memcpy(q, quat, sizeof(quat));
    return 0;
}

int get_id(uint8_t *user_id)
{
    *user_id = id;
	
    return 0;
}


/*  callback function of  when recv a data frame successfully */
static void OnDataReceived(Packet_t *pkt)
{
	int temp[3] = {0};

	frame_count++;
	if(pkt->type != 0xA5)
    {
        return;
    }

    int offset = 0;
    uint8_t *p = pkt->buf;
	while(offset < pkt->payload_len)
	{
		switch(p[offset])
		{
			acc_tag_flag = false;
			gyr_tag_flag = false;
			mag_tag_flag = false;
			eul_tag_flag = false;
			quat_tag_flag = false;

		case kItemID:
			id = p[1];
			offset += 2;
			break;
		case kItemAccRaw:
			acc_tag_flag = true;
			stream2int16(temp, p, offset);
			acc[0] = (float)temp[0] / 1000;
			acc[1] = (float)temp[1] / 1000;
			acc[2] = (float)temp[2] / 1000;
			offset += 7;
			break;
		case kItemGyrRaw:
			gyr_tag_flag = true;
			stream2int16(temp, p, offset);
			gyr[0] = (float)temp[0] / 10;
			gyr[1] = (float)temp[1] / 10;
			gyr[2] = (float)temp[2] / 10;
			offset += 7;
			break;
		case kItemMagRaw:
			mag_tag_flag = true;
			stream2int16(temp, p, offset);
			mag[0] = (float)temp[0] / 10;
			mag[1] = (float)temp[1] / 10;
			mag[2] = (float)temp[2] / 10;
			offset += 7;
			break;
		case kItemRotationEul:
			eul_tag_flag = true;
			stream2int16(temp, p, offset);
			eul[1] = (float)temp[0] / 100;
			eul[0] = (float)temp[1] / 100;
			eul[2] = (float)temp[2] / 10;
			offset += 7;
			break;
		case kItemRotationQuat:
			quat_tag_flag = true;
			memcpy(quat, p + offset + 1, sizeof(quat));
			offset += 17;
			break;
		case kItemPressure:
			offset += 5;
			break;
		case KItemIMUSOL:
			acc_tag_flag = true;
			gyr_tag_flag = true;
			mag_tag_flag = true;
			eul_tag_flag = true;
			quat_tag_flag = true;

			id =p[offset + 1];
	
			memcpy(acc,p + 12,sizeof(acc));
			memcpy(gyr,p + 24,sizeof(gyr));
			memcpy(mag,p + 36,sizeof(mag));
			memcpy(eul,p + 48,sizeof(eul));
			memcpy(quat, p + 60, sizeof(quat));
			offset += 76;
			break;
		case KItemGWSOL:
			gw_tag_flag = true;
			break;
		default:
			printf("data decode wrong\r\n");
			return;
			break;
		}
    }
}


int imu_data_decode_init(void)
{
    Packet_DecodeInit(&RxPkt, OnDataReceived);
    return 0;
}


int stream2int16(int *dest,uint8_t *src,int offset)
{
	dest[0] = (int16_t)(src[offset + 1] | src[offset + 2] << 8);
	dest[1] = (int16_t)(src[offset + 3] | src[offset + 4] << 8);
	dest[2] = (int16_t)(src[offset + 5] | src[offset + 6] << 8);	
	return 0;
}
