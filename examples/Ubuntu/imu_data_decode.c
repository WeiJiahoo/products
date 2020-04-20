#include <string.h>
#include <stdio.h>

#include "packet.h"
#include "imu_data_decode.h"

static Packet_t RxPkt; /* used for data receive */
#if 0

#else
static float acc[3];
static float gyr[3];
static float mag[3];
#endif

static float eul[3];
static float quat[4];
static uint8_t id;

    
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
    return strlen((char *)quat);
}

int get_id(uint8_t *user_id)
{
    *user_id = id;
	
    return 0;
}
int frame_count;

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
		case kItemID:
			id = p[1];
			offset += 2;
			break;
		case kItemAccRaw:
			temp[0] = (int16_t)(p[offset + 1] | p[offset + 2] << 8);
			temp[1] = (int16_t)(p[offset + 3] | p[offset + 4] << 8);
			temp[2] = (int16_t)(p[offset + 5] | p[offset + 6] << 8);

			acc[0] = (float)temp[0] / 1000;
			acc[1] = (float)temp[1] / 1000;
			acc[2] = (float)temp[2] / 1000;
			offset += 7;
			break;
		case kItemGyrRaw:
			temp[0] = (int16_t)(p[offset + 1] | p[offset + 2] << 8);
			temp[1] = (int16_t)(p[offset + 3] | p[offset + 4] << 8);
			temp[2] = (int16_t)(p[offset + 5] | p[offset + 6] << 8);
			gyr[0] = (float)temp[0] / 10;
			gyr[1] = (float)temp[1] / 10;
			gyr[2] = (float)temp[2] / 10;
			offset += 7;
			break;
		case kItemMagRaw:
			temp[0] = (int16_t)(p[offset + 1] | p[offset + 2] << 8);
			temp[1] = (int16_t)(p[offset + 3] | p[offset + 4] << 8);
			temp[2] = (int16_t)(p[offset + 5] | p[offset + 6] << 8);
			mag[0] = (float)temp[0] / 10;
			mag[1] = (float)temp[1] / 10;
			mag[2] = (float)temp[2] / 10;

			offset += 7;
			break;
		case kItemRotationEul:
			temp[0] = (int16_t)(p[offset + 1] | p[offset + 2] << 8);
			temp[1] = (int16_t)(p[offset + 3] | p[offset + 4] << 8);
			temp[2] = (int16_t)(p[offset + 5] | p[offset + 6] << 8);
			eul[1] = (float)temp[0] / 100;
			eul[0] = (float)temp[1] / 100;
			eul[2] = (float)temp[2] / 10;
			offset += 7;
			break;
		case kItemRotationQuat:
			memcpy(quat, p + offset + 1, sizeof(quat));
			offset += 17;
			break;
		case kItemPressure:
			offset += 5;
			break;
		case KItemIMU_SOL:
			id =p[offset + 1];
	
			memcpy(acc,p + 12,sizeof(acc));
			memcpy(gyr,p + 24,sizeof(gyr));
			memcpy(mag,p + 36,sizeof(mag));
			memcpy(eul,p + 48,sizeof(eul));
			memcpy(quat, p + 60, sizeof(quat));
			
			offset += 76;
			break;
		case KItemGW_SOL:
			
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

