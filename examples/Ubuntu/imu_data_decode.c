#include <string.h>
#include <stdio.h>

#include "packet.h"
#include "imu_data_decode.h"

static Packet_t RxPkt; /* used for data receive */
static int16_t acc[3];
static int16_t gyr[3];
static int16_t mag[3];
static float eul[3];
static float quat[4];
static uint8_t id;

    
int get_raw_acc(int16_t* a)
{
    memcpy(a, acc, sizeof(acc));
    return strlen((char *)acc);
}

int get_raw_gyr(int16_t* g)
{
    memcpy(g, gyr, sizeof(gyr));
    return strlen((char *)gyr);
}

int get_raw_mag(int16_t* m)
{
    memcpy(m, mag, sizeof(mag));
    return strlen((char *)mag);
}

int get_eul(float* e)
{
    memcpy(e, eul, sizeof(eul));
    return strlen((char *)eul);
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

/*  callback function of  when recv a data frame successfully */
static void OnDataReceived(Packet_t *pkt)
{

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
            case kItemAccCalibrated:
            case kItemAccFiltered:
            case kItemAccLinear:
                memcpy(acc, p + offset + 1, sizeof(acc));
                offset += 7;
                break;
            case kItemgyrRaw:
            case kItemgyrCalibrated:
            case kItemgyrFiltered:
                memcpy(gyr, p + offset + 1, sizeof(gyr));
                offset += 7;
                break;
            case kItemMagRaw:
            case kItemMagCalibrated:
            case kItemMagFiltered:
                memcpy(mag, p + offset + 1, sizeof(mag));
                offset += 7;
                break;
            case kItemRotationeul:
                eul[0] = ((float)(int16_t)(p[offset+1] + (p[offset+2]<<8)))/100;
                eul[1] = ((float)(int16_t)(p[offset+3] + (p[offset+4]<<8)))/100;
                eul[2] = ((float)(int16_t)(p[offset+5] + (p[offset+6]<<8)))/10;
                offset += 7;
                break;
            case kItemRotationeul2:
                memcpy(eul, p + offset + 1, sizeof(eul));
                offset += 13;
                break;
            case kItemRotationQuat:
                memcpy(quat, p + offset + 1, sizeof(quat));
                offset += 17;
                break;
            case kItemPressure:
                offset += 5;
                break;
            case kItemTemperature:
                offset += 5;
                break;
            default:
                printf("data decode wrong\r\n");
				return;
                break;
        }
    }
}

*/
int imu_data_decode_init(void)
{
    Packet_DecodeInit(&RxPkt, OnDataReceived);
    return 0;
}

