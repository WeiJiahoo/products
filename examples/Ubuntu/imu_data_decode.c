#include <string.h>
#include <stdio.h>

#include "packet.h"
#include "imu_data_decode.h"



static packet_t RxPkt; /* used for data receive */
/*
 **采用结构体来保存数据
 **将标志位都集中到一个32位的变量上，用位来表示
 **在复制数据时，在用户程序中直接调用一个memcpu函数
 **
 *
 *
 */

int frame_count;

receive_imusol_packet_t receive_imusol;
receive_gwsol_packet_t receive_gwsol;

int get_imu_data(receive_imusol_packet_t *data)
{
	memcpy(data,&receive_imusol,sizeof(receive_imusol_packet_t));
}

int get_gw_data(receive_gwsol_packet_t *data)
{
	memcpy(data,&receive_gwsol,sizeof(receive_gwsol_packet_t));
}

static int stream2int16(int *dest,uint8_t *src)
{
	dest[0] = (int16_t)(src[0] | src[1] << 8);
	dest[1] = (int16_t)(src[2] | src[3] << 8);
	dest[2] = (int16_t)(src[4] | src[5] << 8);	
	return 0;
}   


/*  callback function of  when recv a data frame successfully */
static void on_data_received(packet_t *pkt)
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
			receive_imusol.bitmap = 0;

		case kItemID:
			receive_imusol.bitmap |= 1;
			 receive_imusol.id = p[1];
			offset += 2;
			break;
		case kItemAccRaw:
			receive_imusol.bitmap |= 1 << 1;
			stream2int16(temp, p + offset + 1);
			receive_imusol.acc[0] = (float)temp[0] / 1000;
			receive_imusol.acc[1] = (float)temp[1] / 1000;
			receive_imusol.acc[2] = (float)temp[2] / 1000;
			offset += 7;
			break;
		case kItemGyrRaw:
			receive_imusol.bitmap |= 1 << 2;
			stream2int16(temp, p + offset + 1);
			receive_imusol.gyr[0] = (float)temp[0] / 10;
			receive_imusol.gyr[1] = (float)temp[1] / 10;
			receive_imusol.gyr[2] = (float)temp[2] / 10;
			offset += 7;
			break;
		case kItemMagRaw:
			receive_imusol.bitmap |= 1 << 3;
			stream2int16(temp, p + offset + 1);
			receive_imusol.mag[0] = (float)temp[0] / 10;
			receive_imusol.mag[1] = (float)temp[1] / 10;
			receive_imusol.mag[2] = (float)temp[2] / 10;
			offset += 7;
			break;
		case kItemRotationEul:
			receive_imusol.bitmap |= 1 << 4;
			stream2int16(temp, p + offset + 1);
			receive_imusol.eul[1] = (float)temp[0] / 100;
			receive_imusol.eul[0] = (float)temp[1] / 100;
			receive_imusol.eul[2] = (float)temp[2] / 10;
			offset += 7;
			break;
		case kItemRotationQuat:
			receive_imusol.bitmap |= 1 << 5;
			memcpy(receive_imusol.quat, p + offset + 1, sizeof( receive_imusol.quat));
			offset += 17;
			break;
		case kItemPressure:
			offset += 5;
			break;

		case KItemIMUSOL:
			receive_imusol.bitmap |= 0x1f;

			receive_imusol.id =p[offset + 1];
	
			memcpy(receive_imusol.acc, p + 12, sizeof( receive_imusol.acc));
			memcpy(receive_imusol.gyr, p + 24, sizeof( receive_imusol.gyr));
			memcpy(receive_imusol.mag, p + 36, sizeof( receive_imusol.mag));
			memcpy(receive_imusol.eul, p + 48, sizeof( receive_imusol.eul));
			memcpy(receive_imusol.quat, p + 60, sizeof( receive_imusol.quat));
			offset += 76;
			break;
		case KItemGWSOL:

			receive_gwsol.tag = p[offset];
			receive_gwsol.target_id = p[offset + 1];
			receive_gwsol.node_total = p[offset + 2];
			offset += 8;
			for (int i = 0; i < receive_gwsol.node_total; i++)
			{
				receive_gwsol.receive_imusol[i].bitmap |= 0x1f;
				receive_gwsol.receive_imusol[i].tag = p[offset];
				receive_gwsol.receive_imusol[i].id = p[offset + 1];
				memcpy(receive_gwsol.receive_imusol[i].acc, p + offset + 12, sizeof(receive_imusol.acc));
				memcpy(receive_gwsol.receive_imusol[i].gyr, p + offset + 24, sizeof(receive_imusol.gyr));
				memcpy(receive_gwsol.receive_imusol[i].mag, p + offset + 36, sizeof(receive_imusol.mag));
				memcpy(receive_gwsol.receive_imusol[i].eul, p + offset + 48, sizeof(receive_imusol.eul));
				memcpy(receive_gwsol.receive_imusol[i].quat, p + offset + 60, sizeof(receive_imusol.quat));
				offset += 76;
			}
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
    packet_decode_init(&RxPkt, on_data_received);
    return 0;
}



