#include <string.h>
#include <stdio.h>

#include "packet.h"
#include "imu_data_decode.h"

<<<<<<< HEAD
=======

//bool acc_tag_flag = false;
//bool gyr_tag_flag = false;
//bool mag_tag_flag = false;
//bool eul_tag_flag = false;
//bool quat_tag_flag = false;
//bool imu_tag_flag = false;
//bool gw_tag_flag = false;

>>>>>>> fd06b4be97443ee993410c66ccfe70d59c711947


static Packet_t RxPkt; /* used for data receive */
/*
 **采用结构体来保存数据
 **将标志位都集中到一个32位的变量上，用位来表示
 **在复制数据时，在用户程序中直接调用一个memcpu函数
 **
 *
 *
 */

<<<<<<< HEAD
int frame_count;
=======


static imu_data_t imu_data;

int frame_count;
    
int get_imu_data(imt_data_t *data)
{
	
}
	
	
int get_frame_count(void)
{
	return frame_count;
}
>>>>>>> fd06b4be97443ee993410c66ccfe70d59c711947

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

static int stream2int16(int *dest, uint8_t *src)
{
	dest[0] = (int16_t)(src[0 + 0] | src[0 + 1] << 8);
	dest[1] = (int16_t)(src[0 + 2] | src[0 + 3] << 8);
	dest[2] = (int16_t)(src[0 + 4] | src[0 + 5] << 8);	
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
<<<<<<< HEAD
			receive_imusol.bitmap |= 1 << 3;
			stream2int16(temp, p + offset + 1);
			receive_imusol.mag[0] = (float)temp[0] / 10;
			receive_imusol.mag[1] = (float)temp[1] / 10;
			receive_imusol.mag[2] = (float)temp[2] / 10;
=======
			mag_tag_flag = true;
			stream2int16(temp, p + offset + 1);
			mag[0] = (float)temp[0] / 10;
			mag[1] = (float)temp[1] / 10;
			mag[2] = (float)temp[2] / 10;
>>>>>>> fd06b4be97443ee993410c66ccfe70d59c711947
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
			/*
			 *创建五个数组，acc[12 x 16].....
			 *从数组中打印数据
			 *屏幕上显示连接到的节点数据
			 *节点编号0-15
			 *  0	1	2	3
			 *  4	5	6	7
			 *  8	9	10	11
			 *  12	13	14	15
			 *
			 */

		//	receive_id
		//		node_total
				switch(p[offset])
				{
					case KItemIMUSOL:
						break;

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
    Packet_DecodeInit(&RxPkt, OnDataReceived);
    return 0;
}



