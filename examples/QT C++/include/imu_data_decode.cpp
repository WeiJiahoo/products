#include <string.h>
#include <stdio.h>
#include <QDebug>
#include "kptl.h"
#include "imu_data.h"
#include "imu_data_decode.h"


static pkt_dec_t M_dec;
static frame_packet_t rx_pkt;
static imu_data_t imu_data;

/* default rx slave cnt */
/* 需要把 rf_slave_cn 設定為節點的數量，節點 ID 必須依照順序從 0 開始 */


int dump_imu_data(imu_data_t *imu)
{
    *imu=imu_data;
    //printf("id:%d\r\n", imu->id);
    //printf("eul: %f %f %f\r\n", imu->eul[0], imu->eul[1], imu->eul[2]);
    //printf("acc: %d %d %d\r\n", imu->acc[0], imu->acc[1], imu->acc[2]);
    //printf("gyr: %d %d %d\r\n", imu->gyr[0], imu->gyr[1], imu->gyr[2]);
    return 0;
}

int dump_rf_data(imu_data_t *imu)
{
    *imu=imu_data;

    for(int i=0; i<rf_slave_cnt; i++)
        {
            printf("quat[%d]: %f %f %f %f\r\n", i, imu->rf_quat[i][0], imu->rf_quat[i][1], imu->rf_quat[i][2], imu->rf_quat[i][3]);
            printf("acc[%d]: %d %d %d\r\n",  i, imu->rf_acc[i][0], imu->rf_acc[i][1], imu->rf_acc[i][2]);
            printf("gyr[%d]: %d %d %d\r\n",  i, imu->rf_gyr[i][0], imu->rf_gyr[i][1], imu->rf_gyr[i][2]);
            printf("mag[%d]: %d %d %d\r\n",  i, imu->rf_mag[i][0], imu->rf_mag[i][1], imu->rf_mag[i][2]);
            printf("eul[%d]: %d %d %d\r\n",  i, imu->rf_eul[i][0], imu->rf_eul[i][1], imu->rf_eul[i][2]);
            qDebug()<<i<<"---"<<imu->rf_eul[i][0]<<"---"<< imu->rf_eul[i][1]<<"---"<< imu->rf_eul[i][2];
        }
     return 0;
}


typedef struct
{
    uint8_t flag;           /* 0:read  1:write */
    uint8_t rsvd_addr;
    uint8_t param_cnt;
}ext_data_info_t;

/*  callback function of  when recv a data frame successfully */
void imu_data_parse(imu_data_t *dat, uint8_t *buf, uint32_t len)
{
    int offset;
    uint8_t *p;
    ext_data_info_t *ext = NULL;
    p = buf;
    offset = 0;
    
    while(offset < len)
    {
        switch(p[offset])
        {
            case kItemID:
                dat->id = p[1];
                offset += 2;
                break;
            case kItemAccRaw:
            case kItemAccCalibrated:
            case kItemAccFiltered:
            case kItemAccLinear:
                memcpy(dat->acc, p + offset + 1, sizeof(dat->acc));
                offset += 7;
                break;
            case kItemGyrRaw:
            case kItemGyrCalibrated:
            case kItemGyrFiltered:
                memcpy(dat->gyr, p + offset + 1, sizeof(dat->gyr));
                offset += 7;
                break;
            case kItemMagRaw:
            case kItemMagCalibrated:
            case kItemMagFiltered:
                memcpy(dat->mag, p + offset + 1, sizeof(dat->mag));
                offset += 7;
                break;
            case kItemRotationEular:
                dat->eul[0] = ((float)(int16_t)(p[offset+1] + (p[offset+2]<<8)));
                dat->eul[1] = ((float)(int16_t)(p[offset+3] + (p[offset+4]<<8)));
                dat->eul[2] = ((float)(int16_t)(p[offset+5] + (p[offset+6]<<8)));
                offset += 7;
                break;
            case kItemRotationEular2:
                //memcpy(eular, p + offset + 1, sizeof(eular));
                offset += 4*4+1;
                break;
            case kItemRotationQuat:
                memcpy(dat->q, p + offset + 1, sizeof(dat->q));
                offset += 17;
                break;
            case kItemPressure:
                offset += 5;
                break;
            case kItemTemperature:
                memcpy(&dat->temperature, p + offset + 1, sizeof(dat->temperature));
                offset += 5;
                break;
            case kItemExtend:
                ext = (ext_data_info_t*)(p + offset + 1);
            
                /* get rf slave cnt */
                rf_slave_cnt = ext->param_cnt;
                offset += 4;
                break;
            case kItemRFQuat:
                memcpy(&dat->rf_quat[0][0], p + offset + 1, sizeof(dat->rf_quat[0]) * rf_slave_cnt);
                offset += 1 + sizeof(dat->rf_quat[0]) * rf_slave_cnt;
                break;
            case kItemRFEul:

                memcpy(&dat->rf_eul[0][0], p + offset + 1, sizeof(dat->rf_eul[0]) * rf_slave_cnt);
                offset += 1 + sizeof(dat->rf_eul[0]) * rf_slave_cnt;

                break;

            case kItemRFAccCalibrated:
                memcpy(&dat->rf_acc[0][0], p + offset + 1, sizeof(dat->rf_acc[0]) * rf_slave_cnt);
                offset += 1 + sizeof(dat->rf_acc[0]) * rf_slave_cnt;
                break;
            case kItemRFGyrCalibrated:
                memcpy(&dat->rf_gyr[0][0], p + offset + 1, sizeof(dat->rf_gyr[0]) * rf_slave_cnt);
                offset += 1 + sizeof(dat->rf_gyr[0]) * rf_slave_cnt;
                break;
            case kItemRFMagCalibrated:
                memcpy(&dat->rf_mag[0][0], p + offset + 1, sizeof(dat->rf_mag[0]) * rf_slave_cnt);
                offset += 1 + sizeof(dat->rf_mag[0]) * rf_slave_cnt;
                break;
            default:
                //printf("data decode wrong\r\n");
                return;
        }
    }
    
}

void kptl_cb(frame_packet_t *pkt)
{
    imu_data_parse(&imu_data, pkt->payload, kptl_get_payload_len(pkt));
}
int imu_data_decode_init(void)
{

    M_dec.fp = &rx_pkt;
    M_dec.cb = kptl_cb;
    kptl_decode_init(&M_dec);

    return 0;
}
