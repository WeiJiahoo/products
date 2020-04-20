#include <string.h>
#include <stdio.h>

#include "kptl.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif

#ifndef CH_OK
#define CH_OK   (0)
#endif

#ifndef CH_ERR
#define CH_ERR  (1)
#endif









/* get the patyload size in a frame packet */
uint32_t kptl_get_payload_len(frame_packet_t *p)
{
    return ARRAY2INT16(p->len);
}

/* generate CRC16
    @param  currectCrc:     previous buffer pointer, if is a new start, pointer should refer a zero uint16_t
    @param  src:            current buffer pointer
    @param  lengthInBytes:  length of current buf
*/
void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j=0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *currectCrc = crc;
}

void kptl_create_ping(packet_ping_t *p)
{
    p->start_byte = kFramingPacketStartByte;
    p->packet_type = kFramingPacketType_Ping;
}

void kptl_create_ack(packet_ack_t *p)
{
    p->start_byte = kFramingPacketStartByte;
    p->packet_type = kFramingPacketType_Ack;
}

void kptl_create_nak(packet_nak_t *p)
{
    p->start_byte = kFramingPacketStartByte;
    p->packet_type = kFramingPacketType_Nak;
}
    
void kptl_create_cmd_packet(frame_packet_t *fp, cmd_hdr_t *cp, uint32_t *param)
{
    int i;
    kptl_frame_packet_begin(fp, kFramingPacketType_Command);
    kptl_frame_packet_add(fp, (uint8_t*)cp, 4);
    for(i=0; i<cp->param_cnt; i++)
    {
        kptl_frame_packet_add(fp, (uint8_t *)&param[i], sizeof(uint32_t));
    }
    
    kptl_frame_packet_final(fp);
}

uint32_t kptl_cmd_packet_get_size(cmd_hdr_t *cp)
{
    return 4 + cp->param_cnt*sizeof(uint32_t); 
}

uint32_t kptl_create_generic_resp_packet(frame_packet_t *fp, uint32_t status_code, uint32_t cmd_tag)
{
    uint32_t param[2];
    cmd_hdr_t cp;
    
    cp.tag = kCommandTag_GenericResponse;
    cp.flags = 0x00;
    cp.param_cnt = 2;
    cp.reserved = 0x00;
    
    param[0] = status_code;
    param[1] = cmd_tag;
    kptl_create_cmd_packet(fp, &cp, param);

    return CH_OK;
}

uint32_t kptl_create_property_resp_packet(frame_packet_t *p, uint8_t param_cnt, uint32_t *param)
{
    cmd_hdr_t cp;
    cp.tag = kCommandTag_GetPropertyResponse;
    cp.flags = 0x00;
    cp.reserved = 0x00;
    cp.param_cnt = param_cnt;

    kptl_create_cmd_packet(p, &cp, param);

    return CH_OK;
}

void kptl_create_ping_resp_packet(ping_resp_packet_t *p, uint8_t major, uint8_t minor, uint8_t bugfix, uint8_t opt_low, uint8_t opt_high)
{
    p->hr.start_byte = kFramingPacketStartByte;
    p->hr.packet_type = kFramingPacketType_PingResponse;
    
    p->bug_fix = bugfix;
    p->ver_minor = minor;
    p->ver_major  = major;
    p->protocol_name = 'P';
    p->option_low = opt_low;
    p->option_high = opt_high;
    
    /* crc */
    uint16_t crc;
    crc = 0;
    crc16_update(&crc, (uint8_t*)&p->hr, 8);
    p->crc16[0] = (crc & 0x00FF)>>0;
    p->crc16[1] = (crc & 0xFF00)>>8;
}

uint32_t kptl_frame_packet_begin(frame_packet_t *p, uint8_t frame_type)
{
    p->hr.start_byte = kFramingPacketStartByte;
    p->hr.packet_type = frame_type;
    p->len[0] = 0;
    p->len[1] = 0;
    p->crc16[0] = 0;
    p->crc16[1] = 0;
    memset(p->payload, 0, sizeof(p->payload));
    return CH_OK;
}

uint32_t kptl_frame_packet_add(frame_packet_t *p, void *buf, uint16_t len)
{
    /* add item content into buffer */
    
    if(kptl_get_payload_len(p) > MAX_PACKET_LEN)
    {
        return CH_ERR;
    }
    
    memcpy(p->payload + kptl_get_payload_len(p), buf, len);
    p->len[0] += (len >>0) & 0xFF;
    p->len[1] += (len >>8) & 0xFF;
    return CH_OK;
}

uint32_t kptl_frame_packet_final(frame_packet_t *p)
{
    
    /* crc */
    uint16_t crc;
    crc = 0;
    crc16_update(&crc, (uint8_t*)&p->hr, 2);
    crc16_update(&crc, (uint8_t*)p->len, 2);
    crc16_update(&crc, (uint8_t*)p->payload, kptl_get_payload_len(p));
    
    p->crc16[0] = (crc & 0x00FF) >> 0;
    p->crc16[1] = (crc & 0xFF00) >> 8;
    return CH_OK;
}

/* get total frame packet size */
uint32_t kptl_get_frame_size(frame_packet_t *p)
{
    return p->len[0] + (p->len[1]<<8) + 6;
}

enum status
{
    kStatus_Idle,
    kStatus_Cmd,
    kStatus_LenLow,
    kStatus_LenHigh,
    kStatus_CRCLow,
    kStatus_CRCHigh,
    kStatus_Data,
};

 /**
 * @brief  初始化姿態解碼模塊
 * @note   完成初始化一個引腳配置
 * @param  pkt 接收包指針
 * @param  接收成功回調函數
 * @code

 *      void OnDataReceived(frame_packet_t *pkt)
 *      {
 *          pkt->payload 為數據 pkt->payload_len 為接收到的字節長度 
 *      }
 *
 *      frame_packet_t pkt;
 *      Packet_DecodeInit(&pkt, OnDataReceived);
 * @endcode
 * @retval None
 */

static pkt_dec_t *O_dec;

int kptl_decode_init(pkt_dec_t *d)
{

    d->cnt = 0;
    d->status = kStatus_Idle;
    if(!d->fp)
    {
        return 1;
    }
    memset(d->fp, 0, sizeof(frame_packet_t));

    O_dec=d;

    return 0;
}


    
 /**
 * @brief  decode any type of packet
 * @note   once a pcaket is decoded, the cb function will be called
 * @param  d: decode handle, c: one byte received
 * @retval CH_OK
 */


#define SAFE_CALL_CB    if(O_dec->cb) O_dec->cb(p)

uint32_t kptl_decode(uint8_t c)
{
    int ret = CH_ERR;
    uint16_t crc_calculated = 0;          /* CRC value caluated from a frame */
    frame_packet_t *p = O_dec->fp;
    uint8_t *payload_buf = (uint8_t*)O_dec->fp->payload;
    
    switch(O_dec->status)
    {
        case kStatus_Idle:
            if(c == kFramingPacketStartByte)
            {
                O_dec->status = kStatus_Cmd;
                p->hr.start_byte = c;
            }
            break;
        case kStatus_Cmd:
            p->hr.packet_type = c;
            switch(c)
            {
                case kFramingPacketType_Command:
                    O_dec->status = kStatus_LenLow;
                    break;
                case kFramingPacketType_Data:
                    O_dec->status = kStatus_LenLow;
                    break;
                case kFramingPacketType_Ping:
                    SAFE_CALL_CB;
                    ret = CH_OK;
                    O_dec->status = kStatus_Idle;
                    break;
                case kFramingPacketType_PingResponse:
                    p->len[0] = 0;
                    p->len[1] = 0;
                    O_dec->cnt = 0;
                    O_dec->status = kStatus_Data;
                    break;
                case kFramingPacketType_Ack:
                case kFramingPacketType_Nak:
                    O_dec->status = kStatus_Idle;
                    SAFE_CALL_CB;
                    return CH_OK;
            }
            break;
        case kStatus_LenLow:
            p->len[0] = c;
            O_dec->status = kStatus_LenHigh;
            break;
        case kStatus_LenHigh:
            p->len[1] = c;
            if(kptl_get_payload_len(p) <= MAX_PACKET_LEN)
            {
                O_dec->status = kStatus_CRCLow;
            }
            else
            {
                O_dec->status = kStatus_Idle;
            }
            break;
        case kStatus_CRCLow:
            p->crc16[0] = c;
            O_dec->status = kStatus_CRCHigh;
            break;
        case kStatus_CRCHigh:
            p->crc16[1] = c;
            O_dec->cnt = 0;
            O_dec->status = kStatus_Data;
            break;
        case kStatus_Data:
            payload_buf[O_dec->cnt++] = c;
                   
            if((p->hr.packet_type == kFramingPacketType_Command || p->hr.packet_type == kFramingPacketType_Data) && O_dec->cnt >= kptl_get_payload_len(p))
            {
                /* calculate CRC */
                crc_calculated = 0;
                crc16_update(&crc_calculated, (uint8_t*)&p->hr, 2);
                crc16_update(&crc_calculated, p->len, 2);
                crc16_update(&crc_calculated, payload_buf, O_dec->cnt);
                
                /* CRC match */
                if(crc_calculated == ARRAY2INT16(p->crc16))
                {
                    SAFE_CALL_CB;
                    ret = CH_OK;
                }
                O_dec->status = kStatus_Idle;
            }
            
            if(p->hr.packet_type == kFramingPacketType_PingResponse && O_dec->cnt >= 8) /* ping response */
            {
                p->len[0] = 8;
                p->len[1] = 0;
                SAFE_CALL_CB;
                O_dec->status = kStatus_Idle;
            }
            
            break;
        default:
            O_dec->status = kStatus_Idle;
            break;
    }
    return ret;
}

