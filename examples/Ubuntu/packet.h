#ifndef __PACKET_H__
#define __PACKET_H__

#include <stdint.h>
#include <stdbool.h>

#define MAX_PACKET_LEN          (128)




typedef struct
{
    uint32_t ofs;
    uint8_t buf[MAX_PACKET_LEN];    /* total frame buffer */
    uint16_t payload_len;           
    uint16_t len;                   /* total frame len */
    uint8_t type;
}Packet_t;


/* packet Tx API */
uint32_t Packet_AddData(Packet_t *pkt, uint8_t *buf, uint16_t len);
uint32_t Packet_Begin(Packet_t *pkt);
uint32_t Packet_Final(Packet_t *pkt);
uint32_t Packet_CreatePing(Packet_t *pkt);
uint32_t Packet_CreatePingAck(Packet_t *pkt, uint8_t major, uint8_t minor, uint8_t bugfix, uint16_t option);



/* packet Rx API */
typedef void (*OnDataReceivedEvent)(Packet_t *pkt);
void Packet_DecodeInit(Packet_t *pkt, OnDataReceivedEvent rx_handler);
uint32_t Packet_Decode(uint8_t c);
int printf_num(void);
int time_out(int second);

#endif

