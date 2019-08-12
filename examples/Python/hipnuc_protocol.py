#!/usr/bin/env python3
# -*- coding: utf-8 -*-

' hipnuc protocol module '

import threading
import struct
import datetime

class HipnucFrame_Exception(Exception):
    def __init__(self,err='HI221GW Frame Error'):
        Exception.__init__(self,err)

class HipnucFrame_NoValid_Exception(HipnucFrame_Exception):
    def __init__(self,err='No valid frame received'):
        Exception.__init__(self,err)

class HipnucFrame_NotCompleted_Exception(HipnucFrame_Exception):
    def __init__(self,err='No full frame received'):
        Exception.__init__(self,err)

class HipnucFrame_ErrorFrame_Exception(HipnucFrame_Exception):
    def __init__(self,err='Error frame'):
        Exception.__init__(self,err)


def _parse_data_packet_0x61(data_section:list,node_num = None):
    HI221GW_property = {
        "GWID": data_section[1],
        "CNT": data_section[2]
    }
    return HI221GW_property

def _parse_data_packet_0x71(data_section:list,node_num = None):
    quaternion_list = []

    for pos in range(node_num):
        t_pos = pos * 16
        W = float(struct.unpack("<f", bytes(data_section[t_pos:t_pos + 4]))[0])
        t_pos += 4
        X = float(struct.unpack("<f", bytes(data_section[t_pos:t_pos + 4]))[0])
        t_pos += 4
        Y = float(struct.unpack("<f", bytes(data_section[t_pos:t_pos + 4]))[0])
        t_pos += 4
        Z = float(struct.unpack("<f", bytes(data_section[t_pos:t_pos + 4]))[0])

        temp_dic = {
            "W":W,
            "X":X,
            "Y":Y,
            "Z":Z
        }
        quaternion_list.append(temp_dic)

    quaternion = {
        "quat":quaternion_list
    }

    return quaternion

def _parse_data_packet_0x75(data_section:list,node_num = None):
    acc_list = []

    for pos in range(node_num):
        t_pos = pos * 6
        X = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
        t_pos += 2
        Y = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
        t_pos += 2
        Z = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])

        temp_dic = {
            "X":X,
            "Y":Y,
            "Z":Z
        }
        acc_list.append(temp_dic)

    acc = {
        "acc":acc_list
    }

    return acc

def _parse_data_packet_0x78(data_section:list,node_num = None):
    gyr_list = []

    for pos in range(node_num):
        t_pos = pos * 6
        X = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
        t_pos += 2
        Y = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
        t_pos += 2
        Z = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])

        temp_dic = {
            "X":X,
            "Y":Y,
            "Z":Z
        }
        gyr_list.append(temp_dic)

    gyr = {
        "gyr":gyr_list
    }

    return gyr

data_packet_properties = {
    0x61:{
        "type":"Expanding Information",
        "id_len":1,
        "data_len":3,
        "parse method":_parse_data_packet_0x61
    },
    # quat
    0x71:{
        "type":"quat",
        "id_len":1,
        "data_len":16,
        "parse method":_parse_data_packet_0x71
    },
    # acc
    0x75:{
        "type":"acc",
        "id_len": 1,
        "data_len": 6,
        "parse method": _parse_data_packet_0x75
    },
    #gyr
    0x78:{
        "type":"gyr",
        "id_len": 1,
        "data_len": 6,
        "parse method": _parse_data_packet_0x78
    }
}

def crc16_update(buffer_list, cal_len, cal_pos, crc=0):
    for temp_j in range(cal_len):
        byte = buffer_list[temp_j + cal_pos]
        crc ^= byte << 8
        crc &= 0xffffffff
        for temp_i in range(8):
            temp = crc << 1
            temp &= 0xffffffff
            if (crc & 0x8000):
                temp ^= 0x1021
                temp &= 0xffffffff
            crc = temp

    return (crc & 0xffff)

SampleRate = 0
SamplesReceived = 0
prevSamplesReceived = 0
sample_rate_alive_flag = True

def sample_rate_timer_cb(sample_timer):
    global SampleRate,SamplesReceived,prevSamplesReceived,sample_rate_alive_flag

    SampleRate = SamplesReceived - prevSamplesReceived
    prevSamplesReceived = SamplesReceived
    print("每秒帧率：",SampleRate,datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'))

    if sample_rate_alive_flag == True:
        sample_timer = threading.Timer(1.00, sample_rate_timer_cb,args=(sample_timer,))
        sample_timer.start()

def sample_rate_timer_close():
    global sample_rate_alive_flag
    sample_rate_alive_flag = False

# 找到帧头
def find_frameheader(buffer_list:list):
    # 循环查找，直至抛出异常
    while True:
        # 查找帧头的第一个标识符0x5a,若未找到，将会抛出ValueError异常
        try:
            header_ind = buffer_list.index(0x5a)
        except ValueError:
            raise HipnucFrame_NotCompleted_Exception

        if header_ind + 1 > len(buffer_list) - 1:
            raise HipnucFrame_NotCompleted_Exception

        if buffer_list[header_ind + 1] == 0xa5:
            # 找到帧头标识符0x5aa5，返回帧头位置
            return header_ind
        else:
            # 未找到帧头标识符，切片继续查找
            buffer_list = buffer_list[header_ind + 1:]

# 验证获取长度
def _get_frame_length(buffer_list, header_pos):
    return int(struct.unpack("<h", bytes(buffer_list[header_pos + 2:header_pos + 4]))[0])


# 验证长度是否合法
def _verify_frame_length(buffer_list:list, header_pos):
    # 获取到帧长度
    frame_len = int(struct.unpack("<h", bytes(buffer_list[header_pos + 2:header_pos + 4]))[0])
    # 判断帧长度是否合法
    if frame_len >= 1024:
        raise HipnucFrame_ErrorFrame_Exception
    elif frame_len + header_pos + 6 > len(buffer_list) :
        raise  HipnucFrame_NotCompleted_Exception

# 验证crc是否正确
def _verify_frame_crc(buffer_list, header_pos=0):
    # 获取到帧长度
    frame_len = int(struct.unpack("<h", bytes(buffer_list[header_pos + 2:header_pos + 2 + 2]))[0])
    # 获取帧内的crc
    f_crc = int(struct.unpack("<H", bytes(buffer_list[header_pos + 4:header_pos + 4 + 2]))[0])
    # 计算帧的crc
    cal_crc = crc16_update(buffer_list, 4, header_pos, 0)
    cal_crc = crc16_update(buffer_list, frame_len, header_pos + 6, cal_crc)

    if cal_crc != f_crc:
        raise HipnucFrame_ErrorFrame_Exception


# 截取一条完整且合法的帧，并将帧头帧尾返回
def intercept_one_complete_frame(buffer_list):
    # 找帧头
    header_pos = find_frameheader(buffer_list)
    frame_len = int(struct.unpack("<h", bytes(buffer_list[header_pos + 2:header_pos + 2 + 2]))[0])
    end_pos = header_pos + 5 + frame_len
    # 验证帧长度
    _verify_frame_length(buffer_list, header_pos)
    # 验证crc
    _verify_frame_crc(buffer_list, header_pos)

    return header_pos, end_pos

# 从完整帧中获取信息
def extraction_information_from_frame(frame_list:list, inf_fifo,report_datatype: dict = None):
    # 帧率统计
    global SamplesReceived
    SamplesReceived = SamplesReceived + 1
    # 处理数据帧
    data_dic = {}
    pos = 0

    data_frame_list = frame_list[6:]

    if data_frame_list[pos] == 0x61:
        HI221GW_property = data_packet_properties[0X61]["parse method"](data_frame_list[1:])
        node_num = HI221GW_property["CNT"]

        if report_datatype[data_packet_properties[0x61]["type"]] == True:
            data_dic.update(HI221GW_property)

        data_frame_list = data_frame_list[1 + 3:]
    else:
        #若0x61帧，则节点数默认为8
        node_num = 8
    #遍历解析数据段内包含的数据
    while len(data_frame_list) > 0:
        if data_frame_list[0] in data_packet_properties:

            temp_dic = data_packet_properties[data_frame_list[0]]["parse method"](data_frame_list[1:],node_num)

            if report_datatype[data_packet_properties[data_frame_list[0]]["type"]] == True:
                data_dic.update(temp_dic)
            else:
                pass

            id_len = data_packet_properties[data_frame_list[0]]["id_len"]
            data_len = data_packet_properties[data_frame_list[0]]["data_len"] * node_num
            data_frame_list = data_frame_list[id_len + data_len:]
        else:
            raise HipnucFrame_ErrorFrame_Exception

    inf_fifo.put(data_dic)