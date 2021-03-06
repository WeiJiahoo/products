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

def _parse_data_packet_0x90(data_section:list,node_num = None):
    module_id = {
        "id": data_section[0],
    }
    return module_id

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

def _parse_data_packet_0xD1(data_section:list,node_num = None):
    quaternion_list = []

    # for pos in range(node_num):
    pos = 0
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

def _parse_data_packet_0xA0(data_section:list,node_num = None):
    acc_list = []

    # for pos in range(node_num):
    pos = 0
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

def _parse_data_packet_0xA5(data_section:list,node_num = None):
    linacc_list = []

    # for pos in range(node_num):
    pos = 0
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
    linacc_list.append(temp_dic)

    linacc = {
        "linacc_list":linacc_list
    }

    return linacc

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

def _parse_data_packet_0xB0(data_section:list,node_num = None):
    gyr_list = []

    # for pos in range(node_num):
    pos = 0
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

def _parse_data_packet_0xC0(data_section:list,node_num = None):
    mag_list = []

    # for pos in range(node_num):
    pos = 0
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
    mag_list.append(temp_dic)

    mag = {
        "mag":mag_list
    }

    return mag

def _parse_data_packet_0xD0(data_section:list,node_num = None):
    eul_list = []

    # for pos in range(node_num):
    pos = 0
    t_pos = pos * 6
    Pitch = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
    Pitch = Pitch/100
    t_pos += 2
    Roll = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
    Roll = Roll/100
    t_pos += 2
    Yaw = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
    Yaw = Yaw/10

    temp_dic = {
        "Pitch":Pitch,
        "Roll":Roll,
        "Yaw":Yaw
    }
    eul_list.append(temp_dic)

    int_eul = {
        "int_eul":eul_list
    }

    return int_eul

def _parse_data_packet_0x72(data_section:list,node_num = None):
    eul_list = []

    for pos in range(node_num):
        t_pos = pos * 6
        Pitch = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
        Pitch = Pitch/100
        t_pos += 2
        Roll = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
        Roll = Roll/100
        t_pos += 2
        Yaw = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
        Yaw = Yaw/10

        temp_dic = {
            "Pitch":Pitch,
            "Roll":Roll,
            "Yaw":Yaw
        }
        eul_list.append(temp_dic)

    int_eul = {
        "int_eul":eul_list
    }

    return int_eul

def _parse_data_packet_0xD9(data_section:list,node_num = None):
    float_eul_list = []

    # for pos in range(node_num):
    pos = 0
    t_pos = pos * 12
    Pitch = float(struct.unpack("<f", bytes(data_section[t_pos:t_pos + 4]))[0])
    t_pos += 4
    Roll = float(struct.unpack("<f", bytes(data_section[t_pos:t_pos + 4]))[0])
    t_pos += 4
    Yaw = float(struct.unpack("<f", bytes(data_section[t_pos:t_pos + 4]))[0])

    temp_dic = {
        "Pitch":Pitch,
        "Roll":Roll,
        "Yaw":Yaw
    }

    float_eul_list.append(temp_dic)

    float_eul = {
        "float_eul":float_eul_list
    }

    return float_eul

def _parse_data_packet_0x60(data_section:list,node_num = None):
    test_8f_list = []
    for pos in range(8):
        t_pos = pos * 4
        test_f = float(struct.unpack("<f", bytes(data_section[t_pos:t_pos + 4]))[0])

        test_8f_list.append(test_f)

    test_8f = {
        "test_8f": test_8f_list,
    }

    return test_8f

def _parse_data_packet_0xF0(data_section:list,node_num = None):
    prs_list = []
    # for pos in range(node_num):
    t_pos = 0
    prs_f = float(struct.unpack("<f", bytes(data_section[t_pos:t_pos + 4]))[0])

    prs_list.append(prs_f)

    prs = {
        "prs": prs_list,
    }

    return prs

def _parse_data_packet_0x91(data_section:list,node_num = None):
    pos = 0
    id_temp_list = []
    timestamp_temp_list = []
    acc_temp_list = []
    gyr_temp_list = []
    mag_temp_list = []
    eul_temp_list = []
    quat_temp_list = []
    # id
    id = data_section[pos]
    id_temp_list.append(id)
    pos += 1
    #reserved
    pos += 6
    #timestamp
    timestamp = int(struct.unpack("<I", bytes(data_section[pos:pos + 4]))[0])
    timestamp_temp_list.append(timestamp)
    pos += 4
    #acc
    acc_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    acc_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    acc_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    acc_dic = {
        "X":acc_X,
        "Y":acc_Y,
        "Z":acc_Z
    }
    acc_temp_list.append(acc_dic)
    #gyr
    gyr_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    gyr_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    gyr_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    gyr_dic = {
        "X": gyr_X,
        "Y": gyr_Y,
        "Z": gyr_Z
    }
    gyr_temp_list.append(gyr_dic)
    #mag
    mag_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    mag_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    mag_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    mag_dic = {
        "X": mag_X,
        "Y": mag_Y,
        "Z": mag_Z
    }
    mag_temp_list.append(mag_dic)
    #eul
    eul_Roll = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    eul_Pitch = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    eul_Yaw = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    eul_dic = {
        "Roll": eul_Roll,
        "Pitch": eul_Pitch,
        "Yaw": eul_Yaw
    }
    eul_temp_list.append(eul_dic)
    #quat
    quat_W = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    quat_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    quat_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    quat_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    quat_dic = {
        "W": quat_W,
        "X": quat_X,
        "Y": quat_Y,
        "Z": quat_Z
    }
    quat_temp_list.append(quat_dic)


    temp_dic = {
        "id":id_temp_list,
        "timestamp":timestamp_temp_list,
        "acc":acc_temp_list,
        "gyr":gyr_temp_list,
        "mag":mag_temp_list,
        "eul":eul_temp_list,
        "quat":quat_temp_list
    }
    return temp_dic

rel_node_num = 0
module_node_num = 0
def _parse_data_packet_0x62(data_section:list,node_num = None):
    global rel_node_num
    global module_node_num
    global data_packet_properties
    id_temp_list = []
    timestamp_temp_list = []
    acc_temp_list = []
    gyr_temp_list = []
    mag_temp_list = []
    eul_temp_list = []
    quat_temp_list = []
    pos = 0
    gwid = data_section[0]
    cnt = data_section[1]
    rel_node_num = cnt
    module_node_num = cnt
    data_packet_properties[0x62]["data_len"] = 5 + (76 * cnt)
    pos += 2
    #reserved
    pos += 5
    #0x91 packet
    for node in range(cnt):
        #packet id
        pos += 1
        # id
        id = data_section[pos]
        id_temp_list.append(id)
        pos += 1
        #reserved
        pos += 6
        #timestamp
        timestamp = int(struct.unpack("<I", bytes(data_section[pos:pos + 4]))[0])
        timestamp_temp_list.append(timestamp)
        pos += 4
        #acc
        acc_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        acc_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        acc_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        acc_dic = {
            "X":acc_X,
            "Y":acc_Y,
            "Z":acc_Z
        }
        acc_temp_list.append(acc_dic)
        #gyr
        gyr_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        gyr_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        gyr_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        gyr_dic = {
            "X": gyr_X,
            "Y": gyr_Y,
            "Z": gyr_Z
        }
        gyr_temp_list.append(gyr_dic)
        #mag
        mag_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        mag_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        mag_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        mag_dic = {
            "X": mag_X,
            "Y": mag_Y,
            "Z": mag_Z
        }
        mag_temp_list.append(mag_dic)
        #eul
        eul_Roll = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        eul_Pitch = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        eul_Yaw = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        eul_dic = {
            "Roll": eul_Roll,
            "Pitch": eul_Pitch,
            "Yaw": eul_Yaw
        }
        eul_temp_list.append(eul_dic)
        #quat
        quat_W = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        quat_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        quat_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        quat_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        quat_dic = {
            "W": quat_W,
            "X": quat_X,
            "Y": quat_Y,
            "Z": quat_Z
        }
        quat_temp_list.append(quat_dic)

    temp_dic = {
        "GWD":gwid,
        "CNT":cnt,
        "id":id_temp_list,
        "timestamp":timestamp_temp_list,
        "acc":acc_temp_list,
        "gyr":gyr_temp_list,
        "mag":mag_temp_list,
        "eul":eul_temp_list,
        "quat":quat_temp_list
    }
    return temp_dic

data_packet_properties = {
    0x90: {
        "type": "id",
        "id_len": 1,
        "data_len": 1,
        "parse method": _parse_data_packet_0x90,
        "gw_data":False
    },
    0x61:{
        "type":"Expanding Information",
        "id_len":1,
        "data_len":3,
        "parse method":_parse_data_packet_0x61,
        "gw_data":True
    },
    # gw_quat
    0x71:{
        "type":"quat",
        "id_len":1,
        "data_len":16,
        "parse method":_parse_data_packet_0x71,
        "gw_data":True
    },
    # quat
    0xD1: {
        "type": "quat",
        "id_len": 1,
        "data_len": 16,
        "parse method": _parse_data_packet_0xD1,
        "gw_data":False
    },
    # gw_acc
    0x75:{
        "type":"acc",
        "id_len": 1,
        "data_len": 6,
        "parse method": _parse_data_packet_0x75,
        "gw_data": True
    },
    # acc
    0xA0: {
        "type": "acc",
        "id_len": 1,
        "data_len": 6,
        "parse method": _parse_data_packet_0xA0,
        "gw_data": False
    },
    # linacc
    0xA5: {
        "type": "linacc",
        "id_len": 1,
        "data_len": 6,
        "parse method": _parse_data_packet_0xA5,
        "gw_data": False
    },
    #gw_gyr
    0x78:{
        "type":"gyr",
        "id_len": 1,
        "data_len": 6,
        "parse method": _parse_data_packet_0x78,
        "gw_data": True
    },
    # gyr
    0xB0: {
        "type": "gyr",
        "id_len": 1,
        "data_len": 6,
        "parse method": _parse_data_packet_0xB0,
        "gw_data": False
    },
    # mag
    0xC0: {
        "type": "mag",
        "id_len": 1,
        "data_len": 6,
        "parse method": _parse_data_packet_0xC0,
        "gw_data": False
    },
    # int_eul
    0xD0: {
        "type": "int_eul",
        "id_len": 1,
        "data_len": 6,
        "parse method": _parse_data_packet_0xD0,
        "gw_data": False
    },
    # gw_int_eul
    0x72: {
        "type": "int_eul",
        "id_len": 1,
        "data_len": 6,
        "parse method": _parse_data_packet_0x72,
        "gw_data": True
    },
    # float_eul
    0xD9: {
        "type": "float_eul",
        "id_len": 1,
        "data_len": 12,
        "parse method": _parse_data_packet_0xD9,
        "gw_data": False
    },
    0x60: {
        "type": "test_8f",
        "id_len": 1,
        "data_len": 32,
        "parse method": _parse_data_packet_0x60,
        "gw_data": False
    },
    0xF0: {
        "type": "prs",
        "id_len": 1,
        "data_len": 4,
        "parse method": _parse_data_packet_0xF0,
        "gw_data": False
    },
    0x91: {
        "type": "imusol",
        "id_len": 1,
        "data_len": 76,
        "parse method": _parse_data_packet_0x91,
        "gw_data": False
    },
    0x62: {
        "type": "gwsol",
        "id_len": 1,
        "data_len": 76 * 1,
        "parse method": _parse_data_packet_0x62,
        "gw_data": True
    },
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
    try:
        frame_len = int(struct.unpack("<H", bytes(buffer_list[header_pos + 2:header_pos + 2 + 2]))[0])
    except struct.error:
        raise HipnucFrame_NotCompleted_Exception
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
    global rel_node_num
    global module_node_num
    SamplesReceived = SamplesReceived + 1
    # 处理数据帧
    data_dic = {}
    pos = 0

    data_frame_list = frame_list[6:]

    if data_frame_list[pos] == 0x61:
        HI221GW_property = data_packet_properties[0X61]["parse method"](data_frame_list[1:])
        module_node_num = HI221GW_property["CNT"]

        try:
            if report_datatype[data_packet_properties[0x61]["type"]] == True:
                data_dic.update(HI221GW_property)
        #report type default值为False,即未在config文件中配置的默认为False
        except KeyError:
            pass

        data_frame_list = data_frame_list[1 + 3:]
    else:
        pass
        # #若无0x61帧，则节点数默认为1
        # module_node_num = 1
    #遍历解析数据段内包含的数据
    while len(data_frame_list) > 0:
        if data_frame_list[0] in data_packet_properties:

            temp_dic = data_packet_properties[data_frame_list[0]]["parse method"](data_frame_list[1:],module_node_num)

            try:
                if report_datatype[data_packet_properties[data_frame_list[0]]["type"]] == True:
                    data_dic.update(temp_dic)
                else:
                    pass

            except KeyError:
                pass

            if data_packet_properties[data_frame_list[0]]["gw_data"] == True:
                rel_node_num = module_node_num
            else:
                rel_node_num = 1

            id_len = data_packet_properties[data_frame_list[0]]["id_len"]

            if data_frame_list[0] == 0x62:
                data_len = 76 * rel_node_num + 8
            else:
                data_len = data_packet_properties[data_frame_list[0]]["data_len"] * rel_node_num

            data_frame_list = data_frame_list[id_len + data_len:]
        else:
            # raise HipnucFrame_ErrorFrame_Exception
            data_frame_list = data_frame_list[1:]

    inf_fifo.put(data_dic)
