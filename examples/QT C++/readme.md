# QT C++ 接收程式範例

測試版本:
- QT 5.9.9 + Win10
- Hi221 韌體版本 1.0.4，新協議尚未支援

使用教學:

1. include 資料夾底下分別有 kptl/imu_data/imu_data_decode header 與 cpp 檔，
    在QT加入此6個檔案。

2. QT .pro檔加入 "QT += serialport" 使用序列埠 

3. 在主視窗當中 include header
	```C++
	#ifndef MAINWINDOW_H
	#define MAINWINDOW_H
    
   #include "include/imu_data_decode.h"
   #include "include/kptl.h"
   ```
   
4.  "imu_data_decode_init()" -> 初始化一次 RFreceiver 和 node
5.  "kptl_decode(c)" -> 接收到來自序列的單個 uint8_t 字元並解析
6. 宣告 "imu_data_t imu"，呼叫 "dump_rf_data(&imu)"(無線接收器)或 "dump_imu_data(&imu)"(USB節點)；將數據存入"imu"。
    想獲得 pitch/roll/yaw 的話依序為 imu.eul[0]，imu.eul[1]，imu.eul[2]。
    imu_data_t 結構型態參考以下:

    ```C++
    typedef struct
    {
        uint8_t id;
        float       q[4];
        int16_t     acc[3];
        int16_t     gyr[3];
        int16_t     mag[3];
        float       eul[3];
        float       temperature;
        float       rf_quat[MAX_SLV_CNT][4];
        int16_t     rf_acc[MAX_SLV_CNT][3];
        int16_t     rf_gyr[MAX_SLV_CNT][3];
        int16_t     rf_mag[MAX_SLV_CNT][3];
        int16_t     rf_eul[MAX_SLV_CNT][3];
    }imu_data_t;
    ```

7.  若使用無線接收器, 必須設定在 imu_data_decode.cpp 設定 "rf_slave_cnt" 的值為節點數量，節點 ID 必須依照順序從 0 開始設定，否則會閃退。