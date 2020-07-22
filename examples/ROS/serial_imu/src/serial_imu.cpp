//serial_imu.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sensor_msgs/Imu.h>
#include <signal.h>

#ifdef __cplusplus 
extern "C"{
#endif
#include <stdint.h>
#include <stdbool.h>
#include "packet.h"
#include "imu_data_decode.h"
int imu_data_decode_init(void);
typedef void (*on_data_received_event)(packet_t *ptr);
void packet_decode_init(packet_t *pkt, on_data_received_event rx_handler);
uint32_t packet_decode(uint8_t);

#ifdef __cplusplus
}
#endif

void dump_data_packet(receive_imusol_packet_t *data);

static int frame_rate;

static uint8_t buf[2048];

void timer(int sig)
{
	if(SIGALRM == sig)
	{
		frame_rate = frame_count;
		frame_count = 0;
		alarm(1);
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_imu");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;

	ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("IMU_data", 20);

    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);
	
	imu_data_decode_init();
 	signal(SIGALRM,timer);

    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
	
	alarm(1);
    ros::Rate loop_rate(500);

    while(ros::ok())
    {
        //获取缓冲区内的字节数r
		size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);
            if(n > 0)
			{
				for(int i = 0; i < n; i++)
					packet_decode(buffer[i]);

				sensor_msgs::Imu imu_data;
				imu_data.header.stamp = ros::Time::now();
				imu_data.header.frame_id = "base_link";

				puts("\033c");
 				if(receive_gwsol.tag != KItemGWSOL)
				{
					dump_data_packet(&receive_imusol);
					imu_data.orientation.x = receive_imusol.quat[2];
					imu_data.orientation.y = -receive_imusol.quat[1];
					imu_data.orientation.z = -receive_imusol.quat[0];
					imu_data.orientation.w = receive_imusol.quat[3];
					IMU_pub.publish(imu_data);
					puts("Pleaes enter ctrl + 'c' to quit....");
				}
				else
				{
					printf("       GW ID: %4d\n",receive_gwsol.gw_id);
					for(int i = 0; i < receive_gwsol.n; i++)
					{
						dump_data_packet(&receive_gwsol.receive_imusol[i]);
						puts("");
					}
					puts("Please enter ctrl + 'c' to quit...");
				}
			}
        }
        loop_rate.sleep();
    }
    
	//关闭串口
	sp.close();
 
	return 0;
}

void dump_data_packet(receive_imusol_packet_t *data)
{
	if(bitmap & BIT_VALID_ID)
	{
		printf("     Devie ID:%6d\n",data->id);
	}
	if(bitmap & BIT_VALID_TIMES)
	{
		printf("    Run times:%d days %d:%d:%d:%d\n",data->times / 86400000, data->times / 3600000 % 24, data->times / 60000 % 60, data->times / 1000 % 60, data->times % 1000);
	}
	printf("  Frame Rate:  %4dHz\r\n",frame_rate);
	if(bitmap & BIT_VALID_ACC)
	{
		printf("       Acc(G):%8.3f %8.3f %8.3f\r\n", data->acc[0], data->acc[1], data->acc[2]);
	}
	if(bitmap & BIT_VALID_GYR)
	{
		printf("   Gyr(deg/s):%8.2f %8.2f %8.2f\r\n", data->gyr[0], data->gyr[1], data->gyr[2]);
	}
	if(bitmap & BIT_VALID_MAG)
	{
		printf("      Mag(uT):%8.2f %8.2f %8.2f\r\n", data->mag[0], data->mag[1], data->mag[2]);
	}
	if(bitmap & BIT_VALID_EUL)
	{
		printf("   Eul(R P Y):%8.2f %8.2f %8.2f\r\n", data->eul[0], data->eul[1], data->eul[2]);
	}
	if(bitmap & BIT_VALID_QUAT)
	{
		printf("Quat(W X Y Z):%8.3f %8.3f %8.3f %8.3f\r\n", data->quat[0], data->quat[1], data->quat[2], data->quat[3]);

	}
}
