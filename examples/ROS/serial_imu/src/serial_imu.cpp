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
	//初始化ROS。它允许ROS通过命令行进行名称重映射——目前，这不是重点。同样，在这里指定该节点的名称——必须唯一。这里的名称必须是一个base name，不能包含"/"
    ros::init(argc, argv, "serial_imu");
    //创建句柄,这个句柄用于管理该进程使用的资源。
    ros::NodeHandle n;

	//告诉节点管理器master, 将要在IMU_data topic上发布一个sensor_msgs::Imu的消息。
	//这样master就会告诉所有订阅了IMU_data topic的节点，将要有数据发布。第二个参数是发布序列的大小。
	//在这样的情况下，如果发布的消息太快，缓冲区中的消息在大于20个的时候就会开始丢弃先前发布的消息。
	//NodeHandle::advertise() 返回一个 ros::Publisher对象,它有两个作用: 
	//1) 它有一个publish()成员函数可以让你在topic上发布消息； 2) 如果消息类型不对,它会拒绝发布。
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
	//ros::Rate对象可以允许你指定自循环的频率。它会追踪记录自上一次调用Rate::sleep()后时间的流逝，并休眠直到一个频率周期的时间。在这里，让它以500hz的频率运行。
    ros::Rate loop_rate(500);

    while(ros::ok())
    {
		//roscpp会默认安装一个SIGINT句柄，它负责处理Ctrl + C键盘操作 --> 使得ros::ok()返回FALSE。
		//如果下列条件之一发生，ros::ok()返回false：SIGINT接收到(Ctrl-C);被另一同名节点踢出ROS网络;ros::shutdown()被程序的另一部分调用;所有的ros::NodeHandles都已经被销毁.一旦ros::ok()返回false, 所有的ROS调用都会失效
        //获取缓冲区内的字节数r
		size_t num = sp.available();
        if(num!=0)
        {
            uint8_t buffer[1024];
            //读出数据
            num = sp.read(buffer, num);
            if(num > 0)
			{
				for(int i = 0; i < num; i++)
					packet_decode(buffer[i]);

				sensor_msgs::Imu imu_data;
				imu_data.header.stamp = ros::Time::now();
				imu_data.header.frame_id = "base_link";

//				puts("\033c");
 				if(receive_gwsol.tag != KItemGWSOL)
				{
//					dump_data_packet(&receive_imusol);
					imu_data.orientation.x = receive_imusol.quat[1];
					imu_data.orientation.y = receive_imusol.quat[2];
					imu_data.orientation.z = receive_imusol.quat[3];
					imu_data.orientation.w = receive_imusol.quat[0];
					imu_data.angular_velocity.x = receive_imusol.gyr[0];
					imu_data.angular_velocity.y = receive_imusol.gyr[1];
					imu_data.angular_velocity.z = receive_imusol.gyr[2];
					imu_data.linear_acceleration.x = receive_imusol.acc[0];
					imu_data.linear_acceleration.y = receive_imusol.acc[1];
					imu_data.linear_acceleration.z = receive_imusol.acc[2];
					IMU_pub.publish(imu_data);
//					puts("Pleaes enter ctrl + 'c' to quit....");
				}
				else
				{
					printf("       GW ID: %4d\n", receive_gwsol.gw_id);
					for(int i = 0; i < receive_gwsol.n; i++)
					{
//						dump_data_packet(&receive_gwsol.receive_imusol[i]);
						imu_data.orientation.x = receive_gwsol.receive_imusol[i].quat[2];
						imu_data.orientation.y = receive_gwsol.receive_imusol[i].quat[1];
						imu_data.orientation.z = receive_gwsol.receive_imusol[i].quat[0];
						imu_data.orientation.w = receive_gwsol.receive_imusol[i].quat[3];
						IMU_pub.publish(imu_data);
//						puts("");
					}
//					puts("Please enter ctrl + 'c' to quit...");
				}
			}
        }
		//这条语句是调用ros::Rate对象来休眠一段时间以使得发布频率为500hz。
        loop_rate.sleep();
    }
    
	sp.close();
 
	return 0;
}

void dump_data_packet(receive_imusol_packet_t *data)
{
	if(bitmap & BIT_VALID_ID)
		printf("     Devie ID:%6d\n",data->id);

	if(bitmap & BIT_VALID_TIMES)
		printf("    Run times: %d days  %d:%d:%d:%d\n",data->times / 86400000, data->times / 3600000 % 24, data->times / 60000 % 60, data->times / 1000 % 60, data->times % 1000);

	printf("  Frame Rate:  %4dHz\r\n",frame_rate);
	if(bitmap & BIT_VALID_ACC)
		printf("       Acc(G):%8.3f %8.3f %8.3f\r\n", data->acc[0], data->acc[1], data->acc[2]);

	if(bitmap & BIT_VALID_GYR)
		printf("   Gyr(deg/s):%8.2f %8.2f %8.2f\r\n", data->gyr[0], data->gyr[1], data->gyr[2]);

	if(bitmap & BIT_VALID_MAG)
		printf("      Mag(uT):%8.2f %8.2f %8.2f\r\n", data->mag[0], data->mag[1], data->mag[2]);

	if(bitmap & BIT_VALID_EUL)
		printf("   Eul(R P Y):%8.2f %8.2f %8.2f\r\n", data->eul[0], data->eul[1], data->eul[2]);

	if(bitmap & BIT_VALID_QUAT)
		printf("Quat(W X Y Z):%8.3f %8.3f %8.3f %8.3f\r\n", data->quat[0], data->quat[1], data->quat[2], data->quat[3]);
}
