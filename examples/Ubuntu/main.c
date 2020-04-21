#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>

#include "packet.h"
#include "imu_data_decode.h"

/*
 * @brief Open serial port with the given device name
 *
 * @return The file descriptor on success or -1 on error.
 */

/* recv freq */
static int frame_rate;
static uint8_t ID = 0;
static float acc[3] = {0};
static float gyr[3] = {0};
static float mag[3] = {0};
static float eul[3] = {0};
static float quat[4]  = {0};
static uint8_t buf[2048];
	
int time_out(int second)
{
	struct timeval time_value;
	time_value.tv_sec = second;
	time_value.tv_usec = 0;

	return select(0,NULL,NULL,NULL,&time_value);
}

int ret_frame_count(void)
{
	frame_count = 0;
	int i = time_out(1);

	return frame_count;
}

int open_port(char *port_device)
{
   struct termios options;

	int fd = open(port_device, O_RDWR | O_NOCTTY);
	
	tcgetattr(fd, &options);

	if (fd == -1)
    {
        perror("open_port: Unable to open SerialPort");
		puts("Please enter usb port append to the execution command!!!");
		puts("Please enter ctrl + 'c' to quit...\n");
		return(-1);
    }

    if(fcntl(fd, F_SETFL, 0)<0)
	{
		printf("fcntl failed\n");
	}
    else
	{
		fcntl(fd, F_SETFL, 0);
	}
  
	if(isatty(STDIN_FILENO)==0)
	{
		printf("standard input is not a terminal device\n");
	}
	else 
	{
		printf("isatty success!\n");
		printf("1");
	
	}


	bzero(&options,sizeof(options));
	options.c_cflag = B115200 | CS8 | CLOCAL |CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
	return (fd);
}

void *pthread_frame_rate(void *arg)
{
	while(1)
		frame_rate = ret_frame_count();

	pthread_exit(NULL);
}


int main(int argc,const char *argv[])
{
	pthread_t ph_rate;
	int ret_pth1 = pthread_create(&ph_rate, NULL, pthread_frame_rate, NULL);
	int fd = 0;
    char dir_usb_dev[64] = "/dev/";

	if(argc >1)
	{
		strcat(dir_usb_dev,argv[1]);
	  	fd = open_port(dir_usb_dev);
	}
	else
	{
		puts("Please enter USB port append to the execution command!!!");
		return 0;
	}

	imu_data_decode_init();
    

	
	ssize_t n = 0;
	int i;

	while(1)
	{
		n = read(fd, buf, sizeof(buf));

		if(n > 0)
		{
			for(i=0; i<n; i++)
			{
				Packet_Decode(buf[i]);
			}
			
			get_id(&ID);
			get_raw_acc(acc);
			get_raw_gyr(gyr);
			get_raw_mag(mag);
			get_eul(eul);
			get_quat(quat);
		
			printf("\033c");

			printf("    Device ID:  %-8d\n",ID);
			printf("   Frame Rate: %4dHz\n", frame_rate);
			if(acc_tag_flag)
				printf("       Acc(G):	%8.3f %8.3f %8.3f\r\n", acc[0], acc[1], acc[2]);
			if(gyr_tag_flag)
				printf("   gyr(deg/s):	%8.2f %8.2f %8.2f\r\n", gyr[0], gyr[1], gyr[2]);
			if(mag_tag_flag)
				printf("      mag(uT):	%8.2f %8.2f %8.2f\r\n", mag[0], mag[1], mag[2]);
			if(eul_tag_flag)
				printf("   eul(R P Y):  %8.2f %8.2f %8.2f\r\n", eul[0], eul[1], eul[2]);
			if(quat_tag_flag)
				printf("quat(W X Y Z):  %8.3f %8.3f %8.3f %8.3f\r\n", quat[0], quat[1], quat[2], quat[3]);

			printf("Please enter ctrl + 'c' to quit...\n");

		}
	}	
	sleep(1);
	pthread_join(ph_rate, NULL);

    close(fd);
}
