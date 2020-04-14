#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>
#include <pthread.h>

#include "packet.h"
#include "imu_data_decode.h"

/*
 * @brief Open serial port with the given device name
 *
 * @return The file descriptor on success or -1 on error.
 */

bool run_flag = true;
int rev_num;

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
	while(run_flag)
		rev_num = printf_num();

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
		fd = open_port("/dev/user_uart");

	
	imu_data_decode_init();
    
   
	uint8_t ID = 0;
	int16_t Acc[3] = {0};
	int16_t Gyo[3] = {0};
	int16_t Mag[3] = {0};
	float Eular[3] = {0};
	float Quat[4]  = {0};
	int32_t Pressure = 0;
	ssize_t n = 0;
	int i;
	uint8_t buf[1024];
	while(run_flag)
	{
		n = read(fd, buf, sizeof(buf));
		if(n > 0)
		{
			for(i=0; i<n; i++)
				Packet_Decode(buf[	i]);;

			get_id(&ID);
			printf("\033c");
			
			printf("    device id:  %-8d\n",ID);
			printf("   frame rate: %4dHz\n", rev_num);
			if(get_raw_acc(Acc))
				printf("	  Acc:	%-8d %-8d %-8d\r\n",Acc[0], Acc[1], Acc[2]);
			else
				printf("	  Acc:\n");

			if(get_raw_gyo(Gyo))
				printf("	  Gyo:	%-8d %-8d %-8d\r\n",Gyo[0], Gyo[1], Gyo[2]);
			else
				printf("	  Gyo:\n");

			if(get_raw_mag(Mag))
				printf("	  Mag:	%-8d %-8d %-8d\r\n",Mag[0], Mag[1], Mag[2]);
			else
				printf("	  Msg:\n");

			if(get_eular(Eular))
				printf(" Eular(P R Y):  %-8.2f %-8.2f %-8.2f\r\n",Eular[0], Eular[1], Eular[2]);
			else
				printf(" Eular(P R Y):  \n");

			if(get_quat(Quat))
				printf("quat(W X Y Z):  %-8.3f %-8.3f %-8.3f %-8.3f\r\n",Quat[0], Quat[1], Quat[2], Quat[3]);
			
			printf("Please enter ctrl + 'c' to quit...\n");
		}
	}
	sleep(1);
	pthread_join(ph_rate,NULL);

    close(fd);
}
