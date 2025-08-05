#include <stdio.h>
#include<stdlib.h>
#include <string.h>
#include <unistd.h>
#include<fcntl.h>

#define LED3_PATH "/sys/class/leds/beaglebone:green:usr3"


void writeLED(char filename [],char val []);

int main()
{
	printf("code started \n");
	writeLED("/brightness","1");
	while(1);
	return 0;
}

void writeLED(char filename [],char val [])
{
	int fd;
	int value = 1;
	char filep[100];
	sprintf(filep,LED3_PATH"%s",filename);
	printf("filepath  %s\n",filep);
	//fd = open("/sys/class/leds/beaglebone:green:usr3/brightness",O_RDWR);
	fd = open(filep,O_RDWR);
	if(fd < 0)
	{
		printf("file failed to open fd= %d",fd);
		exit(1);
	}	
	else
	{
		printf("file opened fd= %d",fd);
	}


	int ret =write(fd,val,1);
	if(ret > 0)
	{
		printf("write done\n ");
	}	
	else
	{
		printf("write  failed ret= %d",fd);
		_exit(1);
	}
}
