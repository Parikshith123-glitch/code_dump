#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

void setup_uart(int fd);

int main()
{
    int uart4 = open("/dev/ttyS4", O_RDWR);
    int uart2 = open("/dev/ttyS2", O_RDWR);

    if(uart4 < 0 || uart2 < 0)
    {
        perror("error opening the UART");
        return 1;
    }

setup_uart(uart4);
setup_uart(uart2);

	//char string[50] = "testing for sending the raw data \n";
int raw_value = 1;
while(1){

	write(uart4, &raw_value, 4);

	usleep(100000);

	static int read_raw_value;
	read(uart2, &read_raw_value, 4);

	usleep(100000);

	printf("This is from uart2 after receving the data %d\n", read_raw_value);
	read_raw_value += 1;
}

        close(uart4);
        close(uart2);
    return 0;
}


void setup_uart(int fd) {
    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag = CS8 | CREAD | CLOCAL;
    options.c_lflag = 0;
    options.c_iflag = 0;
    options.c_oflag = 0;

    tcsetattr(fd, TCSANOW, &options);
}
