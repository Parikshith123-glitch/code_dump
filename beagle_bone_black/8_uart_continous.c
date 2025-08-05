#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

int main()
{
    int uart = open("/dev/ttyS4", O_RDWR);

    if(uart < 0)
    {
        perror("error opening UART");
        return 1;
    }

struct termios options;
tcgetattr(uart, &options);          // X = 4 or 5
cfsetispeed(&options, B9600);        // or B115200, etc.
cfsetospeed(&options, B9600);

options.c_cflag &= ~PARENB;
options.c_cflag &= ~CSTOPB;
options.c_cflag &= ~CSIZE;
options.c_cflag |= CS8;
options.c_cflag |= CREAD | CLOCAL;
options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
options.c_iflag &= ~(IXON | IXOFF | IXANY);
options.c_oflag &= ~OPOST;

tcsetattr(uart, TCSANOW, &options);


	while(1)
	{
		char ch;
		if(read(STDIN_FILENO, &ch, 1) > 0)
		{
			write(uart, &ch, 1);

			char rx;
			usleep(90000);
			if(read(uart, &rx, 1) > 0)
			{
				write(STDOUT_FILENO, &rx, 1);
			}
		}
	}

	close(uart);
    return 0;
}
