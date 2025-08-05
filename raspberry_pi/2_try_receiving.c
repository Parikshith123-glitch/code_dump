#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

int main() {
	int uart = open("/dev/ttyAMA5", O_RDWR | O_NOCTTY);
	//int uart = open("/dev/ttyAMA3", O_RDWR | O_NOCTTY);
	 if (uart < 0) {
        perror("Failed to open UART");
        return 1;
    }

    // UART configuration
    struct termios options;
    tcgetattr(uart, &options);

    // Set Baud Rate
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag &= ~PARENB;  // No parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CSIZE;   // Clear size bits
    options.c_cflag |= CS8;      // 8 data bits

    options.c_cflag |= CREAD | CLOCAL;  // Enable receiver, ignore modem control lines
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable flow control
    options.c_oflag &= ~OPOST; // Raw output

    tcsetattr(uart, TCSANOW, &options);  // Apply settings

    char buffer[1000];
	int count = 0;
	int idx = 0;
	char ch;

	while(1)
	{
		if(read(uart, &ch, 1)>0)
		{	if(ch == '\n')
			{
				buffer[idx] = '\0';
				printf("Received String: %s %d\n", buffer, count++);
				idx = 0;
			} else {
				buffer[idx++] = ch;
				if (idx >= sizeof(buffer) - 1) idx = 0;  // Safety reset
			}
		}

	}
    close(uart);
    return 0;
}
