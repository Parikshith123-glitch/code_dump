//----------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>

void setup_uart(int fd);

int main()
{
    int fd = open("/dev/ttyS4", O_RDWR | O_NOCTTY);
    if(fd < 0 )
    {
        perror("Error opening the file");
        return 1;
    }

    setup_uart(fd);

    char value = 2;

    while(1)
    {
        int ret_value = write(fd, &value, sizeof(value));
        if(ret_value > 0 )
        {
            printf("Sent 0x%X (%d), bytes: %d\n", value, value, ret_value);
        }
        else
        {
            perror("Error sending data");
        }
        usleep(50000);  // 50 ms delay
    }

    close(fd);
    return 0;
}

void setup_uart(int fd) {
    struct termios options;

    if(tcgetattr(fd, &options) < 0) {
        perror("tcgetattr failed");
        return;
    }

    // Set baud rate
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // Set 8N1 (8 data bits, no parity, 1 stop bit)
    options.c_cflag &= ~PARENB;        // No parity
    options.c_cflag &= ~CSTOPB;        // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;            // 8 bits
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem ctrl lines

    // Disable software flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL);  // Don't map CR or NL

    // Raw output
    options.c_oflag &= ~OPOST;

    // Raw input
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Flush before applying
    tcflush(fd, TCIOFLUSH);

    if(tcsetattr(fd, TCSANOW, &options) < 0) {
        perror("tcsetattr failed");
        return;
    }
}

