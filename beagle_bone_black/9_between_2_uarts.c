#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

void setup_uart(int fd);
char buffer[50];

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

    while(1)
    {
        char ch;
        if(read(STDIN_FILENO, &ch, 1) > 0)      // This STDIN macro is for reading from the keyboard.
        {
            write(uart4, &ch, 1); 				// input from uart4
            char rx;
//          usleep(90000);

        	if (read(uart2, &rx, 1) > 0) {  	// Read response from UART2
   	    	snprintf(buffer, sizeof(buffer), "from UART4 to UART2 received: %c\n", rx);
       		write(STDOUT_FILENO, buffer, strlen(buffer));  // Print to terminal
       		}
        }
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

