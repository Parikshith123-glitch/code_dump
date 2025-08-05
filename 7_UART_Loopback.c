#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

int main()
{
    int uart = open("/dev/ttyS4", O_RDWR);

    if(uart < 0)
    {
        perror("error opening UART");
		return 1;
    }

// write
	int i = 0;

while(1)
	{
	char text[] = " -Text- ";
	write(uart, text, strlen(text));
    usleep(10000);

//read
	char buffer[50] = {0};
	int n = read(uart, buffer, sizeof(buffer)-1);

	if(n > 0)
	{
		printf("Received string with count: %d %s", i, buffer);
		} else {
		printf("Nothing was received");
	}

//	usleep(500000);
	i++;
}
	close(uart);
    return 0;
}
