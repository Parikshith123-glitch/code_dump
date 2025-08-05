#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>

#define PORT 8080

int main()
{
    int fd = socket(AF_INET, SOCK_STREAM, 0), count = 0;
    struct sockaddr_in address;
    socklen_t length = sizeof(address);
    char *string = "From Client to Server";
    char buffer[100];

    address.sin_family=AF_INET;
    address.sin_port=htons(PORT);

    inet_pton(AF_INET, "10.1.30.171", &address.sin_addr);

    connect(fd, (struct sockaddr *)&address,length);

while(1)
{
	memset(buffer, 0, sizeof(buffer));

	send(fd, string, strlen(string), 0);
	read(fd, buffer, sizeof(buffer));

    	printf("Received from server: %s, count = %d\n", buffer, count++);
    	usleep(1000000);
}
close(fd);
    return 0;

}
