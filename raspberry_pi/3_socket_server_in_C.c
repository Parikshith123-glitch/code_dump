#include <stdio.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>

#define PORT 8080

int main()
{
	int fd, new_fd, count = 0;
	struct sockaddr_in address;
	socklen_t length = sizeof(address);
	char buffer[100];
	char *response = "Sending this from server to client";

	fd = socket(AF_INET, SOCK_STREAM, 0);

	address.sin_family=AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port=htons(PORT);

	bind(fd, (struct sockaddr*)&address, length);

	listen(fd, 3);

	new_fd = accept(fd, (struct sockaddr *)&address, &length);

while(1)
{
	int n =	read(new_fd, buffer, sizeof(buffer));
	if(n <= 0)
	{
		printf("client disconnected \n" );
		break;
	}
	printf("Received from Client: %s, count = %d\n", buffer, count++);

	send(new_fd, response, strlen(response),0);
	usleep(1000000);
}

close(new_fd);
close(fd);

	return 0;
}
