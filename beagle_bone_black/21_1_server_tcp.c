#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>         // for close()
#include <arpa/inet.h>      // for inet_addr(), sockaddr_in

#define PORT 1234

int main() {
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    char buffer[1024] = {0};
    int addr_len = sizeof(client_addr);

    // 1. Create socket
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == 0) {
        perror("Socket failed");
        exit(EXIT_FAILURE);
    }

    // 2. Bind address and port
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;  // 0.0.0.0
    server_addr.sin_port = htons(PORT);

    bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));

    // 3. Listen for incoming connections
    listen(server_fd, 3);
    printf("Server listening on port %d...\n", PORT);

    // 4. Accept client connection
    client_fd = accept(server_fd, (struct sockaddr *)&client_addr, (socklen_t*)&addr_len);

	while(1)
{
	memset(buffer, 0, sizeof(buffer));  // Clear buffer for fresh read
    read(client_fd, buffer, 1024);

	int compare = strcmp(buffer, "quit");
	if(compare != 0)
	{
    	printf("Received from client: %s\n", buffer);
	} else {
		// 5. Send ending response

		char *msg = "quit received: Closing client";
		printf("quit has been received from client, closing server and client\n");

    	send(client_fd, msg, strlen(msg), 0);

    	close(client_fd);
    	close(server_fd);
		break;
	}
}
    return 0;
}

