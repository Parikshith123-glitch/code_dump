// udp_client.c
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define PORT 8000
#define SERVER_IP "127.0.0.1"

int main() {
    int sock;
    struct sockaddr_in server_addr;
    char buffer[1024];

    // Create UDP socket
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Server info
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

    while(1) {
        printf("Enter string to send: ");
        scanf(" %[^\n]", buffer);  // Reads until newline, skips leading space

        sendto(sock, buffer, strlen(buffer), 0,
               (struct sockaddr*)&server_addr, sizeof(server_addr));

        if(strcmp(buffer, "quit") == 0) break;

        // Receive response
        socklen_t addr_len = sizeof(server_addr);
        int n = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
                         (struct sockaddr*)&server_addr, &addr_len);
        if(n < 0) {
            perror("recvfrom");
            continue;
        }

        buffer[n] = '\0';
        printf("Server reply: %s\n", buffer);
    }

    close(sock);
    return 0;
}



/*
--------------------------------------------

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define PORT 8000
#define SERVER_IP "127.0.0.1"

int main() {
    int sock;
    struct sockaddr_in server_addr;
    char buffer[1024];

    // 1. Create UDP socket
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    // 2. Server info
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

	while(1)
	{
		printf("Enter string to send: ");
		scanf("%[^\n]%*s", buffer);

		sendto(sock, buffer, strlen(buffer), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));

		if(strcmp(buffer, "quit") == 0) break;

// Prepare to receive response
        socklen_t addr_len = sizeof(server_addr);
        int n = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&server_addr, &addr_len);
        buffer[n] = '\0';  // Null-terminate string

		printf("Server reply: %s\n", buffer);
}

	close(sock);
	return 0;
}
*/
