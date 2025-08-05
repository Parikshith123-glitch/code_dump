// udp_server.c
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>

#define PORT 8000

int main()
{
    int sock;
    struct sockaddr_in server_addr, client_addr;
    char buffer[100];

    // Creating UDP socket
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock < 0) {
        perror("socket");
        exit(1);
    }

    // Bind to port
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if(bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind");
        exit(1);
    }

    printf("Server listening on port %d\n", PORT);

    while(1) {
        socklen_t addr_len = sizeof(client_addr);

        int receive = recvfrom(sock, buffer, sizeof(buffer)-1, 0,
                               (struct sockaddr*)&client_addr, &addr_len);
        if(receive < 0) {
            perror("recvfrom");
            continue;
        }

        buffer[receive] = '\0';  // Null-terminate the received data

		char client_ip[INET_ADDRSTRLEN];
		inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
		printf("Received from %s:%d\n", client_ip, ntohs(client_addr.sin_port));



//        printf("Received from client: %s\n", buffer);

        if(strcmp(buffer, "quit") == 0) {
            printf("Quitting\n");
            break;
        }

        char *response = "Acknowledgement from server";
        sendto(sock, response, strlen(response), 0,
               (struct sockaddr*)&client_addr, addr_len);
    }

    close(sock);
    return 0;
}


/*

--------------------------------------
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>

#define PORT 8000

int main()
{
	int sock;
	struct sockaddr_in server_addr, client_addr;
	char buffer[100];

	// creating udp socket
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if(sock < 0)
	{
		perror("socket");
		exit(1);
	}

	// bind to port

	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(PORT);
	server_addr.sin_addr.s_addr = INADDR_ANY;

	bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
	printf("listening to port %d\n", PORT);

	while(1)
	{
		socklen_t addr_len = sizeof(client_addr);

		int receive = recvfrom(sock, buffer, sizeof(buffer)-1, 0, (struct sockaddr*)&client_addr, &addr_len);
		buffer[receive] = '\0';

		printf("Received from client: %s\n", buffer);

		if(strcmp(buffer, "quit") == 0)
		{
			printf("Quitting\n");
			break;
		}

		char *response = "Send this to client i.e. aknowledgement\n";
		sendto(sock, response, strlen(response), 0, (struct sockaddr* )&client_addr, sizeof(client_addr));
	}
	close(sock);
	return 0;
}
*/
