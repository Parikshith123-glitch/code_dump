#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>         // for close()
#include <arpa/inet.h>      // for inet_addr(), sockaddr_in

#define PORT 1234

int main() {
    int sock;
    struct sockaddr_in server_addr;
    char buffer[1024] = {0};

    // 1. Create socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation error");
        return -1;
    }

    // 2. Setup server address struct
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    // 3. Connect to server
    connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));

    // 4. Send message to server
    char msg[100];

while(1)
	{
		memset(msg, 0, sizeof(msg));
		printf("Enter the message to send to the server: ");
		scanf ("%[^\n]%*c", msg);

		int counter = strcmp(msg, "quit");

		if(counter != 0)
		{
			send(sock, msg, strlen(msg), 0);
		} else {

			send(sock, msg, strlen(msg), 0);
			read(sock, buffer, 1024);
			printf("%s\n", buffer);

			close(sock);
			break;
		}
	}
    return 0;
}

