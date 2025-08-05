// server_pi.c
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define PORT 8080

int main() {
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    char buffer[1024] = {0};
    char *response = "Hello from Raspberry Pi (C server)\n";

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;  // Listen on all interfaces
    server_addr.sin_port = htons(PORT);

    bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    listen(server_fd, 1);

    printf("C Server on Pi: Waiting for connection...\n");

    client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &addr_len);
    printf("Client connected.\n");

    while (1) {
        memset(buffer, 0, sizeof(buffer));
        int bytes = read(client_fd, buffer, sizeof(buffer));
        if (bytes <= 0) break;

        printf("Received from client: %s", buffer);
        send(client_fd, response, strlen(response), 0);
    }

    close(client_fd);
    close(server_fd);
    return 0;
}
