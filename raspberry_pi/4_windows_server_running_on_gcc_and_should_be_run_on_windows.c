// server_win.c
#include <stdio.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#define PORT 8080

int main() {
    WSADATA wsa;
    SOCKET fd, new_fd;
    struct sockaddr_in address;
    int length = sizeof(address);
    char buffer[1024];
    char *response = "Reply from server to clients\n";

    // Initialize Winsock
    WSAStartup(MAKEWORD(2,2), &wsa);

    fd = socket(AF_INET, SOCK_STREAM, 0);
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    bind(fd, (struct sockaddr *)&address, sizeof(address));
    listen(fd, 3);

    printf("Waiting for connections...\n");

    while (1) {
    new_fd = accept(fd, (struct sockaddr *)&address, &length);
    if (new_fd < 0) {
        printf("Failed to accept connection\n");
        continue;
    }

    while (1) {  // <-- keep replying to one client
        int bytes_received = recv(new_fd, buffer, sizeof(buffer), 0);
        if (bytes_received <= 0) {
            break;  // client disconnected
        }

        buffer[bytes_received] = '\0';  // null-terminate the buffer
        printf("Received from client: %s\n", buffer);
        send(new_fd, response, strlen(response), 0);
    }

    closesocket(new_fd);
}

    
    closesocket(new_fd);
    closesocket(fd);
    WSACleanup();
    return 0;
}
