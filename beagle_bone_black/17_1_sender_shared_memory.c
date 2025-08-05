#include <stdio.h>
#include <sys/ipc.h>
#include <stdlib.h>
#include <sys/shm.h>
#include <string.h>

//#define SIZE 10000
#define SIZE 4096
//#define SIZE 2048
//#define SIZE 1024

int main()
{       key_t shm_key = 123;
        char string_to_send[SIZE];

        int shm_id = shmget(shm_key, SIZE, 0666 | IPC_CREAT);
        if(shm_id == -1)
        {
                perror("shmget");
                exit(1);
        }

        char *ptr = (char *)shmat(shm_id, NULL, 0);
        if (ptr == (char *)(-1)) {
        perror("shmat");
        exit(1);
    }

//while(1)
//{
//        printf("Enter the string to send: ");

//              getchar();                                              // consume leftover newline after scanf
//        fgets(string_to_send, sizeof(string_to_send), stdin);
//        string_to_send[strcspn(string_to_send, "\n")] = 0;  // strip newline

//              strcpy(ptr, string_to_send);
//}

        memset(ptr, 'A', SIZE - 1);
        ptr[SIZE - 1] = '\0';  // null-terminate to make it a valid C string

        shmdt(ptr);

        return 0;
}
