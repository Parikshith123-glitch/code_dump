#include <stdio.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <string.h>
#include <stdlib.h> // for exit conditions

struct msg_buffer
{
        long mtype;
        char message[100];
};

int main()
{
        key_t msg_key = 123;
        char choice;
        char string_to_send[100];

        int my_id = msgget(msg_key, 0666 | IPC_CREAT); // will return an msg queue identifier
        if (my_id == -1)
        {
                perror("msgget");
        } else {
                printf("msg queue created successfully\n");
                printf("The msg id is %d\n", my_id);
        }

while (1) {
    printf("Do you want to continue: ");
    scanf(" %c", &choice);  // NOTE: space before %c to consume newline

    struct msg_buffer buffer;
    buffer.mtype = 1;

    if (choice == 'y') {
        printf("Enter the string to send: ");
        getchar();  // consume leftover newline after scanf
        fgets(string_to_send, sizeof(string_to_send), stdin);
        string_to_send[strcspn(string_to_send, "\n")] = 0;  // strip newline

        strcpy(buffer.message, string_to_send);
        msgsnd(my_id, &buffer, sizeof(buffer.message), 0);
    } else {
        strcpy(buffer.message, "Nothing to send end this here\n");
        msgsnd(my_id, &buffer, sizeof(buffer.message), 0);
        exit(1);
    }

    printf("Msg sent: %s\n", buffer.message);
}

        return 0;
}
