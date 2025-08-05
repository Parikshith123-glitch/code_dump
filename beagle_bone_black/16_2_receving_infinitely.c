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
        int msgid = msgget(msg_key, 0666); // get the same message queue
        if (msgid == -1) {
        perror("msgget");
        exit(1);
        }

        struct msg_buffer buffer;

while(1)
{
       int ret_value = msgrcv(msgid, &buffer, sizeof(buffer.message), 1,0);

        if(ret_value == -1)
        {
                perror("msgrcv");
                exit(1);
        } else {
                printf("Received Message: %s\n", buffer.message);
        }
}
        return 0;
}
