#include <stdio.h>
#include <sys/ipc.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/shm.h>
#include <string.h>

//#define SIZE 10000
#define SIZE 4096
//#define SIZE 2048
//#define SIZE 1024

int main()
{     key_t shm_key = 123;

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

char last_message[1024] = {0};


//while (1) {
  //      if (strcmp(ptr, last_message) != 0 && strlen(ptr) > 0) {
    //        printf("Received: %.1024s...\n", ptr);  // limit output to 100 chars for readability

        //              strncpy(last_message, ptr, sizeof(last_message) - 1);
        //              last_message[sizeof(last_message) - 1] = '\0';  // ensure null-termination
    //  }
    //    usleep(100000);  // Sleep 100ms to reduce CPU usage
    //}

        fwrite(ptr, 1, SIZE, stdout);
//    	fflush(stdout);  // flush stdout buffer

    	shmdt(ptr);
//      shmctl(shm_id, IPC_RMID, NULL);  // Cleanup done here

        return 0;
}
