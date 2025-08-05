#include <stdio.h>
#include <unistd.h>		// for usleep()
#include <string.h>
#include <stdlib.h> 	// for exit conditions

#include <pthread.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/shm.h>

#define SIZE 4096

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
int step = 0;

struct msg_buffer
{
	long mtype;
	char message[100];
};

void *thread1_func(void* arg);
void *thread2_func(void* arg);
void *thread3_func(void* arg);

int main()
{	pthread_t thread1, thread2, thread3;

	char string_sent[100] = "From Main Process";

	pthread_create(&thread1, NULL, thread1_func, NULL);

	pthread_create(&thread2, NULL, thread2_func, NULL);

	pthread_create(&thread3, NULL, thread3_func, NULL);

	pthread_join(thread1, NULL);
	pthread_join(thread2, NULL);
	pthread_join(thread3, NULL);

	return 0;
}

void *thread3_func(void* arg)
{

	pthread_mutex_lock(&mutex);
	while(step < 2)
		pthread_cond_wait(&cond, &mutex);

	key_t shm_key = 123;
	int shm_id = shmget(shm_key, SIZE, 0666 | IPC_CREAT);
	if(shm_id == -1)
	{
		perror("shmget");
		exit(1);
	}

	char *ptr = (char* )shmat(shm_id, NULL, 0);
	if(ptr == (char* )(-1))
	{
		perror("shmat");
		exit(1);
	}

    fwrite(ptr, 1, strlen(ptr), stdout);	// Don't print full SIZE of memory
 	printf("\nThread3: Reading from shared memory:\n");


	pthread_mutex_unlock(&mutex);

	shmctl(shm_id, IPC_RMID, NULL);		  	// This is for removing the newly created shared memory.

	shmdt(ptr);

}


void *thread2_func(void* arg)
{	// in this func, we receive the string from msg que and create a shared memory and put this string in that shared memory

		pthread_mutex_lock(&mutex);
		while(step < 1)
			pthread_cond_wait(&cond, &mutex);
		/* ------------ Creation of shared memory ----------------- */
		key_t shm_key = 123;
		int shm_id = shmget(shm_key, SIZE, 0666 | IPC_CREAT);
		if(shm_id == -1)
		{
			perror("shmget");
			exit(1);
		}

		char* ptr = (char* )shmat(shm_id, NULL, 0);
		if(ptr == (char* )(-1))
		{
			perror("shmat");
			exit(1);
		}

		/* --------------- Creation of message queue --------------*/

        key_t msg_key = 123;
        int msgid = msgget(msg_key, 0666); // get the same message queue
        if (msgid == -1)
		{
	        perror("msgget");
        	exit(1);
        }

		/* ---------  Receiving the message from message queue ----- */
        struct msg_buffer buffer;
        int ret_value = msgrcv(msgid, &buffer, sizeof(buffer.message), 1,0);

        if(ret_value == -1)
        {
                perror("msgrcv");
                exit(1);
        } else {
				strcat(buffer.message, "\nThread2: After Receiving and Creating Shared Memory");
//                printf("%s\n", buffer.message);
        }

		/* ------ Sending the string to shared memory --------*/

		strcpy(ptr, buffer.message);

	step = 2;

	pthread_cond_broadcast(&cond);  			// signals all the threads to wake up cause it is releasing the mutex.
	pthread_mutex_unlock(&mutex);

		shmdt(ptr);
}


void *thread1_func(void* arg)
{  // Func to create msg queue and send string to it
	//struct msg_buffer* buff = (struct msg_buffer* )arg;
	//buff->mtype = 1;

	pthread_mutex_lock(&mutex);
	key_t msg_key = 123;

        int my_id = msgget(msg_key, 0666 | IPC_CREAT); // will return an msg queue identifier
        if (my_id == -1)
        {
                perror("msgget");
        } else {
                printf("Message Queue created successfully\n");
        }

        // buffer is a struct and we are not using it but we need the string we got from the main process and add a signature from thread1.

		struct msg_buffer buffer;
		buffer.mtype = 1;
		strcpy(buffer.message, "Thread1: After creation of message queue");

        int ret_value = msgsnd(my_id, &buffer, sizeof(buffer.message), 0);
		if(ret_value == -1)
        {
                perror("msgsnd");
                exit(1);
        }

	step = 1;
	pthread_cond_broadcast(&cond);  			// signals all the threads to wake up cause it is releasing the mutex.
	pthread_mutex_unlock(&mutex);

//        printf("%s\n", buffer.message);
}
