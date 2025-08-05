#include <stdio.h>

#include <stdlib.h>
#include <time.h>

#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <semaphore.h>
#include <time.h>

#define SIZE 2048

void *thread_func(void *arg);

struct thread_data {
    char* shm_ptr;     // Pointer to shared memory
    char* message;     // String to write
};

int main()
{	// 1. creation of shared memory.
	// 2. function that takes a struct(shm_pointer and string) as a argument and prints which thread is using the shared memeory.
	// 3. detach the shared memory just before the return 0; statement.

	srand(time(NULL));

/*     ----------Creation of shared memory--------      */

	key_t shmkey = 123;
	int shmid = shmget(shmkey, SIZE, 0666 | IPC_CREAT);
	if(shmid == -1)
	{
		perror("shmget");
		exit(1);
	}

	char *ptr = (char*)shmat(shmid, NULL, 0);
	if(ptr == (char*)(-1))
	{
		perror("shmat");
		exit(1);
	}

	pthread_t thread1, thread2, thread3;

	char string1[] = "This is from thread1";
	char string2[] = "This is from thread2";
	char string3[] = "This is from thread3";

	struct thread_data buffer1;
	buffer1.shm_ptr = ptr;
	buffer1.message = string1;

	struct thread_data buffer2;
	buffer2.shm_ptr = ptr;
	buffer2.message = string2;

	struct thread_data buffer3;
	buffer3.shm_ptr = ptr;
	buffer3.message = string3;

	pthread_create(&thread1, NULL, thread_func, &buffer1);
	pthread_create(&thread2, NULL, thread_func, &buffer2);
	pthread_create(&thread3, NULL, thread_func, &buffer3);

	pthread_join(thread1, NULL);
	pthread_join(thread2, NULL);
	pthread_join(thread3, NULL);

	shmdt(ptr);
	shmctl(shmid, IPC_RMID, NULL);	

	return 0;
}

void *thread_func(void *arg)
{
	usleep(rand() % 10000);  // Up to 10ms delay

	struct thread_data *rec_buff = (struct thread_data* )arg;

	// struct thread_data rec_buff;
	strcpy(rec_buff->shm_ptr, rec_buff->message);
	printf("%s", rec_buff->shm_ptr);
	printf("\n");
}
