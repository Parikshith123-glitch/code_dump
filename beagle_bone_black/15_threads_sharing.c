#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

int shared_data = 0;
int data_ready = 0;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

void *sending_func(void* arg);
void *receiving_func(void* arg);

int main()
{
	pthread_t thread1, thread2, thread3, thread4, thread5;

	int attribute_sent = 47;

	pthread_create(&thread1, NULL, sending_func, &attribute_sent);
	pthread_create(&thread2, NULL, receiving_func, "thread2");
	pthread_create(&thread3, NULL, receiving_func, "thread3");

	pthread_join(thread1, NULL);
	pthread_join(thread2, NULL);
	pthread_join(thread3, NULL);

	return 0;
}

void *sending_func(void* arg)
{
	int value = *(int *)arg;
	pthread_mutex_lock(&mutex);

	shared_data = 100;
	data_ready = 1;
	pthread_cond_broadcast(&cond);
	pthread_mutex_unlock(&mutex);

	printf("This is inside the function and the attribute is %d\n", value);
	return NULL;
}

void *receiving_func(void* arg)
{
	char* value = (char *)arg;

	pthread_mutex_lock(&mutex);
	while(!data_ready)
	{
		pthread_cond_wait(&cond, &mutex);
	}
	printf("[%s] Received Data %d\n", value,shared_data);
//	printf("This is the receiving function and the name is: %s\n", value);

	pthread_mutex_unlock(&mutex);
	return NULL;
}
