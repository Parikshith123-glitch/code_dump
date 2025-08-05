#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

void* thread_func(void*);

int main()
{
	pid_t pid = fork();
	pthread_t thread_1;
	int data = 47;

    if (pid == 0) {
        // This is the child process
        printf("Child: PID = %d, PPID = %d\n", getpid(), getppid());
    } if (pid > 0){
        // This is the parent process
        printf("Parent: PID = %d, PPID = %d\n", getpid(), getppid());
    }

	pthread_create(&thread_1, NULL, thread_func, &data);
	printf("Thread ID: %lu\nProcess ID: %d\nParent Process ID: %d\n", (unsigned long)pthread_self(), getpid(), getppid());

	pthread_join(thread_1, NULL);

	return 0;
}

void* thread_func(void* data)
{
	int value = *(int *)data;
	printf("This is inside the function call and the attribute obtained %d\n", value);
	return NULL;
}
