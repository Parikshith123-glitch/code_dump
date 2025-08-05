**process in linux**



For the parent, fork() returns the process ID of the newly created child. This is useful because the parent may create, and thus need to track several children (via wait() or one of its relatives). For the child, fork() returns 0.

If necessary, the child can obtain its own process ID using getpid().



by getppid() - parent id can be obtained.



When your program runs (say, you compile and run ./prog), here's what happens:



The shell (or some process) starts your program.

That program becomes a process with:



Its own Process ID (getpid())

A Parent Process ID (getppid()), which is typically your shell (like bash or sh)



So before any fork(), your program is already a running process — created by a parent (e.g., the shell).



**------------------------------------------------------------------------------------------**



Now, what does fork() do?

fork() duplicates the current process.

It creates a new child process, almost identical to the parent.



Both parent and child continue running independently.

The \*\*child gets a new unique PID\*\*, but shares the same code, file descriptors, environment, etc.



**------------------------------------------------------------------------------------------**



There is a possibility that the main thread may end before the execution of the created thread thread1 and it may lead to unexpected behaviour of the program. So, there is a functionality in C to wait for the execution of the particular thread (pthread\\\_join\\\*)(thread\\\_id, NULL)).



**------------------------------------------------------------------------------------------**



**For inter-process communication (IPCs)**



Now global variable is done. In this we are changing the value of the global value and the other 2 threads are taking this variable and printing by using mutex.

Also used -



pthread\_mutex\_t mutex = PTHREAD\\\_MUTEX\\\_INITIALIZER;



pthread\_cond\_t cond = PTHREAD\\\_COND\\\_INITIALIZER;



and next is memory queues and shared memory (also to use semaphores)



**------------------------------------------------------------------------------------------**

**Using message queues**

