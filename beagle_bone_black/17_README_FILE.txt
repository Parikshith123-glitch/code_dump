If we assigned 4096 bytes initially, for a particular shmid (we can check this using $ shmcs -m), for this id this much memory is assigned until the 
end and if we try to change the SIZE (ex to 10000 bytes) in this .c code to a size which is more than the assigned size then we get INVALID ARGUMENT in
terminal, this is because we have assigned 2048 bytes of SHARED MEMORY and we are trying to access more that this. 

This can be checked with $ ipcs -m and we can remove this MEMORY SPACE using $ ipcrm -m <shmid>, this removes the memory itself. 

This can be avoided using the following line of code that should be added after the detachment in the receiver.c file 

-     shmdt(ptr);					   // Detaching the pointer from the shared memory
-     shmctl(shm_id, IPC_RMID, NULL);  // Cleanup done here

By adding this every time we execute the memory will be cleaned / cleared. We can assign any amount of shared memory space.
