#gcc 14_process_threads.c -o output_14_pthread -pthread
#gcc 15_threads_sharing.c -o output_15 -pthread

#gcc 16_sender_message_queue.c -o output_sender_16
#gcc 16_receiver_message_queue.c -o output_receiver_16

#gcc 16_2_sending_infinately.c -o output_send_16_2
#gcc 16_2_receiving_infinately.c -o output_rec_16_2

#gcc 17_1_sender_shared_memory.c -o output_17_1_send
#gcc 17_1_receiver_shared_memory.c -o output_17_1_receive

#gcc 18_threads_que_sha_mem.c -o output_18 -pthread
#gcc 19_semaphore_threads.c -o output_19 -pthread

#gcc 20_trying_race_cond_shared_mem.c -o output_20 -pthread

#gcc 21_1_server_tcp.c -o output_server
#gcc 21_1_client_tcp.c -o output_client

gcc server_udp.c -o server_output_udp
gcc client_udp.c -o client_output_udp
