#!/bin/bash

config-pin P9_11 uart	# This is UART4
config-pin P9_13 uart


#config-pin P9_21 uart	# This is UART2
#config-pin P9_22 uart

#config-pin P8_37 uart	# This is UART5 - getting error for this
#config-pin P8_38 uart

#config-pin P9_24 uart
#config-pin P9_26 uart

gcc uart_testing.c -o output_testing_uart
#gcc 7_UART_Loopback.c -o output_7
#gcc 8_uart_continous.c -o output_8
#gcc 9_between_2_uarts.c -o output_9
#gcc 10_uart_sending_raw_data.c -o output_10
gcc 11_pic32_bbb_uart_comms.c -o output_11
#gcc test.c -o output_test
