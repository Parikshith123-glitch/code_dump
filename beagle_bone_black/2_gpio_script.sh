#!/bin/bash

PATH="/sys/class/gpio/gpio7"

echo "gpio7" > /sys/class/gpio/export

echo "out" > ${PATH}/direction
while true;do
           echo "1" > ${PATH}/value

/bin/sleep 0.4
	   echo "0" > ${PATH}/value
/bin/sleep 0.4
done
