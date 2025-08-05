#!/bin/bash

count=0
PATH="/sys/class/leds/beaglebone:green:usr"

while true; do
	for i in {0..3}; do
		echo 1 > ${PATH}${i}/brightness
		echo "LED $i is ON"
		/bin/sleep 0.5
	done
	for i in {0..3}; do
		echo 0 > ${PATH}${i}/brightness
		echo "LED $i is OFF"
		/bin/sleep 0.5
	done
	((count++))
	echo "The number of blinks is $count"
done
