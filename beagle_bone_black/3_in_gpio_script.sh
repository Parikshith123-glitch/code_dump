#!/bin/bash

count=0
PATH="/sys/class/gpio/gpio7" # Pin 9_42

led_value=0;

echo "in" > ${PATH}/direction

for i in {0..3}; do echo "out" > /sys/class/leds/beaglebone:green:usr$i/direction;
done


echo "out" > /sys/class/leds/beaglebone:green:usr0/direction


while true;do
	# if the pin -> value is high then increment and echo the count (It gets 3.3V)

	value=$(<${PATH}/value)


    	if [ "$value" == "1" ];
	then	((count++))
		echo "Input has been given, the count is $count"

        if [ "$led_value" == 0 ]; then
            led_value=1
        else
            led_value=0
        fi
	for i in {0..3}; do echo $led_value > /sys/class/leds/beaglebone:green:usr$i/brightness; done

	/bin/sleep 0.5
	fi
done
