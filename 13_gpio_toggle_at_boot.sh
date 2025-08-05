#!/bin/bash

PIN=7
GPIO_PATH="/sys/class/gpio/gpio$PIN"

# Export if not already exported
if [ ! -d "$GPIO_PATH" ]; then
    echo "$PIN" > /sys/class/gpio/export
fi

echo "out" > ${GPIO_PATH}/direction

while true; do
    for i in {1..10}; do
        echo "1" > ${GPIO_PATH}/value
        sleep 0.5
        echo "0" > ${GPIO_PATH}/value
        sleep 0.5
    done
done
