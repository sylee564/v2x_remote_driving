#!/bin/sh
echo "---------- CAN OPEN ----------"
echo "1234" | sudo -S modprobe can_dev
echo "1234" | sudo -S modprobe can
echo "1234" | sudo -S modprobe can_raw
 
echo "1234" | sudo -S ip link set can0 type can bitrate 500000    
echo "1234" | sudo -S ip link set up can0
