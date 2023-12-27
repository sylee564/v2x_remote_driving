#!/bin/sh
echo "---------- CAN OPEN ----------"
echo "avgenius1" | sudo -S modprobe can_dev
echo "avgenius1" | sudo -S modprobe can
echo "avgenius1" | sudo -S modprobe can_raw
 
echo "avgenius1" | sudo -S ip link set can0 type can bitrate 500000    
echo "avgenius1" | sudo -S ip link set up can0
