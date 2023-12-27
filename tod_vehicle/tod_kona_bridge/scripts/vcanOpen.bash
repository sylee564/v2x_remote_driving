#!/bin/bash
echo "---------- CAN OPEN ----------"
echo "1234" | sudo -S modprobe vcan
echo "1234" | sudo -S ip link add name vcan0 type vcan
echo "1234" | sudo -S ip link set up vcan0
