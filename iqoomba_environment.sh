#!/bin/bash

if [ $# -eq 1 ]; then
	echo export ROS_HOSTNAME=$1
	echo export ROS_IP=$1
	echo export ROS_MASTER_URI=http://$1:11311


	export ROS_HOSTNAME=$1
	export ROS_IP=$1
	export ROS_MASTER_URI=http://$1:11311
else
    echo "IP of system running ROSCORE required"
fi



