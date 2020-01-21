#!/bin/bash

if [ $# -eq 1 ]; then
    echo '$1 = ' $1
else
    echo "IP of ROS master required"
    exit
fi

export ROS_HOSTNAME=$1
export ROS_IP=$1
export ROS_MASTER_URI=http://$1:11311

roslaunch iqoomba_2dnav iqoomba_configuration.launch
