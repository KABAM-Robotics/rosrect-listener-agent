#!/bin/bash

clear

docker build -t error_resolution_diagnoser .

docker stop agent

docker rm agent 

docker run -it \
--env-file runtime.env \
--network=host \
--name=agent  \
--volume="${HOME}/.cognicept/agent/logs:/root/.cognicept/agent/logs" \
error_resolution_diagnoser:latest  \
roslaunch error_resolution_diagnoser error_resolution_diagnoser.launch 

# --env="ROS_MASTER_URI=http://localhost:11311" \