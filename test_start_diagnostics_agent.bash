#!/bin/bash

clear

docker build -t rosrect_agent_ros .

docker stop agent

docker rm agent 

docker run -it \
--env-file runtime.env \
--network=host \
--name=agent  \
--volume="${HOME}/.cognicept/agent/logs:/root/.cognicept/agent/logs" \
rosrect_agent_ros:latest  \
roslaunch rosrect-listener-agent listener-agent.launch 

# --env="ROS_MASTER_URI=http://localhost:11311" \