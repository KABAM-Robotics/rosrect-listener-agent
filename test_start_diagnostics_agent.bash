#!/bin/bash

clear

docker build -t cognicept_diagnostics_agent .

sudo docker run -it \
--rm \
--env-file runtime.env \
--network=host \
--name=cgs_diagnostics_agent  \
--volume="${HOME}/.cognicept/agent/logs:/root/.cognicept/agent/logs" \
cognicept_diagnostics_agent:latest  \
roslaunch rosrect-listener-agent listener-agent.launch 

# --env="ROS_MASTER_URI=http://localhost:11311" \