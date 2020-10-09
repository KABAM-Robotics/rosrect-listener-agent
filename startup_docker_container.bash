#!/bin/bash

# clear

docker build -t error_resolution_diagnoser .

docker stop cgs_diagnostics_agent

docker rm cgs_diagnostics_agent 

docker run -d \
--env-file ~/.cognicept/runtime.env \
--restart unless-stopped \
--network=host \
--name=cgs_diagnostics_agent  \
--volume="${HOME}/.cognicept/agent/logs:/root/.cognicept/agent/logs" \
error_resolution_diagnoser:latest  \
rosrun error_resolution_diagnoser error_resolution_diagnoser 