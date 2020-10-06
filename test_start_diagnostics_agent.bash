#!/bin/bash

# clear

docker build -t cognicept_diagnostics_agent:jeff .

docker stop cgs_diagnostics_agent

docker rm cgs_diagnostics_agent 

docker run -d \
--env-file ~/.cognicept/runtime.env \
--restart unless-stopped \
--network=host \
--name=cgs_diagnostics_agent  \
--volume="${HOME}/.cognicept/agent/logs:/root/.cognicept/agent/logs" \
cognicept_diagnostics_agent:jeff  \
rosrun rosrect-listener-agent rosrect-listener-agent
