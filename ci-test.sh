#Rewriting the test file

source /opt/ros/melodic/setup.bash

# git clone the folder here 

mkdir -p catkin_test/src
cd catkin_test/src
git init 
git clone https://github.com/cognicept-admin/rosrect-listener-agent.git
cd rosrect-listener-agent
git fetch && git checkout travis-ci
cd ..

#Troubleshooting
export ECS_API=http://0.0.0.0:8000
export AGENT_POST_API=https://postman-echo.com
echo "Checking ECS API Val"
echo $ECS_API

# move back a folder for catkin_make
cd ..
# catkin_make
# catkin_make run_tests