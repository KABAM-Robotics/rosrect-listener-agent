

#chmod +x rosrect-ecs-api-server/src/ecs_endpoint.pynohup python3 rosrect-ecs-api-server/src/ecs_endpoint.py &

#Troubleshooting
export ECS_API=http://0.0.0.0:8000
export AGENT_POST_API=https://postman-echo.com
echo "Checking ECS API Val"
echo $ECS_API


pip3 install -r rosrect-ecs-api-server/requirements.txt
python3 rosrect-ecs-api-server/src/ecs_endpoint.py &
APP_PID=$!