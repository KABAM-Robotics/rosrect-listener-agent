

#chmod +x rosrect-ecs-api-server/src/ecs_endpoint.pynohup python3 rosrect-ecs-api-server/src/ecs_endpoint.py &

#Troubleshooting
export ECS_API=http://127.0.0.1:8000
export AGENT_POST_API=https://postman-echo.com
export ECS_DB_LOC=rosrect-ecs-api-server/ecs.db
echo "Checking ECS API Val"
echo $ECS_API


pip3 install -r rosrect-ecs-api-server/requirements.txt
python3 rosrect-ecs-api-server/src/ecs_endpoint.py &
APP_PID=$!

ls rosrect-ecs-api-server/
# cd rosrect-ecs-api-server
# docker build -t rosrect_ecs_api_server .

# docker run -it \
# --env-file runtime.env \
# -p 8000:8000 \
# --name=ecs_api_server  \
# --volume="${HOME}/rosrect-ecs-api-server/ecs.db:/root/.cognicept/ecs.db" \
# --network=host \
# rosrect_ecs_api_server:latest  \
# ecs_api_server/ecs_endpoint.py

