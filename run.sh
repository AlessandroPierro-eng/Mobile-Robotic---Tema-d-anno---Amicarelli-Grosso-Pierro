#!/bin/bash
echo "Avvio del container security_fleet_container..."
xhost +local:root

docker run -it --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd)/ros_ws:/root/ros_workspace" \
    --name security_fleet_container \
    mr05_project_image \
    bash