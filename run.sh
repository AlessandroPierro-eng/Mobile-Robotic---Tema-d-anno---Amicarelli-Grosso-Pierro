# #!/bin/bash
# echo "Avvio del container security_fleet_container..."
# xhost +local:root

# docker run -it --rm \
#     --net=host \
#     --env="DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#     --volume="$(pwd)/ros_ws:/root/ros_workspace" \
#     --name security_fleet_container \
#     mr05_project_image \
#     bash

# Run che sfrutta la GPU NVIDIA
#!/bin/bash
echo "Avvio del container security_fleet_container..."
xhost +local:root

if command -v nvidia-smi &> /dev/null; then
    echo "GPU NVIDIA rilevata. Abilitazione accelerazione hardware..."
    GPU_FLAGS="--gpus all --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all"
else
    echo "Nessuna GPU NVIDIA rilevata o driver mancanti. Avvio in modalita' CPU..."
    GPU_FLAGS=""
fi

docker run -it --rm \
    $GPU_FLAGS \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd)/ros_ws:/root/ros_workspace" \
    --name security_fleet_container \
    mr05_project_image \
    bash