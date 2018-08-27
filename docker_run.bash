sudo docker run -it \
    --net=host \
    --name="drone" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    osrf/ros:indigo-desktop-full \
    bash
export containerId=$(sudo docker ps -l -q)