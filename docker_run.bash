sudo docker run -it \
    --net=host \
    --name="pidrone" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device /dev/snd:/dev/snd \
    pidrone \
    bash
export containerId=$(sudo docker ps -l -q)