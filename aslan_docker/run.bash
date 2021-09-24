#!/bin/bash

docker run \
    -it --rm \
    --name aslan_container \
    --privileged \
    --net=host \
    -u aslan_user \
    --gpus all \
    --env DISPLAY \
    --env NVIDIA_VISIBLE_DEVICES=all \
    --env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
    --env LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
    -v /dev/dri:/dev/dri \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /dev/input:/dev/input \
    -v /lib/modules:/lib/modules \
    projaslan/aslan:melodic
