#!/bin/bash

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

VOLUMES="--volume=$XSOCK:$XSOCK:rw
         --volume=$XAUTH:$XAUTH:rw"

nvidia-docker run \
    -it --rm \
    --name aslan_container \
    $VOLUMES \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    -u aslan_user \
    --privileged -v /dev/bus/usb:/dev/bus/usb \
    -v /dev/input:/dev/input \
    -v /lib/modules:/lib/modules \
    --net=host \
    projaslan/aslan:melodic
