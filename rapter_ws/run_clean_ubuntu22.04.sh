#!/bin/bash

docker run -m=5g --rm --gpus all -it --net=host --ipc=host --pid=host --privileged -v /dev/shm:/dev/shm -e DISPLAY=$DISPLAY -v /dev/input:/dev/input:rw -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v $(pwd)/:/home/root:rw -w /home/root --name=sim_class2_docker u22_ros2_docker:v1 /bin/bash -l

