docker build -t ros-humble -f Dockerfile.ros .
xhost +local:
docker run -it --rm --net=host --env="DISPLAY" --env="ROS_DOMAIN_ID" \
    -v ~/fourier-sim/IsaacSim-ros_workspaces/humble_ws:/humble_ws:rw \
    -v ~/fourier-sim:/fourier-sim:rw \
    --name ros_ws_docker ros-humble /bin/bash