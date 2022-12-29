#!/usr/bin/env bash

#
# Graphically Recursive Simultaneous Task Allocation, Planning,
# Scheduling, and Execution
#
# Copyright (C) 2020-2021
#
# Author: Andrew Messing
# Author: Glen Neville
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
if [ $# -lt 1 ]
then
  echo "Usage: $0 <target>"
  echo -e "\t<target> ::= remote|nvidia"
  exit 1
fi

# to lower case
target="${1,,}"

if [ "$target" == "nvidia" ]
then
  # If the container is running stop it
  if [ "$( docker container inspect -f '{{.State.Running}}' remote_ros_nvidia_dev )" == "true" ]; then
    docker container stop remote_ros_nvidia_dev
  fi

  TMP_LINE="$(nvidia-smi | grep NVIDIA-SMI)"
  REGEX="Driver Version: ([0-9]{3})\."
  if [[ "$TMP_LINE" =~ $REGEX ]]
  then
    NVIDIA_DRIVER_VERSION="${BASH_REMATCH[1]}"
    echo "Nvidia driver version: ${NVIDIA_DRIVER_VERSION}"
  else
    echo "Error: Cannot find Nvidia driver version"
    exit 1
  fi

  CUSTOM_USER=debugger

  branch_name="$(git symbolic-ref HEAD 2>/dev/null)"

  echo "Building nvidia docker"
  docker build -f grstapse.dockerfile \
    --build-arg NVIDIA_DRIVER_VERSION="$NVIDIA_DRIVER_VERSION" \
    --build-arg CUSTOM_USER=$CUSTOM_USER \
    --target REMOTE_ROS_NVIDIA \
    -t amessing/grstapse:nvidia . || { echo "Build docker failed"; exit 1; }

  # Connect the existing X11 socket on the local machine to the one in the docker
  XSOCK=/tmp/.X11-unix
  # Create a new socket and mounting it to the docker
  XAUTH=/tmp/.docker.xauth
  if [ ! -f $XAUTH ]
  then
      xauth_list=$(xauth nlist $DISPLAY | sed -e 's/^..../ffff/')
      if [ ! -z "$xauth_list" ]
      then
          echo $xauth_list | xauth -f $XAUTH nmerge -
      else
        touch $XAUTH
      fi
      chmod a+r $XAUTH
  fi

  echo "Running nvidia docker"
  docker run \
      -e DISPLAY=$DISPLAY \
      -e XAUTHORITY=$XAUTH \
      -e QT_X11_NO_MITSHM=1 \
      --gpus all \
      --hostname localhost \
      --ipc=host \
      -it \
      --name remote_ros_nvidia_dev \
      --net=host \
      --privileged \
      --rm \
      --runtime=nvidia \
      --security-opt seccomp=unconfined \
      -v $XSOCK:$XSOCK:rw \
      -v $XAUTH:$XAUTH \
      -v $PWD/configs/tmux.conf:/home/$CUSTOM_USER/.tmux.conf \
      -v $PWD/gurobi/gurobi.lic:/opt/gurobi/gurobi.lic:ro \
      amessing/grstapse:nvidia

  #xhost -local:docker
elif [ "$target" == "remote" ]
then
  # If the container is running stop it
    if [ "$( docker container inspect -f '{{.State.Running}}' remote_dev )" == "true" ]; then
      docker container stop remote_dev
    fi

  docker build -f grstapse.dockerfile \
    --target REMOTE \
    -t amessing/grstapse:remote . || { echo "Build docker failed"; exit 1; }

  docker run \
          -d \
          -it \
          --name remote_dev \
          --hostname localhost \
          --ipc=host \
          --net=host \
          --privileged \
          --rm \
          --security-opt seccomp=unconfined \
          -v $PWD/configs/tmux.conf:/home/$CUSTOM_USER/.tmux.conf \
          -v $PWD/gurobi/gurobi.lic:/opt/gurobi/gurobi.lic:ro \
          amessing/grstapse:remote
else
  echo "Unknown target"
  echo "Usage: $0 <target>"
  echo -e "\t<target> ::= remote|nvidia"
  exit 1
fi
