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
HOST_USER_NAME="$(id -un)"
DOCKER_USER_NAME=debugger

ssh-keygen -f "/home/${HOST_USER_NAME}/.ssh/known_hosts" -R "[localhost]:7776"
ssh localhost -p 7776 -l ${DOCKER_USER_NAME} -Y