#!/usr/bin/python3

#  Graphically Recursive Simultaneous Task Allocation, Planning,
#  Scheduling, and Execution
#
#  Copyright (C) 2020-2021
#
#  Author: Andrew Messing
#  Author: Glen Neville
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.

#  Graphically Recursive Simultaneous Task Allocation, Planning,
#  Scheduling, and Execution
#
#
#  Author: Andrew Messing
#  Author: Glen Neville
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.

#  Graphically Recursive Simultaneous Task Allocation, Planning, Scheduling, and Execution
#
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
import re

original_str = "typename ContainerAllocator::template rebind"
replacement_str = "typename std::allocator_traits<ContainerAllocator>::template rebind_alloc"

pattern = re.compile(
    r'std::vector<\s*([a-zA-Z:][a-zA-Z0-9:_<>]*)\s*,\s*typename\s+ContainerAllocator::template\s+rebind<\s*([a-zA-Z:][a-zA-Z0-9:_<>]*)\s*>::other\s*>')


def edit_file(filepath):
    # Read in the file
    with open(filepath, 'r') as file:
        filedata = file.read()

    # Replace the target string
    filedata = filedata.replace(
        'std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >',
        'std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>')

    def repl(m):
        return 'std::vector<{}, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<{}>>'.format(
            m.group(1), m.group(2))

    filedata = re.sub(pattern, repl, filedata)

    # print(filedata)
    # exit(1)

    # Write the file out again
    with open(filepath, 'w') as file:
        file.write(filedata)


def main():
    # msgs
    parent_path = "/opt/ros/noetic/include/"
    for entry in os.scandir(parent_path):
        if entry.is_dir() and entry.path.endswith("_msgs"):
            for subdir, dirs, files in os.walk(entry.path):
                for filename in files:
                    print("Editing: {}".format(subdir + os.sep + filename))
                    edit_file(subdir + os.sep + filename)
    # From roscpp
    edit_file("/opt/ros/noetic/include/ros/serialization.h")


if __name__ == "__main__":
    main()
