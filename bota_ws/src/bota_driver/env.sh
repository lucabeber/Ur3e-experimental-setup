 
#!/bin/bash

# Copyright (c) 2015, Isaac I. Y. Saito
# Copyright (c) 2017, Mathias Lüdtke
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
export OS_NAME=${OS_NAME:-ubuntu}

function  _ros1_defaults {
    export OS_CODE_NAME=${OS_CODE_NAME:-$1}
    export ROS1_DISTRO=${ROS1_DISTRO:-$ROS_DISTRO}
    export BUILDER=${BUILDER:-catkin_tools}
    export ROS_VERSION=1
    export ROS_VERSION_EOL=false
    export ROS_PYTHON_VERSION=${ROS_PYTHON_VERSION:-2}
}

function  _ros2_defaults {
    export OS_CODE_NAME=${OS_CODE_NAME:-$1}
    export ROS2_DISTRO=${ROS2_DISTRO:-$ROS_DISTRO}
    export BUILDER=${BUILDER:-colcon}
    export ROS_VERSION=2
    export ROS_VERSION_EOL=false
    export ROS_PYTHON_VERSION=3
}

function _set_ros_defaults {
    case "$ROS_DISTRO" in
    "indigo"|"jade")
        _ros1_defaults "trusty"
        export ROS_VERSION_EOL=true
        ;;
    "kinetic")
        _ros1_defaults "xenial"
        ;;
    "lunar")
        _ros1_defaults "xenial"
        export ROS_VERSION_EOL=true
        ;;
    "melodic")
        _ros1_defaults "bionic"
        ;;
    "noetic")
        export BUILDER=${BUILDER:-colcon}
        _ros1_defaults "focal"
        export ROS_PYTHON_VERSION=3
        ;;
    "ardent")
        _ros2_defaults "xenial"
        export ROS_VERSION_EOL=true
        ;;
    "bouncy"|"crystal")
        _ros2_defaults "bionic"
        export ROS_VERSION_EOL=true
        ;;
    "dashing")
        _ros2_defaults "bionic"
        ;;
    "eloquent")
        _ros2_defaults "bionic"
        ;;
    "foxy")
        _ros2_defaults "focal"
        ;;
    "rolling")
        _ros2_defaults "focal"
        ;;
    "false")
        unset ROS_DISTRO
        ;;
    *)
        echo "ROS_DISTRO '$ROS_DISTRO' is not supported"
        exit 1
        ;;
    esac

    if [ "$ROS_PYTHON_VERSION" = 2 ]; then
        export PYTHON_VERSION_NAME=python
    elif [ "$ROS_PYTHON_VERSION" = 3 ]; then
        export PYTHON_VERSION_NAME=python3
    fi

}

if [ -n "${ROS_DISTRO}" ]; then
    _set_ros_defaults
fi