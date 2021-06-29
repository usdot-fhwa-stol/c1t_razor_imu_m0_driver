#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.sh
catkin_make_isolated --install --only-pkg-with-deps c1t_razor_imu_m0_driver -DCMAKE_BUILD_TYPE=Release
