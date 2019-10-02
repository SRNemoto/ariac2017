#!/bin/bash
# script for installing moveit on a machine

sudo apt-get install ros-kinetic-moveit-core
sudo apt-get install ros-kinetic-moviet-kinematics
sudo apt-get install ros-kinetic-moveit-ros-planning
sudo apt-get install ros-kinetic-moveit-ros-move-group
sudo apt-get install ros-kinetic-moveit-planners-ompl
sudo apt-get install ros-kinetic-moveit-ros-visualization
sudo apt-get install ros-kinetic-moveit-simple-controller-manager
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-ros-controllers
sudo apt-get install ros-kinetic-ros-control

cd ~
mkdir -p catkin_ws/src

cd ~
git clone https://github.com/osrf/universal_robot -b ariac_ur10_moveit_config_kinetic
mv universal_robot/ur10_moveit_config catkin_ws/src
rm -rf universal_robot

cd ~/catkin_ws
catkin_make -j1

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
