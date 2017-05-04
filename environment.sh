#!/bin/bash

echo "Activating ROS Kinetic..."
source /opt/ros/kinetic/setup.bash
echo "...done."
echo

echo "Setting up PYTHONPATH."
echo "Note: We assume you cloned the Software repository in the folder 'duckietown' at home"
export PYTHONPATH=$DUCKIETOWN_ROOT/catkin_ws/src:$PYTHONPATH
echo

echo "Setup ROS_VARIABLES."
source $DUCKIETOWN_ROOT/ros_variables.sh
echo

echo "Building machines file..."
make -C  $DUCKIETOWN_ROOT
echo "...done"
echo

echo "Activating development."
source $DUCKIETOWN_ROOT/catkin_ws/devel/setup.bash

# TODO: check that the time is >= 2015

# TODO: run a python script that checks all libraries are installed

exec "$@" #Passes arguments. Need this for ROS remote launching to work.
