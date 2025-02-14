#!/usr/bin/bash

# TODO: Load namespace from cmdline
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/luggage_av -p stamped:=true -p frame_id:=luggage_av
