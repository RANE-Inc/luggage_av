#!/bin/bash


echo "Side ${side}: Turning right 90 degrees..."
ros2 action send_goal /luggage_av/spin nav2_msgs/action/Spin \
    "{target_yaw: -0.8, time_allowance: {sec: 10, nanosec: 0}}" # TODO: Calibrate

# Loop for 4 sides of the square
for side in {1..4}; do
    echo "Side ${side}: Driving forward 0.5 meters..."
    ros2 action send_goal /luggage_av/drive_on_heading nav2_msgs/action/DriveOnHeading \
      "{target: {x: 0.3, y: 0.0, z: 0.0}, speed: 1.0, time_allowance: {sec: 3, nanosec: 0}}" # TODO: Calibrate

    echo "Side ${side}: Turning left 90 degrees..."
    ros2 action send_goal /luggage_av/spin nav2_msgs/action/Spin \
      "{target_yaw: 0.8, time_allowance: {sec: 10, nanosec: 0}}" # TODO: Calibrate
done

echo "Side ${side}: Turning 180 degrees..."
ros2 action send_goal /luggage_av/spin nav2_msgs/action/Spin \
    "{target_yaw: 0.8, time_allowance: {sec: 10, nanosec: 0}}" # TODO: Calibrate

echo "Square path complete."
