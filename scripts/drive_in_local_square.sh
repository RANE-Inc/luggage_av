#!/bin/bash

# Loop for 4 sides of the square

    

echo "Side ${side}: Turning left 90 degrees..."
ros2 action send_goal /luggage_av/spin nav2_msgs/action/Spin \
    "{target_yaw: -1.57, time_allowance: {sec: 10, nanosec: 0}}"
    
for side in {1..4}; do
    echo "Side ${side}: Driving forward 2 meters..."
    ros2 action send_goal /luggage_av/drive_on_heading nav2_msgs/action/DriveOnHeading \
      "{target: {x: 0.5, y: 0.0, z: 0.0}, speed: 1.0, time_allowance: {sec: 3, nanosec: 0}}"
    

    echo "Side ${side}: Turning left 90 degrees..."
    ros2 action send_goal /luggage_av/spin nav2_msgs/action/Spin \
      "{target_yaw: 1.57, time_allowance: {sec: 10, nanosec: 0}}"
done

echo "Side ${side}: Turning left 90 degrees..."
ros2 action send_goal /luggage_av/spin nav2_msgs/action/Spin \
    "{target_yaw: 1.57, time_allowance: {sec: 10, nanosec: 0}}"
    
echo "Square path complete."
