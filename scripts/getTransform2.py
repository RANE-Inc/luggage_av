#!/usr/bin/env python3

import rclpy
import tf2_py as tf2
import yaml
import subprocess
from tf2_msgs.srv import FrameGraph
import tf2_ros
import time

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('view_frames')

    buffer = tf2_ros.Buffer(node=node)
    listener = tf2_ros.TransformListener(buffer, node, spin_thread=False)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # listen to tf for 5 seconds
    node.get_logger().info('Listening to tf data during 5 seconds...')
    start_time = time.time()
    while (time.time() - start_time) < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info('Getting transform from map to base_link...')
    try:
        transform = buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        node.get_logger().info('Transform: {}'.format(transform))
    except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
        node.get_logger().error('Failed to get transform: {}'.format(e))


if __name__ == '__main__':
    main()
