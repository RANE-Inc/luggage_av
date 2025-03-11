#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from tf2_ros import LookupException
import sys
import math

class BaseLinkToMapListener(Node):

    def __init__(self, first_frame='base_link', second_frame='map'):
        super().__init__('base_link_to_map_listener')
        namespace = self.get_namespace().lstrip('/')
        self.first_name_ = f'{namespace}/' + first_frame
        self.second_name_ = f'{namespace}/' + second_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.cmd_ = Twist ()
        # self.publisher_ = self.create_publisher(Twist, "{}/cmd_vel".format(self.second_name_),10)
        
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            # Lookup the transform from map to base_link
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.first_name_,
                self.second_name_,
                rclpy.time.Time()  # Time (0 means latest available)
            )
            self.print_transform(transform)
            self.timer.cancel()
            # TODO: figure out how to end this properly and not just cancel the timer which results in hanging program

        except TransformException as ex:
            self.get_logger().info(f"Could not get transform: {ex}")

    def print_transform(self, transform: TransformStamped):
        # self.get_logger().info(f"Transform: {transform}")
        self.get_logger().info(f"Translation: [x: {transform.transform.translation.x}, "
                              f"y: {transform.transform.translation.y}, "
                              f"z: {transform.transform.translation.z}]")
        self.get_logger().info(f"Rotation: [x: {transform.transform.rotation.x}, "
                              f"y: {transform.transform.rotation.y}, "
                              f"z: {transform.transform.rotation.z}, "
                              f"w: {transform.transform.rotation.w}]")

def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkToMapListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()