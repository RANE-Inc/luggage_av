#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped


# Tested with:      ros2 run luggage_av getTransform.py --ros-args -r /tf:=tf -r /tf_static:=tf_static -r __ns:=/luggage_av


class BaseLinkToMapListener(Node):

    def __init__(self, first_frame='base_link', second_frame='map'):
        super().__init__('map_to_base_link_listener')
        
        # setup the frames
        namespace = self.get_namespace().lstrip('/')
        self.base_frame_ = f'{namespace}/' + first_frame
        self.target_frame_ = f'{namespace}/' + second_frame
        
        # setup the publisher
        self.publisher_ = self.create_publisher(TransformStamped, f'{self.base_frame_}_to_{self.target_frame_}_transform', 10)

        # setup the tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        try:
            # Lookup the transform
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame_,
                self.target_frame_,
                rclpy.time.Time()  # Time (0 means latest available)
            )
            # Publish the transform
            self.publisher_.publish(transform)
            self.print_transform(transform)

        except TransformException as ex:
            self.get_logger().info(f"Could not get transform: {ex}")

    def print_transform(self, transform: TransformStamped):
        self.get_logger().info(f"Publishing tf: "
                    f"[x: {transform.transform.translation.x:.3f}, "
                    f"y: {transform.transform.translation.y:.3f}, "
                    f"z: {transform.transform.translation.z:.3f}]"
                    f", [x: {transform.transform.rotation.x:.3f}, "
                    f"y: {transform.transform.rotation.y:.3f}, "
                    f"z: {transform.transform.rotation.z:.3f}, "
                    f"w: {transform.transform.rotation.w:.3f}]")

def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkToMapListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()