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
        self.first_name_ = first_frame
        self.second_name_ = second_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.cmd_ = Twist ()
        # self.publisher_ = self.create_publisher(Twist, "{}/cmd_vel".format(self.second_name_),10)
        
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            trans = self._tf_buffer.lookup_transform(self.second_name_, self.first_name_, rclpy.time.Time())
            self.cmd_.linear.x = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
            self.cmd_.angular.z = 4 * math.atan2(trans.transform.translation.y , trans.transform.translation.x)
            self.publisher_.publish(self.cmd_)

        except LookupException as e:
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))

        # try:
        #     # Lookup the transform from map to base_link
        #     transform: TransformStamped = self.tf_buffer.lookup_transform(
        #         '/luggage_av/map',
        #         '/luggage_av/base_link',
        #         rclpy.time.Time()  # Time (0 means latest available)
        #     )
        #     self.print_transform(transform)
        #     self.timer.cancel()
        #     rclpy.shutdown()
        # except TransformException as ex:
        #     self.get_logger().info(f"Could not get transform: {ex}")

    def print_transform(self, transform: TransformStamped):
        self.get_logger().info(f"Transform: {transform}")
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
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()