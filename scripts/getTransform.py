import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped

class BaseLinkToMapListener(Node):

    def __init__(self):
        super().__init__('base_link_to_map_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call the timer to periodically check the transform
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            # Lookup the transform from map to base_link
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map',          # Target frame
                'base_link',    # Source frame
                rclpy.time.Time()  # Time (0 means latest available)
            )
            self.print_transform(transform)
            self.timer.cancel()
            rclpy.shutdown()
        except TransformException as ex:
            self.get_logger().info(f"Could not get transform: {ex}")

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
    rclpy.spin(node)

if __name__ == '__main__':
    main()