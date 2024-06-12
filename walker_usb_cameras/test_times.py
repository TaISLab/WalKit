import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.time import Time

class ImageTimestampSubscriber(Node):
    def __init__(self):
        super().__init__('image_timestamp_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )       

    def image_callback(self, msg):
        if hasattr(self,'last_time'):
            time_diff = Time.from_msg(msg.header.stamp) - self.last_time
            if time_diff.nanoseconds<0.0:
                self.get_logger().info(f" image ts dif: {time_diff.nanoseconds/1e6} ms")

        self.last_time = Time.from_msg(msg.header.stamp)

def main(args=None):
    rclpy.init(args=args)
    node = ImageTimestampSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()