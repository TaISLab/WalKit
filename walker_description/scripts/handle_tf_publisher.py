import rclpy
from rclpy.node import Node
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class HandleTfPublisher(Node):

    def __init__(self):
        super().__init__('handle_tf_publisher')
        self.declare_parameters(
            namespace='',
            parameters=[
                 ('handle_calibration_file', os.path.join(get_package_share_directory('walker_description'), "config", "params.yaml") ),
                 ('frame_id', 'base_footprint' ),
                 ('child_frame_id_suffix', '_handle_id' ),
                 ('handle_height_topic_name', '/handle_height'),
                 ('period', 0.05),
            ]
        )

        self.handle_height_topic_name = self.get_parameter('handle_height_topic_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id_suffix = self.get_parameter('child_frame_id_suffix').value
        self.period = self.get_parameter('period').value
        self.handle_calibration_file = self.get_parameter('handle_calibration_file').value

        with open(self.handle_calibration_file, 'r') as stream:
            try:
                parsed_yaml=yaml.safe_load(stream)
                self.handle_x = parsed_yaml['handle_x']
                self.handle_y = parsed_yaml['handle_y']
                self.handle_z = parsed_yaml['handle_z']                
            except yaml.YAMLError as exc:
                print(exc)

        # stored data
        self.new_data_available = True # send a dummy first data to fill tf_tree
        self.handle_height = 2                
        # ROS stuff
        self.tf_publisher = StaticTransformBroadcaster(self)

        self.handle_height_sub = self.create_subscription(Int32, self.handle_height_topic_name, self.handle_height_cb, 10)

        self.tmr = self.create_timer(self.period, self.timer_callback)


        self.get_logger().info("handle tf publisher started")  

    def handle_height_cb(self, msg):        
        if len(self.handle_z)<msg.data:
            if (self.handle_height != msg.data):
                self.handle_height = msg.data
                self.new_data_available = True

    def get_tf(self,offset, frame_prefix,y_side):
        # build template tf
        handle_tf = TransformStamped()
        handle_tf.header.stamp = self.get_clock().now().to_msg()
        handle_tf.header.frame_id = self.frame_id
        handle_tf.child_frame_id = frame_prefix + self.child_frame_id_suffix
        handle_tf.transform.translation.x = self.handle_x[offset]
        handle_tf.transform.translation.y = y_side * self.handle_y[offset]
        handle_tf.transform.translation.z = self.handle_z[offset]
        handle_tf.transform.rotation.w = 1
        return handle_tf

    def timer_callback(self):
        if (self.new_data_available):
            self.new_data_available = False           
            # right handle 
            right_handle_tf = self.get_tf(self.handle_height, "right", 1)

            # left handle 
            left_handle_tf = self.get_tf(self.handle_height, "left", -1)
            self.tf_publisher.sendTransform(left_handle_tf)
            self.get_logger().info("New data sent!")

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = HandleTfPublisher()

    try:
      rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
      pass

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
