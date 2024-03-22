import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker
from squaternion import Quaternion
from math import pi
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan 
from rclpy.qos import qos_profile_sensor_data


class ClusterPlot(Node):

    def __init__(self):
        super().__init__('cluster_plot')

        self.laser_topic_name= '/scan_feet'
        self.markers_topic_name = '/markers'
        self.markers_namespace = 'clusters'
        self.markers_lifetime_s = 1.

        # ROS stuff
        self.laser_sub = self.create_subscription( LaserScan, self.laser_topic_name, self.listener_callback, qos_profile_sensor_data)      
        self.marker_pub = self.create_publisher(Marker, self.markers_topic_name, 10)

        self.get_logger().info("Cluster plotter started")  

    def listener_callback(self, msg):
        self.ranges = msg.ranges
        self.get_logger().info(".")
        if not hasattr(self, 'range_counts'): 
            self.range_counts = np.zeros_like(self.ranges)

        for i in range(len(self.ranges)):
            if (self.ranges[i]< self.min_val):
                self.range_counts[i] = self.range_counts[i] + 1
        self.scans_read = self.scans_read +1


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ClusterPlot()

    rclpy.spin(minimal_subscriber)
 
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
