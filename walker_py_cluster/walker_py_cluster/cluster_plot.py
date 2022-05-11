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

        self.laser_topic_name= '/scan_filtered'
        self.markers_topic_name = '/markers'
        self.markers_namespace = 'clusters'
        self.markers_lifetime_s = 1.

        self.angles = None
        self.x_min = -0.75
        self.y_min = -0.4
        self.x_max = 0.4
        self.y_max = 0.4

        # # Here is the Marker to be published in RViz
        # self.colormap = plt.get_cmap('jet')

        # self.marker_base = Marker()
        # self.marker_base.type = Marker.ARROW
        # self.marker_base.ns = self.markers_namespace 
        # self.marker_base.lifetime = rclpy.duration.Duration(seconds=self.markers_lifetime_s).to_msg()

        # # Position/Orientation mode
        # # arrow length
        # self.marker_base.scale.x = 3e-2
        # # arrow width
        # self.marker_base.scale.y = 3e-2
        # # arrow height
        # self.marker_base.scale.z = 3e-2
        
        # self.marker_base.color.a = 1. 
        # self.marker_base.color.r, self.marker_base.color.g, self.marker_base.color.b, dummy = self.colormap(0)

        # # arrows will point downwards
        # quat = Quaternion.from_euler(0.0, pi/2.0, 0.0)
        # self.marker_base.pose.orientation.w = quat[0]
        # self.marker_base.pose.orientation.x = quat[1]
        # self.marker_base.pose.orientation.y = quat[2]
        # self.marker_base.pose.orientation.z = quat[3]

        # ROS stuff
        self.laser_sub = self.create_subscription( LaserScan, self.laser_topic_name, self.listener_callback, qos_profile_sensor_data)      
        self.marker_pub = self.create_publisher(Marker, self.markers_topic_name, 10)

        self.get_logger().info("Cluster plotter started")  

    def cast_to_cartesians(self, msg):

        # do once, save time ...
        if (self.angles ==None):
            self.angles = np.arange(msg.angle_min, msg.angle_max+msg.angle_increment, msg.angle_increment)
            self.cos_i = np.cos(self.angles)
            self.sin_i = np.sin(self.angles)
        
        r = np.array(msg.ranges)
        valid_indexs = np.logical_and(r>msg.range_min, r<msg.range_max) 
                
        x = r * self.cos_i
        y = r * self.sin_i
        new_x = x[valid_indexs].copy()
        new_y = y[valid_indexs].copy()
        return (new_x, new_y)

    def remove_outliers(self x0,y0):
        valid_indexs_x = np.logical_and(x0>self.x_min, x0<self.x_max) 
        valid_indexs_y = np.logical_and(y0>self.y_min, y0<self.y_max) 

        x = x0[valid_indexs_x].copy()
        y = y0[valid_indexs_y].copy()

        return (x, y)

    def listener_callback(self, msg):
        (x,y) = self.cast_to_cartesians(msg)
        (x,y) = self.remove_outliers(x,y)
        data = self.build_dataset(x,y)

        if (self.prev_data == None):
            self.prev_data = data
        else:
            self.match_datasets(data)

    def build_dataset(self,x,y):


    def match_datasets(self, new_dataset):
        



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ClusterPlot()

    rclpy.spin(minimal_subscriber)
 
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
