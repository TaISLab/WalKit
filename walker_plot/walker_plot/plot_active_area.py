import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from squaternion import Quaternion
from math import pi
import matplotlib.pyplot as plt

class AreaPlotter(Node):

    def __init__(self):
        super().__init__('active_area_plotter')
        self.markers_topic_name = '/markers'
        self.markers_namespace = 'active_area'
        self.markers_lifetime_s = 1.
        self.colormap = plt.get_cmap('jet')

        self.marker_base = Marker()
        self.marker_base.header.frame_id = 'base_link'
        self.marker_base.type = Marker.LINE_STRIP
        self.marker_base.action = Marker.ADD
        self.marker_base.id = 0
        self.marker_base.ns = self.markers_namespace 
        self.marker_base.lifetime = rclpy.duration.Duration(seconds=self.markers_lifetime_s).to_msg()
        # LINE_STRIP markers use only the x component of scale, for the line width
        self.marker_base.scale.x = 0.01 
        #self.marker_base.scale.y = 1.5 
        #self.marker_base.scale.z = 1.5 

        self.marker_base.color.a = 1. 
        self.marker_base.color.r = 1.
        self.marker_base.color.g = 0.5
        self.marker_base.color.b = 0.5
        self.marker_base.pose.orientation.w = 1.

        # from laser_box_filter.yaml at walker_step_detector
        min_z = -0.4
        max_z = 0.4
        min_x = -0.75
        max_x = 0.4
        min_y = -0.4
        max_y = 0.4

        x_list = [min_x, min_x, max_x, max_x, min_x]
        y_list = [min_y, max_y, max_y, min_y, min_y]
        z = (max_z + min_z) / 2.0

        for i in range(len(x_list)):
            p = Point()
            p.x = x_list[i]
            p.y = y_list[i]
            p.z = z
            self.marker_base.points.append(p)

        # ROS stuff
        self.area_pub = self.create_publisher(Marker, self.markers_topic_name, 10)
        self.timer = self.create_timer(self.markers_lifetime_s, self.periodic_callback)

        self.get_logger().info("Active Area plotter started")  

    def periodic_callback(self):
        area_marker = self.marker_base
        area_marker.header.stamp = self.get_clock().now().to_msg()
        self.area_pub.publish(area_marker)      
    

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = AreaPlotter()

    rclpy.spin(minimal_subscriber)
 
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()