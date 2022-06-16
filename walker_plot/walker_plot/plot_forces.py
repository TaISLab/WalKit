#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from walker_msgs.msg import ForceStamped
from visualization_msgs.msg import Marker
from squaternion import Quaternion
from math import pi
import matplotlib.pyplot as plt

class ForcePlotter(Node):

    def __init__(self):
        super().__init__('handle_plotter')

        self.left_handle_topic_name = '/left_handle'
        self.right_handle_topic_name= '/right_handle'
        self.markers_topic_name = '/markers'
        self.markers_namespace = 'forces'
        self.markers_lifetime_s = 1.
        self.force_scale_factor = 100.0


        # Here is the Marker to be published in RViz
        self.colormap = plt.get_cmap('jet')

        self.arrow_base = Marker()
        self.arrow_base.type = Marker.ARROW
        self.arrow_base.ns = self.markers_namespace 
        self.arrow_base.lifetime = rclpy.duration.Duration(seconds=self.markers_lifetime_s).to_msg()

        # Position/Orientation mode
        # arrow length
        self.arrow_base.scale.x = 3e-2
        # arrow width
        self.arrow_base.scale.y = 3e-2
        # arrow height
        self.arrow_base.scale.z = 3e-2
        
        self.arrow_base.color.a = 1. 
        self.arrow_base.color.r, self.arrow_base.color.g, self.arrow_base.color.b, dummy = self.colormap(0)

        # arrows will point downwards
        quat = Quaternion.from_euler(0.0, pi/2.0, 0.0)
        self.arrow_base.pose.orientation.w = quat[0]
        self.arrow_base.pose.orientation.x = quat[1]
        self.arrow_base.pose.orientation.y = quat[2]
        self.arrow_base.pose.orientation.z = quat[3]

        # ROS stuff
        self.left_handle_sub = self.create_subscription( ForceStamped, self.left_handle_topic_name, self.listener_callback, 10) 
        self.left_handle_pub = self.create_publisher(Marker, self.markers_topic_name, 10)

        self.right_handle_sub = self.create_subscription( ForceStamped, self.right_handle_topic_name, self.listener_callback, 10) 
        self.right_handle_pub = self.create_publisher(Marker, self.markers_topic_name, 10)
        self.get_logger().info("Force plotter started")  

    def listener_callback(self, msg):
        hand_marker = self.arrow_base
        len_value = float(msg.force)/ float(10.0*self.force_scale_factor)
        len_value = max( min(len_value,1.0), 0.001 )

        col_value = float(msg.force)/ float(3.0*self.force_scale_factor)
        col_value = max( min(col_value,1.0), 0.001 )        
        color = self.colormap(col_value)

        #print("raw:[" + str(msg.force) + "], \t" + 
        #      "len:[" + str(len_value) + "], \t" + 
        #      "col:[" + str(col_value) + "]" )

        hand_marker.header.frame_id = msg.header.frame_id        
        hand_marker.scale.x = len_value
        hand_marker.color.r, hand_marker.color.g, hand_marker.color.b, dummy = color


        if ('right' in msg.header.frame_id ):
            self.arrow_base.id = 0
            self.right_handle_pub.publish(hand_marker)
        elif ('left' in msg.header.frame_id):
            self.arrow_base.id = 1
            self.left_handle_pub.publish(hand_marker)
        else:
            self.get_logger().error("Don't know where to publish [" + msg.header.frame_id + "]")    
                
    

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ForcePlotter()

    rclpy.spin(minimal_subscriber)
 
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()