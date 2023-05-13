#!/usr/bin/python3

import pandas as pd
import math
import numpy as np
from os.path import join
from sys import stderr

import rclpy
import tf2_geometry_msgs
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped
from walker_msgs.msg import StabilityStamped

class DummyUser(Node):

    def __init__(self):
        super().__init__('dummy_user')
    
        self.declare_parameters(
            namespace='',
            parameters=[
                 ('stability_topic_name', '/user_stability'),
                 ('user_id', 'user3'),
                 ('user_tinetti', 0.4)
            ]
        )

        self.stability_topic_name = self.get_parameter('stability_topic_name').value
        self.user_id              = self.get_parameter('user_id').value
        self.user_tinetti         = self.get_parameter('user_tinetti').value

        # ROS stuff
        self.stability_pub  = self.create_publisher(StabilityStamped, self.stability_topic_name,  10)
                
        self.get_logger().info("dummy user main loop started")  

        # TODO crap here
        # usuario 1 tin 1 sta 1
        #x_pose = [-1.66, -1.89, -3.22, -2.53,  1.56,  3.83,  2.41,  2.22, -0.77,  0.02, -2.51, -3.38, -1.09 ]
        #y_pose = [ 3.19,  5.28,  5.89, 15.08, 14.76, 13.59, 10.78,  7.82,  8.32, 14.99, 15.18,  5.87,  3.44 ]
        # usuario 2 tin 0.6  sta 0.2
        #x_pose = [-1.66, -1.89, -3.22, -3.97, -2.53,  1.56,  3.83,  2.41,  2.22, -0.77,  0.02, -2.51, -3.38, -1.09 ]
        #y_pose = [ 3.19,  5.28,  5.89,  7.32, 15.08, 14.76, 13.59, 10.78,  7.82,  8.32, 14.99, 15.18,  5.87,  3.44 ]
        # usuario 3 tin 0.4  sta 0.4
        x_pose = [ 2.01, -1.66, -1.89, -3.22, -3.97, -2.53,  1.56,  3.83,  2.41,  2.22, -0.77,  0.02, -2.51, -3.38, -1.09 ]
        y_pose = [ 3.06,  3.19,  5.28,  5.89,  7.32, 15.08, 14.76, 13.59, 10.78,  7.82,  8.32, 14.99, 15.18,  5.87,  3.44 ]
        # usuario 4 tin 0.8  sta 0.6
        x_pose = [ 2.01, -1.66, -1.89, -3.22, -3.97, -2.53,  1.56,  3.83,  2.41,  2.22, -0.77,  0.02, -2.51, -3.38, -1.09 ]
        y_pose = [ 3.06,  3.19,  5.28,  5.89,  7.32, 15.08, 14.76, 13.59, 10.78,  7.82,  8.32, 14.99, 15.18,  5.87,  3.44 ]
        s_pose = [ 0.56,  0.59,  0.58,  0.89,  0.32,  0.58,  0.76,  0.59,  0.78,  0.82,  0.32,  0.99,  0.01,  0.07,  0.24 ]

        self.trans_speed = 0.85
        timer_period = 0.25
        
        self.x_pose = np.array(x_pose)
        self.y_pose = np.array(y_pose)
        self.s_pose = np.array(s_pose)

        x_inc = np.diff(self.x_pose)
        y_inc = np.diff(self.y_pose)
        self.x_sign = np.sign(x_inc)
        self.y_sign = np.sign(y_inc)

        d = np.cumsum(np.append([0], np.sqrt(x_inc*x_inc + y_inc*y_inc)))
        
        self.t_pose = d/self.trans_speed

        t_pose_upsampled = np.arange(0, self.t_pose[-1], timer_period, dtype=float)

        self.x_pose = np.interp(t_pose_upsampled, self.t_pose, self.x_pose)
        self.y_pose = np.interp(t_pose_upsampled, self.t_pose, self.y_pose)
        self.s_pose = np.interp(t_pose_upsampled, self.t_pose, self.s_pose)

        self.i = 0

        # Create a timer 
        self.timer = self.create_timer(timer_period, self.callback)

    def callback(self):
        self.sendStability(self.x_pose[self.i], self.y_pose[self.i], self.s_pose[self.i])

        self.get_logger().warn("We are at [" + str(round( self.x_pose[self.i],2)) +  ", " + str(round( self.y_pose[self.i],2)) +  " ].\n\n")   

        if self.i == (np.size(self.y_pose)-1):
            self.destroy_node()
        else:
            self.i =self.i+1


    def sendStability(self, x,y, sta):
        # Stability msg
        outMsg = StabilityStamped()

        # User ID
        outMsg.uid = self.user_id

        # User Tineti score
        outMsg.tin = self.user_tinetti

        # Stability value (0-1)
        outMsg.sta = sta

        # User pose for given stability (m)  
        outMsg.pose = PoseStamped()
        outMsg.pose.header.frame_id = 'map'
        outMsg.pose.header.stamp = (self.get_clock().now()).to_msg()

        outMsg.pose.pose.position.x = x
        outMsg.pose.pose.position.y = y
        outMsg.pose.pose.orientation.w = 1.0

        # and publish 
        self.stability_pub.publish(outMsg)
        #self.get_logger().warn("We are at [" + str(round(x ,2)) +  ", " + str(round(y ,2)) +  " ].\n\n")   



def main(args=None):
    rclpy.init(args=args)

    myNode = DummyUser()

    try:
        rclpy.spin(myNode)
    except KeyboardInterrupt:
        print('User-requested stop')
    except BaseException:
        print('Exception in execution:', file=stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        myNode.destroy_node()
        rclpy.shutdown()  


if __name__ == '__main__':
    main()