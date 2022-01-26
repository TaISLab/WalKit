from turtle import right
import rclpy
from rclpy.node import Node

from walker_msgs.msg import ForceStamped 
from std_msgs.msg import Float64MultiArray
from walker_msgs.msg import StepStamped
from std_msgs.msg import String
from numpy import interp
from squaternion import Quaternion
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker
from scipy import interpolate

class LoadPlotter(Node):

    def __init__(self):
        super().__init__('load_plotter')
        self.loads_topic_name = '/loads'
        self.left_handle_topic_name = '/left_handle'
        self.right_handle_topic_name= '/right_handle'
        self.user_desc_topic_name= '/user_desc'
        self.period = 0.1
        self.markers_topic_name = '/markers'

        self.markers_namespace = 'steps_load'
        self.markers_lifetime_s = 1.
        self.force_scale_factor = 3.0

        # Here is the Marker to be published in RViz
        self.colormap = plt.get_cmap('jet')

        # base markers
        
        # Sphere
        self.marker_ref = Marker()
        self.marker_ref.type = Marker.SPHERE
        self.marker_ref.ns = self.markers_namespace 
        self.marker_ref.lifetime = rclpy.duration.Duration(seconds=self.markers_lifetime_s).to_msg()
        # diameter in x direction
        self.marker_ref.scale.x = 1e-1
        # diameter in y direction
        self.marker_ref.scale.y = 1e-1
        # diameter in z direction
        self.marker_ref.scale.z = 1e-1

        # Text
        self.text_marker_ref = Marker()
        self.text_marker_ref.type = Marker.TEXT_VIEW_FACING
        self.text_marker_ref.ns = self.markers_namespace + '_txt'
        self.text_marker_ref.lifetime = rclpy.duration.Duration(seconds=self.markers_lifetime_s).to_msg()

        # stored data
        self.left_handle_msg = None
        self.right_handle_msg = None
        self.loads_msg = None
        
        self.weight = 100
        self.right_handle_weight = 0
        self.left_handle_weight  = 0
        self.leg_load = 0

        # ROS stuff
        self.left_loads_sub  = self.create_subscription(StepStamped, self.loads_topic_name + "_left",  self.l_loads_lc, 10) 
        self.right_loads_sub = self.create_subscription(StepStamped, self.loads_topic_name + "_right", self.r_loads_lc, 10) 
        self.marker_pub_ = self.create_publisher(Marker, self.markers_topic_name, 10)

        self.get_logger().info("load plotter started")  

    def l_loads_lc(self, msg):
        self.loads_lc(msg,0)

    def r_loads_lc(self, msg):
        self.loads_lc(msg,1)

    def loads_lc(self, msg,id):
        if (id == 1):
                marker_right = self.fill_in_marker(msg, 1)
                self.marker_pub_.publish(marker_right)
                marker_right_text = self.fill_in_text_marker(msg, 1)
                self.marker_pub_.publish(marker_right_text)
        elif (id == 0):
                marker_left  = self.fill_in_marker(msg, 0)
                self.marker_pub_.publish(marker_left)
                marker_left_text  = self.fill_in_text_marker(msg, 0)
                self.marker_pub_.publish(marker_left_text)                
        else:
            self.get_logger().error("Don't know about which step are you talking [" + id + "]")    
            return            
    
    def fill_in_marker(self, stepStMsg, index):

        marker = self.marker_ref        
        # I don't need this...
        # quat = Quaternion.from_euler(0.0, pi/2.0, 0.0)
        # marker.pose.orientation.w = quat[0]
        # marker.pose.orientation.x = quat[1]
        # marker.pose.orientation.y = quat[2]
        # marker.pose.orientation.z = quat[3]

        if (index==0):
            marker.type = marker.SPHERE  # left
        else:
            marker.type = marker.CUBE

        # set id
        marker.id = index

        # Set position
        marker.header = stepStMsg.position.header
        marker.pose.position = stepStMsg.position.point
        marker.pose.position.z = 0.5

        # Set scale according to load
        marker.scale.x = marker.scale.y = marker.scale.z = max(0.01,stepStMsg.load/1000.0)
        #self.get_logger().error("plotting scale: " + str(marker.scale.x) )    

        # Set colors
        speed_mod = pow(stepStMsg.speed.x * stepStMsg.speed.x + stepStMsg.speed.y * stepStMsg.speed.y + stepStMsg.speed.z * stepStMsg.speed.z , 0.5)
        col_value = float(speed_mod) / float(3.0*self.force_scale_factor)
        col_value = max( min(col_value,1.0), 0.0 )        
        color = self.colormap(col_value)

        marker.color.a = stepStMsg.confidence
        marker.color.r, marker.color.g, marker.color.b, dummy = color




        return marker

    def fill_in_text_marker(self, stepStMsg, index):

        marker = self.text_marker_ref

        # set id
        marker.id = index

        # Set position
        marker.header = stepStMsg.position.header
        marker.pose.position = stepStMsg.position.point
        marker.pose.position.z = 0.5

        marker.color.a = 1.0
        #marker.color.r, marker.color.g, marker.color.b, dummy = (1,1,1,1)
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        #set text
        if (index==0):
            marker.text = 'left'
        else:
            marker.text = 'right'

        return marker

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = LoadPlotter()

    rclpy.spin(minimal_subscriber)
 
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()