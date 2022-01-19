from turtle import right
import rclpy
from rclpy.node import Node

from walker_msgs.msg import ForceStamped 
from std_msgs.msg import Float64MultiArray
from leg_detector_msgs.msg import StepArray
from std_msgs.msg import String
from numpy import interp
from squaternion import Quaternion
import matplotlib.pyplot as plt

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
        self.force_scale_factor = 100.0

        # Here is the Marker to be published in RViz
        self.colormap = plt.get_cmap('jet')

        # Constants from callibration
        self.weight_points = [0, 1, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20] # measurement points in kg
        self.left_handle_points = [2.7, 46.7, 81.8, 138.5, 193.1, 255.01, 334.6, 430.9, 474.1, 551, 612.1, 669.7] # handle
        self.right_handle_points = [0.025, 42.13, 68.2, 139.2, 223.8, 290.5, 346.5, 447.2, 481.7, 559.5, 640.1, 662.8] # handle

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
        self.left_handle_sub = self.create_subscription(ForceStamped, self.left_handle_topic_name, self.handle_lc, 10) 
        self.right_handle_sub = self.create_subscription(ForceStamped, self.right_handle_topic_name, self.handle_lc, 10) 
        self.loads_sub = self.create_subscription(StepArray, self.loads_topic_name, self.loads_lc, 10) 
        self.user_desc_sub = self.create_subscription(String, self.user_desc_topic_name, self.user_desc_lc, 10) 
        self.tmr = self.create_timer(self.period, self.timer_callback)
        self.marker_pub_ = self.create_publisher(Marker, self.markers_topic_name, 10)

        self.get_logger().info("load detector started")  

    def handle_lc(self, msg):
        if ('right' in msg.header.frame_id ):
            self.right_handle_msg = msg          
            self.right_handle_weight = interp(msg.force, self.right_handle_points, self.weight_points, left=0)
        elif ('left' in msg.header.frame_id):
            self.left_handle_msg = msg
            self.left_handle_weight  = interp(msg.force, self.left_handle_points, self.weight_points, left=0) 
        else:
            self.get_logger().error("Don't know which handle are you talking [" + msg.header.frame_id + "]")    
            return                
        self.leg_load = self.weight - self.left_handle_weight - self.right_handle_weight

    def user_desc_lc(self, msg):
        user_fields = msg.data.split(':')
        if len(user_fields)>2:
            self.weight = user_fields[2]

    def loads_lc(self, msg):
            if len(msg.steps) == 2:
                self.loads_msg = msg                            
            else:
                self.get_logger().error("Leg data incomplete [" + str(len(msg.steps)) + "]")    
    
    def timer_callback(self):
        if ( (self.left_handle_msg is not None) and  (self.right_handle_msg is not None) and  (self.loads_msg is not None) ):
            # We plot here an shere where we have a load

            # When we have an unknown weight distribution, load is set to -1 
            if (self.loads_msg.steps[0].load>=0) and (self.loads_msg.steps[1].load>=0):
                marker_left  = self.fill_in_marker(msg, 0)
                marker_left_text  = self.fill_in_text_marker(msg, 0)
                marker_right = self.fill_in_marker(msg, 1)
                marker_right_text = self.fill_in_text_marker(msg, 1)

                self.marker_pub_.publish(marker_left)
                self.marker_pub_.publish(marker_left_text)
                self.marker_pub_.publish(marker_right)
                self.marker_pub_.publish(marker_right_text)

                # TODO: Plot weight centroid using handle forces ...
            else:
                self.get_logger().warn("Unknown weight distribution on legs L(" + str(self.loads_msg.steps[0].load) + ") - R(" + str(self.loads_msg.steps[1].load) + ")")
        else:
            self.get_logger().error("Not all data received yet ...") 
    
    def fill_in_marker(self, stepArrayMsg, index):

        marker = self.marker_ref        
        # I don't need this...
        # quat = Quaternion.from_euler(0.0, pi/2.0, 0.0)
        # marker.pose.orientation.w = quat[0]
        # marker.pose.orientation.x = quat[1]
        # marker.pose.orientation.y = quat[2]
        # marker.pose.orientation.z = quat[3]

        # set id
        marke.id = index
        # Set position
        marker.header.frame_id = stepArrayMsg.header.frame_id        
        marker.pose.position = stepArrayMsg.steps[index].leg.position        
        marker.pose.position.z = 1

        # Set colors
        col_value = float(stepArrayMsg.steps[index].force)/ float(3.0*self.force_scale_factor)
        col_value = max( min(col_value,1.0), 0.0 )        
        color = self.colormap(col_value)

        marker.color.a = stepArrayMsg.steps[index].leg.confidence
        marker.color.r, marker.color.g, marker.color.b, dummy = color
        return marker

    def fill_in_text_marker(self, stepArrayMsg, index):

        marker = self.text_marker_ref

        # set id
        marke.id = index
        
        # Set position
        marker.header.frame_id = stepArrayMsg.header.frame_id        
        marker.pose.position = stepArrayMsg.steps[index].leg.position        
        marker.pose.position.z = 1

        marker.color.a = 1
        #marker.color.r, marker.color.g, marker.color.b, dummy = (1,1,1,1)

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