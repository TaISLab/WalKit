from turtle import right
import rclpy
from rclpy.node import Node

from walker_msgs.msg import ForceStamped 
from std_msgs.msg import Float64MultiArray
from leg_detector_msgs.msg import StepArray
from std_msgs.msg import String
from numpy import interp

class PartialLoads(Node):

    def __init__(self):
        super().__init__('partial_loads')
        self.loads_topic_name = '/loads'
        self.left_handle_topic_name = '/left_handle'
        self.right_handle_topic_name= '/right_handle'
        self.steps_topic_name= '/detected_steps'
        self.user_desc_topic_name= '/user_desc'
        self.period = 0.1
        self.speed_delta = 0.1

        self.weight_points = [0, 1, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20] # measurement points in kg
        self.left_handle_points = [2.7, 46.7, 81.8, 138.5, 193.1, 255.01, 334.6, 430.9, 474.1, 551, 612.1, 669.7] # handle
        self.right_handle_points = [0.025, 42.13, 68.2, 139.2, 223.8, 290.5, 346.5, 447.2, 481.7, 559.5, 640.1, 662.8] # handle

        # stored data
        self.left_handle_msg = None
        self.right_handle_msg = None
        self.steps_msg = None
        
        self.left_speed = 0
        self.right_speed = 0
        self.speed_diff = 0

        self.weight = 100
        self.right_handle_weight = 0
        self.left_handle_weight  = 0
        self.leg_load = 0
        self.right_leg_load = 0
        self.left_leg_load = 0

        # ROS stuff
        self.loads_pub = self.create_publisher(Float64MultiArray, self.loads_topic_name, 10)

        self.left_handle_sub = self.create_subscription(ForceStamped, self.left_handle_topic_name, self.handle_lc, 10) 
        self.right_handle_sub = self.create_subscription(ForceStamped, self.right_handle_topic_name, self.handle_lc, 10) 
        self.steps_sub = self.create_subscription(StepArray, self.steps_topic_name, self.steps_lc, 10) 
        self.user_desc_sub = self.create_subscription(String, self.user_desc_topic_name, self.user_desc_lc, 10) 
        self.tmr = self.create_timer(self.period, self.timer_callback)


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

    def steps_lc(self, msg):
            if len(msg.steps) == 2:
                self.steps_msg = msg                            
                self.left_speed = self.steps_msg.steps[0].speed
                self.right_speed = self.steps_msg.steps[1].speed
                self.speed_diff = self.left_speed - self.right_speed
            else:
                self.get_logger().error("Leg data incomplete [" + str(len(msg.steps)) + "]")    
    
    def timer_callback(self):
        if ( (self.left_handle_msg is not None) and  (self.right_handle_msg is not None) and  (self.steps_msg is not None) ):
                if (self.speed_diff>self.speed_delta):
                    self.right_leg_load = self.leg_load
                    self.left_leg_load = 0
                elif (self.speed_diff<-self.speed_delta):
                    self.right_leg_load = 0
                    self.left_leg_load = self.leg_load
                else:
                    self.right_leg_load = 0.5 * self.leg_load
                    self.left_leg_load = 0.5 * self.leg_load
                msg = Float64MultiArray()
                msg.data.append(self.left_leg_load)
                msg.data.append(self.right_leg_load)
                self.loads_pub.publish(msg)
        else:
            self.get_logger().error("Not all data received yet ...")                

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = PartialLoads()

    rclpy.spin(minimal_subscriber)
 
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()