import rclpy
from rclpy.node import Node

from walker_msgs.msg import ForceStamped, StepStamped 
from std_msgs.msg import Float64MultiArray, String
from scipy import interpolate
import yaml
from ament_index_python.packages import get_package_share_directory
import os


class PartialLoads(Node):

    def __init__(self):
        super().__init__('partial_loads')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('handle_calibration_file', os.path.join(get_package_share_directory('walker_loads'), "config", "params.yaml") ),
                ('left_loads_topic_name', '/left_loads'),
                ('right_loads_topic_name', '/right_loads'),
                ('left_handle_topic_name', '/left_handle'),
                ('right_handle_topic_name', '/right_handle'),
                ('left_steps_topic_name', '/detected_step_left'),
                ('right_steps_topic_name', '/detected_step_right'),
                ('user_desc_topic_name', '/user_desc'),
                ('period', 0.1),
                ('speed_delta', 0.05),
                ('force_calibration.weight_points', None),
                ('force_calibration.left_handle_points', None),
                ('force_calibration.right_handle_points', None)
            ])


        self.left_loads_topic_name = self.get_parameter('left_loads_topic_name').value
        self.right_loads_topic_name = self.get_parameter('right_loads_topic_name').value
        self.left_handle_topic_name = self.get_parameter('left_handle_topic_name').value
        self.right_handle_topic_name= self.get_parameter('right_handle_topic_name').value
        self.left_steps_topic_name= self.get_parameter('left_steps_topic_name').value
        self.right_steps_topic_name= self.get_parameter('right_steps_topic_name').value
        self.user_desc_topic_name= self.get_parameter('user_desc_topic_name').value
        self.period = self.get_parameter('period').value
        self.speed_delta = self.get_parameter('speed_delta').value
        self.handle_calibration_file = self.get_parameter('handle_calibration_file').value

        with open(self.handle_calibration_file, 'r') as stream:
            try:
                parsed_yaml=yaml.safe_load(stream)
                self.weight_points = parsed_yaml['weight_points']        
                self.left_handle_points = parsed_yaml['left_handle_points']
                self.right_handle_points = parsed_yaml['right_handle_points']
            except yaml.YAMLError as exc:
                print(exc)
        
        self.fr = interpolate.interp1d(self.right_handle_points, self.weight_points, fill_value = "extrapolate")
        self.fl = interpolate.interp1d(self.left_handle_points,  self.weight_points, fill_value = "extrapolate")

        # stored data
        self.left_handle_msg = None
        self.right_handle_msg = None
        self.left_step_msg = None
        self.right_step_msg = None

        self.speed_diff = 0
        self.right_speed = None
        self.left_speed  = None

        self.weight = 100
        self.right_handle_weight = 0
        self.left_handle_weight  = 0
        self.leg_load = 0

        # ROS stuff
        self.left_load_pub  = self.create_publisher(StepStamped, self.left_loads_topic_name,  10)
        self.right_load_pub = self.create_publisher(StepStamped, self.right_loads_topic_name, 10)

        self.left_handle_sub = self.create_subscription(ForceStamped, self.left_handle_topic_name, self.handle_lc, 10) 
        self.right_handle_sub = self.create_subscription(ForceStamped, self.right_handle_topic_name, self.handle_lc, 10) 

        self.left_steps_sub = self.create_subscription(StepStamped, self.left_steps_topic_name,  self.l_steps_lc, 10) 
        self.right_steps_sub = self.create_subscription(StepStamped, self.right_steps_topic_name, self.r_steps_lc, 10)

        self.user_desc_sub = self.create_subscription(String, self.user_desc_topic_name, self.user_desc_lc, 10) 
        self.tmr = self.create_timer(self.period, self.timer_callback)


        self.get_logger().info("load detector started")  

    def user_desc_lc(self, msg):
        user_fields = msg.data.split(':')
        if len(user_fields)>2:
            self.weight = user_fields[2]

    def l_steps_lc(self, msg):
        self.steps_lc(msg,0)

    def r_steps_lc(self, msg):
        self.steps_lc(msg,1)

    def handle_lc(self, msg):
        if ('right' in msg.header.frame_id ):
            self.right_handle_msg = msg          
            self.right_handle_weight = self.fr(msg.force)
        elif ('left' in msg.header.frame_id):
            self.left_handle_msg = msg
            self.left_handle_weight  = self.fl(msg.force) 
        else:
            self.get_logger().error("Don't know about which handle are you talking [" + msg.header.frame_id + "]")    
            return                
        self.leg_load = self.weight - self.left_handle_weight - self.right_handle_weight


    def steps_lc(self, msg, id):
        if  (id==1):
                self.right_step_msg = msg
                self.right_speed = self.right_step_msg.speed
        elif (id==0):
                self.left_step_msg = msg
                self.left_speed = self.left_step_msg.speed
        else:
            self.get_logger().error("Don't know about which step are you talking [" + id + "]")    
            return          
        
        if ((self.left_step_msg is not None) and  (self.right_step_msg is not None) ): 
            speed_diff_x = self.left_speed.x - self.right_speed.x
            speed_diff_y = self.left_speed.y - self.right_speed.y
            speed_diff_z = self.left_speed.z - self.right_speed.z
            #self.speed_diff = pow(speed_diff_x * speed_diff_x + speed_diff_y * speed_diff_y + speed_diff_z * speed_diff_z , 0.5)
            self.speed_diff = speed_diff_x

    def timer_callback(self):
        if ( (self.left_handle_msg is not None) and  (self.right_handle_msg is not None) and (self.left_step_msg is not None) and  (self.right_step_msg is not None) ):
                if (self.speed_diff>self.speed_delta):
                    self.right_leg_load = self.leg_load
                    self.left_leg_load = 0.0
                elif (self.speed_diff<-self.speed_delta):
                    self.right_leg_load = 0.0
                    self.left_leg_load = self.leg_load
                else:
                    # both leg standing. We can't be sure about how much on each one!
                    self.right_leg_load = -1.0 #0.5 * self.leg_load
                    self.left_leg_load =  -1.0 #0.5 * self.leg_load
                   
                # Build msg and publish
                # left
                msg = self.left_step_msg                
                msg.load = self.left_leg_load
                self.left_load_pub.publish(msg)

                #right
                msg = self.right_step_msg                
                msg.load = self.right_leg_load
                self.right_load_pub.publish(msg)
                #self.get_logger().warn("Weight distribution on legs L(" + str(self.left_leg_load) + ") - R(" + str(self.right_leg_load) + ")")

        else:
            pass
            #self.get_logger().error("Not all data received yet ...")                

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = PartialLoads()

    rclpy.spin(minimal_subscriber)
 
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()