import numpy as np
import yaml
import os
from operator import index
from scipy import interpolate

import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from walker_msgs.msg import ForceStamped 


'''
Hence, we can detect the user's steps by simply searching for inflection points in the function resulting from difference between forces:
f_diff = F_right_Z - F_left_Z

gait parameters from f_diff and time
• Step time (SpT): Average time between maximum-minimum (Right) or minimum-maximum (Left) in seconds. 
• Stride time (SdT): Average time between maximum-maximum (Right) and minimum-minimum (Left) in seconds.
• Number of Step (NoS): Numbers of inflection points.

gait parameters from f_diff and odometry
• Step length (SpL): Average length between maximum-minimum (Right) or minimum-maximum (Left) in meters.
• Stride length (SdL): Average length between maximum-maximum (Right) and minimum-minimum (Left) in meters.

gait parameters from test:
• Time required (Tr): Number of seconds that the user takes to complete the test.
• Distance(d): Distance walked by user in meters.
• Cadence (CAD): 60 * NoS/Tr .
• Average walking velocity (WV): d/Tr.


User's supports (UrS) can be estimated as the sum of forces 
F_left_Z + F_right_Z
W_B: weight-bearing the inverse of UrS.

'''

class GaitMonitorHand(Node):

    def __init__(self):
        super().__init__('gait_monitor_forces')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('handle_calibration_file', os.path.join(get_package_share_directory('walker_loads'), "config", "params.yaml") ),
                ('left_handle_topic_name', '/left_handle'),
                ('right_handle_topic_name', '/right_handle'),
                ('user_desc_topic_name', '/user_desc'),
                ('odom_topic_name', '/odom'),
                ('global_frame_id', '/odom'),
                ('period', 2.0),
            ])


        self.left_handle_topic_name = self.get_parameter('left_handle_topic_name').value
        self.right_handle_topic_name= self.get_parameter('right_handle_topic_name').value
        self.user_desc_topic_name= self.get_parameter('user_desc_topic_name').value
        self.odom_topic_name = self.get_parameter('odom_topic_name').value
        self.period = self.get_parameter('period').value
        self.global_frame_id = self.get_parameter('global_frame_id').value
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
        self.right_handle_weights = []
        self.left_handle_weights = []
        self.handle_weight_times = []
        self.handle_weight_dists = []


        self.prev_position = None
        self.cur_position = None
        self.travelled = 0
        
        # ROS stuff
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.left_handle_sub = self.create_subscription(ForceStamped, self.left_handle_topic_name, self.handle_lc, 10) 
        self.right_handle_sub = self.create_subscription(ForceStamped, self.right_handle_topic_name, self.handle_lc, 10) 
        self.sub = self.create_subscription(Odometry, self.odom_topic_name, self.odom_callback, 10)

        # mimics a ROS1 'latched' topic
        latched_profile = QoSProfile(depth=1)
        latched_profile.history = QoSHistoryPolicy.KEEP_LAST
        latched_profile.reliability = QoSReliabilityPolicy.RELIABLE
        latched_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.user_desc_sub = self.create_subscription(String, self.user_desc_topic_name, self.user_desc_lc, latched_profile) 
        self.tmr = self.create_timer(self.period, self.timer_callback)

        self.get_logger().info("Force based Gait monitor started")  

    def user_desc_lc(self, msg):
        user_fields = msg.data.split(':')
        if len(user_fields)>2:
            self.weight = user_fields[2]

    def handle_lc(self, msg):
        if ('right' in msg.header.frame_id ):
            self.right_handle_weights.append(self.fr(msg.force))
            self.handle_weight_times.append(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
            self.handle_weight_dists.append(self.travelled)
            # later we will combine both forces, so better to have same number of points in time/space
            if (len(self.left_handle_weights)>0):
                self.left_handle_weights.append(self.left_handle_weights[-1])

        elif ('left' in msg.header.frame_id):
            self.left_handle_weights.append(self.fl(msg.force)) 
            self.handle_weight_times.append(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
            self.handle_weight_dists.append(self.travelled)            
            # later we will combine both forces, so better to have same number of points in time/space
            if (len(self.right_handle_weights)>0):
                self.right_handle_weights.append(self.right_handle_weights[-1])


        else:
            self.get_logger().error("Don't know about which handle are you talking [" + msg.header.frame_id + "]")    
            return                
        self.leg_load = self.weight - self.left_handle_weight - self.right_handle_weight

    def odom_callback(self, msg):
        self.prev_position = self.cur_position
        self.cur_position = msg.pose.pose.position

        if (self.prev_position!= None):
            d = np.sqrt(np.power(self.cur_position.x - self.prev_position.x, 2.0 ) + np.power(self.cur_position.y - self.prev_position.y, 2.0 ) )            
            self.travelled += d




    def timer_callback(self):
        if ( (len(self.right_handle_weights)>0) and  (len(self.left_handle_weights)>0) ):
            [l_SpT, r_SpT, l_SdT, r_SdT, NoS] = self.get_gait_times()            
            [l_SpL, r_SpL, l_SdL, r_SdL, NoS] = self.get_gait_lenghts()
            
            start_stamp = self.handle_weight_times[0]
            end_stamp = self.handle_weight_times[-1]
            Tr = end_stamp - start_stamp  
               
            d = self.travelled 
            CAD =  60.0 * NoS/Tr
            WV = d/Tr

            self.get_logger().info("Current gait parameters:") 
            self.get_logger().info("\t  [" + str( ) + "]")             

            self.get_logger().info("\t  Left leg:")            
            self.get_logger().info("\t\t Step time (SpT): ["     + str(l_SpT ) + "]")             
            self.get_logger().info("\t\t Stride time (SdT):  ["  + str(l_SdT) + "]")             
            self.get_logger().info("\t\t Step length (SpL): ["   + str(l_SpL) + "]")             
            self.get_logger().info("\t\t Stride length (SdL): [" + str(l_SdL ) + "]")             

            self.get_logger().info("\t  Right leg:")            
            self.get_logger().info("\t\t Step time (SpT): ["     + str(r_SpT ) + "]")             
            self.get_logger().info("\t\t Stride time (SdT):  ["  + str(r_SdT) + "]")             
            self.get_logger().info("\t\t Step length (SpL): ["   + str(r_SpL) + "]")             
            self.get_logger().info("\t\t Stride length (SdL): [" + str(r_SdL ) + "]")   

            self.get_logger().info("\t Time required (Tr): [" + str(Tr ) + "]")  
            self.get_logger().info("\t Number of Step (NoS):  [" + str(NoS ) + "]")             
            self.get_logger().info("\t Distance (d): [" + str(d) + "]")  
            self.get_logger().info("\t Cadence (CAD): [" + str(CAD) + "]")  
            self.get_logger().info("\t Average walking velocity (WV): [" + str(WV) + "]")  

    def get_gait_times(self):
        '''
        f_diff = F_right_Z - F_left_Z
        • Step time (SpT): Average time between maximum-minimum (Right) or minimum-maximum (Left) in seconds. 
        • Stride time (SdT): Average time between maximum-maximum (Right) and minimum-minimum (Left) in seconds.
        '''
        [lSpT, rSpT, lSdT, rSdT, NoS] = self.get_params_times(self.handle_weight_times)
        return [lSpT, rSpT, lSdT, rSdT, NoS]
        
    def get_params_times(self, x):
        # substract forces:
        f_diff = np.array(self.right_handle_weights) - np.array(self.left_handle_weights)

        # find force mins and maxs
        [curr_min_v, curr_max_v] = self.find_min_max(f_diff) 
        
        # vectors with relevant magnitude at those critital points
        maxs = x[curr_max_v]
        mins = x[curr_min_v]

        NoS = (len(curr_max_v) + len(curr_min_v))/2

        # SdX magnitude between minimum-minimum (Left)
        x_bet_mins = np.diff(np.array(mins))
        lSdX = np.nanmean(x_bet_mins)

        # SdX magnitude between maximum-maximum (Right)
        x_bet_maxs = np.diff(np.array(maxs))
        rSdX = np.nanmean(x_bet_maxs)

        # SpX magnitude between minimum-maximum (Left)
        x_min_max = self.distance_between(maxs, mins)
        lSpX = np.nanmean(x_min_max)

        # SpX magnitude between maximum-minimum (Right)
        x_max_min = self.distance_between(mins, maxs)
        rSpX = np.nanmean(x_max_min)

        return [lSpX, rSpX, lSdX, rSdX, NoS]


    def get_gait_lenghts(self):
        '''
        f_diff = F_right_Z - F_left_Z
        • Step time (SpT): Average time between maximum-minimum (Right) or minimum-maximum (Left) in seconds. 
        • Stride time (SdT): Average time between maximum-maximum (Right) and minimum-minimum (Left) in seconds.
        '''
        [lSpL, lSdL, rSpL, rSdL] = self.get_params_times(self.handle_weight_dists)
        return [lSpL, lSdL, rSpL, rSdL]


    def find_min_max(self,data):        
        curr_min_i = 0
        curr_max_i = 0
        min_v = []
        max_v = []        
        got_data_min = got_data_max = False
        for i in range(1,len(data)):
            fi = data[i]
            if fi>0:
                if got_data_min:
                    #print('Storing minimum ' + str(data[curr_min_i]) )
                    min_v.append(curr_min_i)
                    got_data_min = False
                # searching positive
                if (fi > data[curr_max_i]) or (not got_data_max):
                    #print('fi: ' + str(fi) + ' can be a max'  )
                    curr_max_i = i
                    got_data_max = True
            elif fi<0:
                if got_data_max:
                    #print('Storing max ' + str(data[curr_max_i]))
                    max_v.append(curr_max_i)
                    got_data_max = False
                # searching positive
                if (fi < data[curr_min_i]) or (not got_data_min):
                    #print('fi: ' + str(fi) + ' can be a min'  )
                    curr_min_i = i
                    got_data_min = True
        # loop ended, may have candidates to add
        if got_data_max:
            #print('Storing max ' + str(data[curr_max_i]))
            max_v.append(curr_max_i)
        if got_data_min:
            #print('Storing minimum ' + str(data[curr_min_i]) )
            min_v.append(curr_min_i)

        return [min_v, max_v]

    def distance_between(self, end_vec0, start_vec0):
    
        end_vec = np.array(end_vec0)
        start_vec = np.array(start_vec0)

        # difference between vectors should be positive, 
        # hence first value on end_vec should be bigger
        if end_vec[0]<start_vec[0]:
            start_vec = start_vec[1:]

        lend = len(end_vec)
        lstart = len(start_vec)
        # remove tailing end_vec point
        if lend > lstart:
            end_vec = end_vec[0:lstart]
        # remove tailing start_vec point
        if lstart > lend:
            start_vec = start_vec[0:lend]

        # now we can operate
        diff_v = end_vec - start_vec

        return diff_v

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = GaitMonitorHand()

    rclpy.spin(minimal_subscriber)
 
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()