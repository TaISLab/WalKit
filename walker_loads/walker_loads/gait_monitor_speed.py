import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from walker_msgs.msg import StepStamped 
import numpy as np
from nav_msgs.msg import Odometry


'''
gait parameters from step load and time:
• Step time (SpT): Average time between consecutive step loads in alternate legs in seconds. 
• Stride time (SdT): Average time between consecutive step loads on the same leg in seconds. 
• Number of Step (NoS): Numbers of step loads of each leg.

gait parameters from step load and absolute step position:
• Step length (SpL): Average position difference (different legs) between consecutive step loads on alternate leg in meters.
• Stride length (SdL): Average position difference (same leg) between consecutive step loads on the same leg in meters.

gait parameters from test:
• Time required (Tr): Number of seconds that the user takes to complete the test.
• Distance(d): Distance walked by user in meters.
• Cadence (CAD): 60 * NoS/Tr .
• Average walking velocity (WV): d/Tr.

'''

class GaitMonitorSp(Node):

    def __init__(self):
        super().__init__('gait_monitor_sp')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('left_loads_topic_name', '/left_loads'),
                ('right_loads_topic_name', '/right_loads'),
                ('odom_topic_name', '/odom'),
                ('global_frame_id', '/odom'),
                ('period', 2.0),
            ])

        self.left_loads_topic_name = self.get_parameter('left_loads_topic_name').value
        self.right_loads_topic_name = self.get_parameter('right_loads_topic_name').value
        self.odom_topic_name = self.get_parameter('odom_topic_name').value
        self.period = self.get_parameter('period').value
        self.global_frame_id = self.get_parameter('global_frame_id').value
        
        # stored data
        self.is_left_load = False
        self.left_loads = []
        self.left_stride_times = []
        self.left_step_times = []
        self.left_stride_lenghts = []
        self.left_step_lenghts = []

        self.is_right_load = False
        self.right_loads = []
        self.right_stride_times = []
        self.right_step_times = []
        self.right_stride_lenghts = []
        self.right_step_lenghts = []

        self.prev_position = None
        self.cur_position = None
        self.travelled = 0

        # ROS stuff
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.left_loads_sub = self.create_subscription(StepStamped, self.left_loads_topic_name, self.left_loads_cb, 10) 
        self.right_loads_sub = self.create_subscription(StepStamped, self.right_loads_topic_name, self.right_loads_cb, 10) 
        self.sub = self.create_subscription(Odometry, self.odom_topic_name, self.odom_callback, 10)

        self.tmr = self.create_timer(self.period, self.timer_callback)

        self.get_logger().info("Speed Gait monitor started")  

    def odom_callback(self, msg):
        self.prev_position = self.cur_position
        self.cur_position = msg.pose.pose.position

        if (self.prev_position!= None):
            d = np.sqrt(np.power(self.cur_position.x - self.prev_position.x, 2.0 ) + np.power(self.cur_position.y - self.prev_position.y, 2.0 ) )            
            self.travelled += d

    def get_difference(self, position_rel, prev_position_rel):
        # Cast positions to global frame
        try:
            p = self.buffer.transform(position_rel, self.global_frame_id)
            pp = self.buffer.transform(prev_position_rel, self.global_frame_id)
            
            # Distance
            d = np.sqrt(np.power(p.point.x - pp.point.x, 2.0 ) + np.power(p.point.y - pp.point.y, 2.0 ) )

        except Exception as e:
            self.get_logger().error("Can't transform points into frame [" + self.global_frame_id + "]: [" + str(e) + "]")    
            d = np.nan        
        return d


    def left_loads_cb(self, msg):
        if (msg.load>0):
            if not self.is_left_load:
                self.update_stats(msg, self.left_loads, self.right_loads, 
                    self.left_stride_times, self.left_stride_lenghts, self.left_step_times, self.left_step_lenghts)
            self.is_left_load = True
        else:
            self.is_left_load = False

    def right_loads_cb(self, msg):
        if (msg.load>0):
            if not self.is_right_load:
                self.update_stats(msg, self.right_loads, self.left_loads, 
                    self.right_stride_times, self.right_stride_lenghts, self.right_step_times, self.right_step_lenghts)
            self.is_right_load = True
        else:
            self.is_right_load = False

    # mfc: note to self: as all parameters are lists, it will modify its content after return.
    def update_stats(self, msg, leg_loads, opposite_loads, leg_stride_times, leg_stride_lenghts, leg_step_times, leg_step_lenghts):
            leg_loads.append( msg)
            # If we have at least two steps
            if (len(leg_loads)>1):
                stride_time = leg_loads[-1].position.header.stamp - leg_loads[-2].position.header.stamp 
                leg_stride_times.append(stride_time)
                stride_lenght = self.get_difference(leg_loads[-1].position, leg_loads[-2].position)
                leg_stride_lenghts.append(stride_lenght)
            # Last step load from the other leg
            if (len(opposite_loads)>0):
                step_time = leg_loads[-1].position.header.stamp - opposite_loads[-1].position.header.stamp 
                leg_step_times.append(step_time)
                step_lenght = self.get_difference(leg_loads[-1].position, opposite_loads[-1].position)
                leg_step_lenghts.append(step_lenght)

    def timer_callback(self):
        if ( (len(self.left_loads)>0) and  (len(self.right_loads)>0) ):
            l_SpT = np.nanmean(self.left_step_times)
            l_SdT = np.nanmean(self.left_stride_times)
            l_NoS = len(self.left_loads)
            l_SpL = np.nanmean(self.left_stride_lenghts)
            l_SdL = np.nanmean(self.left_step_lenghts)

            r_SpT = np.nanmean(self.right_step_times)
            r_SdT = np.nanmean(self.right_stride_times)
            r_NoS = len(self.right_loads)
            r_SpL = np.nanmean(self.right_stride_lenghts)
            r_SdL = np.nanmean(self.right_step_lenghts)

            NoS = l_NoS + r_NoS

            if ((self.left_loads[0].position.header.stamp - self.right_loads[0].position.header.stamp)>0):
                start_stamp = self.right_loads[0].position.header.stamp
            else:
                start_stamp = self.left_loads[0].position.header.stamp


            if ((self.left_loads[-1].position.header.stamp - self.right_loads[-1].position.header.stamp)>0):
                end_stamp = self.left_loads[-1].position.header.stamp
            else:                
                end_stamp = self.right_loads[-1].position.header.stamp
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

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = GaitMonitorSp()

    rclpy.spin(minimal_subscriber)
 
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()