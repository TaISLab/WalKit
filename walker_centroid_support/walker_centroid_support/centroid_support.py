import yaml
import os
import numpy as np
from scipy import interpolate

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from tf2_ros.buffer import Buffer
from ament_index_python.packages import get_package_share_directory
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PointStamped 
from walker_msgs.msg import ForceStamped, StepStamped 

class CentroidSupport(Node):

    def __init__(self):
        super().__init__('centroid_support')
        self.declare_parameters(
            namespace='',
            parameters=[
                 ('handle_calibration_file', os.path.join(get_package_share_directory('walker_loads'), "config", "params.yaml") ),
                 ('left_loads_topic_name', '/left_loads'),
                 ('right_loads_topic_name', '/right_loads'),
                 ('left_handle_topic_name', '/left_handle'),
                 ('right_handle_topic_name', '/right_handle'),
                 ('user_desc_topic_name', '/user_desc'),
                 ('period', 0.05),
                 ('speed_delta', 0.05),
                 ('centroid_topic_name', '/support_centroid'),
            ]
        )


        self.left_loads_topic_name = self.get_parameter('left_loads_topic_name').value
        self.right_loads_topic_name = self.get_parameter('right_loads_topic_name').value
        self.left_handle_topic_name = self.get_parameter('left_handle_topic_name').value
        self.right_handle_topic_name = self.get_parameter('right_handle_topic_name').value
        self.user_desc_topic_name = self.get_parameter('user_desc_topic_name').value
        self.centroid_topic_name = self.get_parameter('centroid_topic_name').value
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

        self.weight = 100
        self.right_handle_weight = 0
        self.left_handle_weight  = 0
        self.leg_load = 0

        self.right_handle_pose = None
        self.left_handle_pose = None

        # ROS stuff        
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.new_data_available = False
        self.first_data_ready = False
        
        # ROS stuff
        self.centroid_pub = self.create_publisher(PointStamped, self.centroid_topic_name, 10)

        self.left_handle_sub = self.create_subscription(ForceStamped, self.left_handle_topic_name, self.handle_lc, 10) 
        self.right_handle_sub = self.create_subscription(ForceStamped, self.right_handle_topic_name, self.handle_lc, 10) 

        self.left_load_sub  = self.create_subscription(StepStamped, self.left_loads_topic_name,  self.l_steps_lc, 10)
        self.right_load_sub = self.create_subscription(StepStamped, self.right_loads_topic_name, self.r_steps_lc, 10)

        # mimics a ROS1 'latched' topic
        latched_profile = QoSProfile(depth=1)
        latched_profile.history = QoSHistoryPolicy.KEEP_LAST
        latched_profile.reliability = QoSReliabilityPolicy.RELIABLE
        latched_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.user_desc_sub = self.create_subscription(String, self.user_desc_topic_name, self.user_desc_lc, latched_profile) 
        self.tmr = self.create_timer(self.period, self.timer_callback)


        self.get_logger().info("centroid support started")  

    def user_desc_lc(self, msg):
        user_fields = msg.data.split(':')
        if len(user_fields)>2:
            new_weight = int(user_fields[2])
            if (new_weight != self.weight):
                self.weight = new_weight
                self.new_data_available = True

    def handle_lc(self, msg):
        if ('right' in msg.header.frame_id ):
            self.right_handle_msg = msg          
            self.right_handle_weight = self.fr(msg.force)
            self.new_data_available = True
            # Get handle position in the same frame as the steps.
            # We need this only once... unless you change steps frame over time?
            if (self.right_handle_pose==None):
                self.right_handle_pose= self.get_handle_pose(self.right_step_msg, self.right_handle_msg)
        elif ('left' in msg.header.frame_id):
            self.left_handle_msg = msg
            self.left_handle_weight  = self.fl(msg.force) 
            self.new_data_available = True
            if (self.left_handle_pose==None):
                self.left_handle_pose = self.get_handle_pose(self.left_step_msg, self.left_handle_msg)
        else:
            self.get_logger().error("Don't know about which handle are you talking [" + msg.header.frame_id + "]")    
            return

    def l_steps_lc(self, msg):
        self.steps_lc(msg,0)

    def r_steps_lc(self, msg):
        self.steps_lc(msg,1)

    def steps_lc(self, msg, id):
        if  (id==1):
                self.right_step_msg = msg
                self.right_leg_load = self.right_step_msg.load 
                self.right_speed = self.right_step_msg.speed
                self.new_data_available = True
        elif (id==0):
                self.left_step_msg = msg
                self.left_leg_load = self.left_step_msg.load
                self.left_speed = self.left_step_msg.speed
                self.new_data_available = True
        else:
            self.get_logger().error("Don't know about which step are you talking [" + id + "]")    
            return          
        
    def timer_callback(self):
        if (self.new_data_available):
            self.new_data_available = False

            if (not self.first_data_ready ):
                self.first_data_ready = (self.left_handle_msg is not None)  and (self.right_handle_msg is not None) and (self.left_step_msg is not None) and (self.right_step_msg is not None) and (self.right_handle_pose is not None) and (self.left_handle_pose is not None)   

            if ( self.first_data_ready ):
                    right_step_position = np.array([self.right_step_msg.position.point.x, self.right_step_msg.position.point.y, 0])
                    left_step_position = np.array([self.left_step_msg.position.point.x, self.left_step_msg.position.point.y, 0])
                    right_handle_position = np.array([self.right_handle_pose.point.x, self.right_handle_pose.point.y, self.right_handle_pose.point.z])
                    left_handle_position = np.array([self.left_handle_pose.point.x, self.left_handle_pose.point.y, self.left_handle_pose.point.z])

                    centroid = ( self.right_leg_load      * right_step_position   + 
                                 self.left_leg_load       * left_step_position    + 
                                 self.right_handle_weight * right_handle_position + 
                                 self.left_handle_weight  * left_handle_position  ) / self.weight

                    centroid_msg = PointStamped()
                    centroid_msg.header = self.right_step_msg.position.header
                    centroid_msg.point.x = centroid[0]
                    centroid_msg.point.y = centroid[1]
                    centroid_msg.point.z = centroid[2]
                    
                    self.centroid_pub.publish(centroid_msg)

            else:
                #pass
                self.get_logger().error("Not all data received yet ...")
                if (self.left_handle_msg is None):
                    self.get_logger().error("\tleft_handle" )
                if (self.right_handle_msg is None):
                    self.get_logger().error("\tright_handle" )
                if (self.left_step_msg is None):
                    self.get_logger().error("\tleft_step" )
                if (self.right_step_msg is None):
                    self.get_logger().error("\tright_step" )
                if (self.right_handle_pose is None):
                    self.get_logger().error("\tright_handle_pose" )
                if (self.left_handle_pose is None):
                    self.get_logger().error("\tleft_handle_pose" )

    # We take 0,0,0 in handles frame and cast it to steps frame.
    def get_handle_pose(self, step_msg, handle_msg):
        handle_orig = None
        if ( (step_msg is not None) and (handle_msg is not None)):
            try:
                # Handle "is" at the origin of its own tf. So we just need to know the transform between both
                trans = self.buffer.lookup_transform( step_msg.position.header.frame_id, handle_msg.header.frame_id, rclpy.time.Time())                
                #trans.transform.translation
                #trans.transform.rotation
                handle_orig = PointStamped()
                handle_orig.header = step_msg.position.header
                handle_orig.point.x = trans.transform.translation.x
                handle_orig.point.y = trans.transform.translation.y
                handle_orig.point.z = trans.transform.translation.z   
            except Exception as e:
                self.get_logger().error("Can't transform handle point from frame [" + handle_msg.header.frame_id + "] into frame [" + step_msg.position.header.frame_id + "]: [" + str(e) + "]")    
        return handle_orig

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = CentroidSupport()

    rclpy.spin(minimal_subscriber)
 
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
