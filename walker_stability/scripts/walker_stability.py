import pandas as pd
import math
import numpy as np
from os.path import join
from sys import stderr

import rclpy
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

class WalkerStability(Node):

    def __init__(self):
        super().__init__('user_stability')
    
        self.declare_parameters(
            namespace='',
            parameters=[
                 ('antropometric_data_file_uri', join(get_package_share_directory('walker_stability'), "config", "Correlationes.xlsx") ),
                 ('stability_topic_name', '/user_stability'),
                 ('centroid_topic_name', '/support_centroid'),
                 ('user_desc_topic_name', '/user_desc'),
                 ('base_footprint_frame', 'base_footprint'),
                 ('left_handle_frame', 'left_handle_id'),
                 ('right_handle_frame', 'right_handle_id')
            ]
        )

        self.antropometric_data_file_uri = self.get_parameter('antropometric_data_file_uri').value
        self.stability_topic_name = self.get_parameter('stability_topic_name').value
        self.centroid_topic_name = self.get_parameter('centroid_topic_name').value
        self.user_desc_topic_name = self.get_parameter('user_desc_topic_name').value
        self.base_footprint_frame = self.get_parameter('base_footprint_frame').value
        self.left_handle_frame = self.get_parameter('left_handle_frame').value
        self.right_handle_frame = self.get_parameter('right_handle_frame').value

        # stored data
        self.has_user_data = False
        self.user_desc = ''
        self.user_age = 0
        self.user_height = 0
        self.user_weight = 0
        self.user_id = ''
        self.user_gender = ''
        self.user_tinetti = 0
        self.user_description = ''
        self.bos_x_min = 0
        self.bos_x_max = 0
        self.handle_z = 0
        self.handle_x = 0
        self.handle_y_max = 0
        self.handle_y_min = 0

        # ROS stuff
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.stability_pub  = self.create_publisher(StabilityStamped, self.stability_topic_name,  10)
        
        self.centroid_sub = self.create_subscription(PointStamped, self.centroid_topic_name, self.centroid_callback, 10)

        # for user description topic, we mimic a ROS1 'latched' topic
        latched_profile = QoSProfile(depth=1)
        latched_profile.history = QoSHistoryPolicy.KEEP_LAST
        latched_profile.reliability = QoSReliabilityPolicy.RELIABLE
        latched_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.user_desc_sub = self.create_subscription(String, self.user_desc_topic_name, self.user_desc_callback, latched_profile) 
        
        self.get_logger().info("user stability node started")  

    def get_bos_x(self, gender, age, user_height_m, handlebar_height_m):
        user_height = user_height_m *1000
        handlebar_height = handlebar_height_m *1000

        xls = pd.ExcelFile(self.antropometric_data_file_uri)
        if (gender == 'Femenino'):
            df = pd.read_excel(xls, 'AnthroprometricFemale')
        else:
            df = pd.read_excel(xls, 'AnthroprometricMale')

        #Find the correct line
        index = 0
        if (age >= df['Min_age'][index]):
            while(age < df['Min_age'][index] or age > df['Max_age'][index]):
                index+=1

        shoulder_height = df['StatureX_ShoulderY_P'][index]*(user_height-df['StatureX_ShoulderY_MeanX'][index])+df['StatureX_ShoulderY_MeanY'][index]
        elbow_height = df['ShoulderX_ElbowY_P'][index]*(shoulder_height-df['ShoulderX_ElbowY_MeanX'][index])+df['ShoulderX_ElbowY_MeanY'][index]
        fist_height = df['ElbowX_FistY_P'][index]*(elbow_height-df['ElbowX_FistY_MeanX'][index])+df['ElbowX_FistY_MeanY'][index]

        arm = shoulder_height-elbow_height
        forearm = elbow_height-fist_height
        shoulder_height_translated = shoulder_height-handlebar_height
        threshold = 10 #mm
        result = []
        for alpha in np.arange(0,90,0.1):
            for beta in np.arange(alpha,180,0.1):
                elbow_extension = beta-alpha
                if ((elbow_extension>=20) and (elbow_extension<=30) and (math.fabs(forearm*math.sin(alpha)+arm*math.sin(beta)-shoulder_height_translated)<threshold)):
                    result.append([alpha, beta,forearm*math.cos(alpha)+arm*math.cos(beta)])

        self.get_logger().debug("user_height " + str(user_height) + " mm. \n" +
            "handlebar_height " + str(handlebar_height) + " mm. \n" +
            "age " + str(age) +"\n" + 
            "gender " + " " + gender )
        
        if (len(result)==0):
            self.get_logger().warn("No solution recommended handlebar height: " + str((user_height) * 0.45 + 87 ))
            max_d = 1
            min_d = -1
        else:
            max_d = max([row[2] for row in result]) / 1000
            min_d = min([row[2] for row in result]) / 1000
            self.get_logger().debug("Max distance: " +str(max_d) + " m. min distance: " + str(min_d) + " m.")
            self.get_logger().debug("Current handlebar height: " +str(handlebar_height) + " mm. recommended: " + str((user_height) * 0.45 + 87 ))
        
        return (min_d, max_d)


    def user_desc_callback(self, msg):
        user_fields = msg.data.split(':')
        if (msg.data != self.user_desc):   
            if (len(user_fields)<6):
                self.get_logger().warn("User descr is too short! [" + str(len(user_fields)) + "]")
                return 
            
            self.get_logger().info("Valid user data data received.")
            self.has_user_data = True

            if (len(user_fields)==6) or (len(user_fields)==7):
                self.user_desc = msg.data
                self.user_age = int(user_fields[0])
                self.user_height = int(user_fields[1])
                self.user_weight = int(user_fields[2])
                self.user_id = user_fields[3]
                self.user_gender = user_fields[4] # Femenino or Masculino

            if (len(user_fields)==7):
                self.user_tinetti = int(user_fields[5])/28.0
                self.user_description = user_fields[6]
                
            if (len(user_fields)==6):
                self.get_logger().warn("No tinetti in user descr. Assuming 24 points")
                self.user_tinetti = 24.0/28.0
                self.user_description = user_fields[5]
            

    def centroid_callback(self, msg):

        if self.has_user_data:    
            # get latest tf
            try:
                left_handle_transformation = self.tf_buffer.lookup_transform(self.base_footprint_frame, self.left_handle_frame, Time(seconds=0, nanoseconds=0), Duration(seconds=1.0))
                right_handle_transformation = self.tf_buffer.lookup_transform(self.base_footprint_frame, self.right_handle_frame, Time(seconds=0, nanoseconds=0), Duration(seconds=1.0))
                # Get current handle positions in local coordinates too
                centroid = self.tf_buffer.transform(msg, self.base_footprint_frame)
                self.handle_z = left_handle_transformation.transform.translation.z
                self.handle_x = left_handle_transformation.transform.translation.x
                self.handle_y_max = left_handle_transformation.transform.translation.y
                self.handle_y_min = right_handle_transformation.transform.translation.y
            except Exception as e:
                self.get_logger().error("Can't GET transform from handles frame  [" + self.left_handle_frame + ", " + self.right_handle_frame + "] into centroid frame [" + self.base_footprint_frame + "]: [" + str(e) + "]")   
                return 

            (self.bos_x_min,self.bos_x_max)  = self.get_bos_x(self.user_gender, self.user_age, self.user_height, self.handle_x, self.handle_z)
            # Stability depends on centroid distance to these
            feet_handle_dist_x = self.handle_x - centroid.x

            # stability in y axis: 0 in border or larger, max in center            
            if (centroid.y<self.handle_y_min):
                y_sta = 0
            elif (centroid.y>self.handle_y_max):
                y_sta = 0
            else:
                y_sta = (centroid.y - self.handle_y_min) / (self.handle_y_max - self.handle_y_min)

            # stability in x axis: 0 in border or larger, max in center            
            if (feet_handle_dist_x<self.bos_x_min):
                x_sta = 0
            elif (feet_handle_dist_x>self.bos_x_max):
                x_sta = 0
            else:
                x_sta = (feet_handle_dist_x - self.bos_x_min) / (self.bos_x_max - self.bos_x_min)


            # publish Stability data
            outMsg = StabilityStamped()

            # User ID
            outMsg.uid = self.user_id

            # User Tineti score
            outMsg.tin = self.user_tinetti

            # Stability value (0-1)
            outMsg.sta = x_sta * y_sta

            # User pose for given stability (m)  
            outMsg.pose = PoseStamped()
            # note: we could use walker pose instead, so it could have correct orientation
            outMsg.pose.header = centroid.header
            outMsg.pose.pose.position = centroid.point
            outMsg.pose.pose.orientation.w = 1

            # and publish 
            self.stability_pub.publish(outMsg)

        else:
            self.get_logger().warn("No user description received yet. Ignoring centroid msg.")   





def main(args=None):
    rclpy.init(args=args)

    myNode = WalkerStability()

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