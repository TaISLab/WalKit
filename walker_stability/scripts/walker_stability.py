import pandas as pd
import math
import numpy as np
from os.path import join
from sys import stderr

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
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
                 ('user_desc_topic_name', '/user_desc')
            ]
        )

        self.antropometric_data_file_uri= self.get_parameter('antropometric_data_file_uri').value
        self.stability_topic_name= self.get_parameter('stability_topic_name').value
        self.centroid_topic_name= self.get_parameter('centroid_topic_name').value
        self.user_desc_topic_name= self.get_parameter('user_desc_topic_name').value
        
        # stored data
        self.user_desc = ''
        self.age = 0
        self.height = 0
        self.weight = 0
        self.user_id = ''
        self.gender = ''
        self.tinetti = 0
        self.description = ''
        self.bos_x = 0
        
        # ROS stuff
        self.stability_pub  = self.create_publisher(StabilityStamped, self.stability_topic_name,  10)
        
        self.centroid_sub = self.create_subscription(String, self.centroid_topic_name, self.centroid_callback, 10)

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

        print("user_height " + str(user_height) + " mm. \n" +
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
        if (len(user_fields)>6) and (msg.data != self.user_desc):
            self.user_desc = msg.data
            self.age = int(user_fields[0])
            self.height = int(user_fields[1])
            self.weight = int(user_fields[2])
            self.user_id = user_fields[3]
            self.gender = user_fields[4] # Femenino or Masculino
            self.tinetti = int(user_fields[5])
            self.description = user_fields[6]
            
            self.bos_x = self.get_bos_x(self.gender, self.age, self.user_height, handlebar_height_m)

    def centroid_callback(self, msg):
        




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