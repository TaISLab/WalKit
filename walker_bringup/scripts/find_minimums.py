import rclpy
from rclpy.node import Node

from  sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy



class FindMinimus(Node):

    def __init__(self, min_val_, letopic):
        super().__init__('minimus_finder')
        self.topic_name = letopic        
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription( LaserScan, self.topic_name, self.listener_callback, qos_profile=qos_profile)

        self.min_val = min_val_



    def listener_callback(self, msg):
        self.ranges = msg.ranges
        for i in range(len(self.ranges)):
            if (self.ranges[i]< self.min_val):
                print("ranges[" + str(i) + " ] = " + str(self.ranges[i]) )
        print(" . . . . . . .  . . . . in "+ self.topic_name +" \n")



def main(args=None):
    rclpy.init(args=args)

#    minimal_finder = FindMinimus(0.47,'scan_nav')
    minimal_finder = FindMinimus(0.53,'scan')

    rclpy.spin(minimal_finder)

    minimal_finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
