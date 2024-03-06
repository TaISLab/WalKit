import rclpy
from rclpy.node import Node
import numpy as np
from  sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class LaserFilterGenerator(Node):

    def __init__(self, min_val_, filter_prob, build_time, letopic):
        super().__init__('minimus_finder')
        self.min_val = min_val_
        self.filter_prob = filter_prob
        self.build_time = build_time
        self.topic_name = letopic
        self.scans_read = 0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription( LaserScan, self.topic_name, self.listener_callback, qos_profile=qos_profile)
        self.timer = self.create_timer(self.build_time, self.find_minimums)
        self.get_logger().info("Node created")




    def listener_callback(self, msg):
        self.ranges = msg.ranges
        self.get_logger().info(".")
        if not hasattr(self, 'range_counts'): 
            self.range_counts = np.zeros_like(self.ranges)

        for i in range(len(self.ranges)):
            if (self.ranges[i]< self.min_val):
                self.range_counts[i] = self.range_counts[i] + 1
        self.scans_read = self.scans_read +1


    def find_minimums(self):
        self.filtered_indexs = []
        if self.scans_read>0:
            self.range_probs = self.range_counts/ self.scans_read 
            
            for i in range(len(self.range_probs)):
                if (self.range_probs[i]>= self.filter_prob):
                    self.filtered_indexs.append(i)
            
            self.get_logger().info("After " + str(self.scans_read) + " readings, the following indexs have at least probability " + str(self.filter_prob) + " of returning a distance equal or lower to " + str(self.min_val))
            for i in range(len(self.filtered_indexs)):
                print(self.filtered_indexs[i])

            f = open("demofile3.txt", "w")
            f.write("laser_filter:\n")
            f.write("  ros__parameters:\n")
            f.write("    filter1:\n")
            f.write("      name: mask\n")
            f.write("      type: laser_filters/LaserScanMaskFilter\n")
            f.write("      params:\n")
            f.write("        masks:\n")
            f.write("          laser:\n")
            for i in range(len(self.filtered_indexs)):
                f.write("          - " + str(i) + ".\n")
            f.close()

        else:
            self.get_logger().info("No scans read in topic " + self.topic_name)
        exit()


def main(args=None):
    rclpy.init(args=args)

    generator = LaserFilterGenerator(0.53, 0.20, 20, 'scan')

    rclpy.spin(generator)

    generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
