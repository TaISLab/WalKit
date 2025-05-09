"""
Does an statistic analysis of laser readings
Those with low variation are likely to be self colisions
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from  sensor_msgs.msg import LaserScan
import numpy as np


class LaserFilterGeneratorMk2(Node):

    def __init__(self, min_std, min_ave, build_time, letopic):
        super().__init__('laser_filter_gen_mk2')
        self.build_time = build_time
        self.topic_name = letopic
        self.min_std = min_std
        self.min_ave = min_ave

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription( LaserScan, self.topic_name,
                                self.listener_callback, qos_profile=qos_profile)

        self.timer = None
        self.ranges_data = None

        self.get_logger().info("Node created")

    def listener_callback(self, msg):

        self.get_logger().info(".")
        if self.ranges_data is None:
            self.timer = self.create_timer(self.build_time, self.find_minimums)
            self.ranges_data = dict()
            for i in range(len(msg.ranges)):
                self.ranges_data[i] = []

        for i,range_i in enumerate(msg.ranges):
            self.ranges_data[i].append(range_i)

    def find_minimums(self):
        scans_read = len(self.ranges_data[0])
        filtered_indexs = []
        if scans_read>0:

            ranges_data_std = dict()
            ranges_data_fin = dict()
            ranges_data_ave = dict()
            for i,range_i in self.ranges_data.items():
                r = np.array(range_i)
                r = r[np.isfinite(r)]
                ranges_data_std[i] = np.std(r)
                ranges_data_ave[i] = np.mean(r)
                ranges_data_fin[i] = np.count_nonzero(np.isfinite(range_i))
                if (ranges_data_std[i]<= self.min_std) or (ranges_data_ave[i]<= self.min_ave):
                    filtered_indexs.append(i)

            show_debug = True
            if show_debug:
                self.get_logger().info("After " + str(scans_read) +
                                        " readings, standard deviations on each index are: ")
                for i,std_i in ranges_data_std.items():
                    ave_i = ranges_data_ave[i]
                    print(str(i) + '=> av(' + str(round(ave_i,2)) + ')' +
                                   ' std(' + str(round(std_i,2)) + ')'  +
                                   ' prob. (' + str(round(100.0*ranges_data_fin[i]/scans_read,2)) + ')' )


            self.get_logger().info("After " + str(scans_read) +
                " readings, following indexs have standard deviations lower than " +
                str(self.min_std) + ' or average under ' +
                str(self.min_ave))

            for index_i in filtered_indexs:
                ave_i = ranges_data_ave[index_i]
                std_i = ranges_data_std[index_i]
                prob_i = 100.0*ranges_data_fin[i]/scans_read
                print(str(index_i) + 
                      '\t => av(' + str(round(ave_i,2)) + ')' +
                      '\t   std(' + str(round(std_i,2)) + ')'  +
                      '\t   prb(' + str(round(prob_i,2)) + ')' )

            f = open("demofile3.txt", "w")
            f.write("laser_filter:\n")
            f.write("  ros__parameters:\n")
            f.write("    filter1:\n")
            f.write("      name: mask\n")
            f.write("      type: laser_filters/LaserScanMaskFilter\n")
            f.write("      params:\n")
            f.write("        masks:\n")
            f.write("          laser:\n")
            for i in range(len(filtered_indexs)):
                f.write("          - " + str(filtered_indexs[i]) + ".\n")
            f.close()

        else:
            self.get_logger().info("No scans read in topic " + self.topic_name)
        exit()


def main(argv=sys.argv):
    rclpy.init(args=None)

    min_std = 0.2
    compute_time = 30
    min_ave = 0.58

    if len(argv)==4:
        min_std = float(argv[1])
        min_ave = float(argv[2])
        compute_time = int(argv[3])

    generator = LaserFilterGeneratorMk2(min_std, min_ave, compute_time, 'scan')

    rclpy.spin(generator)

    generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
