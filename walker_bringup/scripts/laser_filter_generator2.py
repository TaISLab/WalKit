import rclpy
from rclpy.node import Node
import numpy as np
from  sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class LaserFilterGenerator2(Node):

    def __init__(self, build_time_, threshold_, letopic):
        super().__init__('minimus_finder2')
        self.threshold = threshold_
        self.build_time = build_time_
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


'''
        ranges = np.array(msg.ranges)

        # Filtrar valores inválidos (NaN o infinitos)
        valid = np.isfinite(ranges)
        ranges = ranges[valid]

        if len(ranges) == 0:
            self.get_logger().warn('No hay datos válidos en el mensaje de láser.')
            return

        # Inicialización de arrays acumuladores en la primera pasada
        if self.sum_ranges is None:
            n_points = len(msg.ranges)
            self.sum_ranges = np.zeros(n_points)
            self.sum_squared_ranges = np.zeros(n_points)

        # Rellenar los valores no válidos con 0 para no contaminar las estadísticas
        full_ranges = np.nan_to_num(msg.ranges, nan=0.0, posinf=0.0, neginf=0.0)

        self.sum_ranges += full_ranges
        self.sum_squared_ranges += full_ranges ** 2
        self.count += 1


        if self.count % 50 == 0:  # Mostrar estadísticas cada 50 mensajes
            mean = self.sum_ranges / self.count
            variance = (self.sum_squared_ranges / self.count) - mean**2
            self.get_logger().info(f'--- Estadísticas acumuladas tras {self.count} mensajes ---')
            self.get_logger().info(f'Media de distancias (primeros 10 puntos): {mean[:10]}')
            self.get_logger().info(f'Varianza de distancias (primeros 10 puntos): {variance[:10]}')

'''

    def listener_callback(self, msg):
        self.ranges = np.array(msg.ranges)
        if not hasattr(self, 'original_ranges'): 
            self.range_is_self_colision = np.zeros_like(self.ranges)
            self.original_ranges = self.ranges
            self.original_ranges[self.original_ranges == 0] = 1e-15
        else:
            diff = 100*np.abs(self.ranges-self.original_ranges)/self.original_ranges

            self.range_is_self_colision[diff<self.threshold] = self.range_is_self_colision[diff<self.threshold] + 1
        self.scans_read = self.scans_read +1

    def find_minimums(self):
        self.filtered_indexs = []
        if self.scans_read>0:
            self.get_logger().info(f"After {self.scans_read} readings, the following indexs have varied its value less than {self.threshold} percent at least half the time")

            num_beams = len(self.range_is_self_colision)
            for i in range(num_beams):
                if (self.range_is_self_colision[i]>0.25*self.scans_read):
                    self.filtered_indexs.append(i)
                    print(f"index ({self.filtered_indexs[-1]}) == {self.range_is_self_colision[i]} times exceeded")
            num_collisions = len(self.filtered_indexs)
            print(f"In total {num_collisions} of {num_beams} indexs have self colliding points")

            f = open("demofile3.txt", "w")
            f.write("laser_filter:\n")
            f.write("  ros__parameters:\n")
            f.write("    filter1:\n")
            f.write("      name: mask\n")
            f.write("      type: laser_filters/LaserScanMaskFilter\n")
            f.write("      params:\n")
            f.write("        masks:\n")
            f.write("          laser:\n")

            for i in range((num_collisions)):
                f.write("          - " + str(self.filtered_indexs[i]) + ".\n")
            f.close()

        else:
            self.get_logger().info("No scans read in topic " + self.topic_name)
        exit()


def main(args=None):
    rclpy.init(args=args)

    generator = LaserFilterGenerator2(40, 20, 'scan')

    rclpy.spin(generator)

    generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
