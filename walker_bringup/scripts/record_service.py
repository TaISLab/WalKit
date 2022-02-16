#!/usr/bin/env python3

from std_srvs.srv import SetBool

import rclpy
from rclpy.node import Node
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
import psutil
import signal

class RecordService(Node):

    def __init__(self):
        super().__init__('record_service')
        command_dir = get_package_share_directory('walker_bringup')
        self.comm_record = os.path.join(command_dir, 'script', 'command_record.sh')
        self.comm_stop = os.path.join(command_dir, 'script', 'command_stop.sh')
        self.start_srv = self.create_service(SetBool, '/rosbag_record', self.record_callback)
        self.is_recording = False
        self.get_logger().info("record service available")

    def record_callback(self, request, response):
        if request.data:            # record
            self.get_logger().info("Request: record")
            if not self.is_recording:
                #os.spawnlp(os.P_DETACH, 'recorder', self.comm_record)
                output = subprocess.run(self.comm_record, shell=True)
                # subprocess.Popen(self.comm_record,
                #  stdout=open('/tmp/logfile.log', 'w'),
                #  stderr=open('/tmp/err_logfile.log', 'a'),
                #  preexec_fn=os.setpgrp
                #  )                
                response.success = True
                self.is_recording = True
                response.message = 'record started'
            else:
                response.success = False
                response.message = 'record already running'
        if not request.data:        # stop record
            self.get_logger().info("Request: stop")
            if self.is_recording:
                #os.spawnlp(os.P_DETACH, 'stopper', self.comm_stop)
                output = subprocess.run(self.comm_stop, shell=True)
                # subprocess.Popen(self.comm_stop,
                #  stdout=open('/tmp/logfile.log', 'w'),
                #  stderr=open('/tmp/err_logfile.log', 'a'),
                #  preexec_fn=os.setpgrp
                #  )
                response.success = True
                self.is_recording = False
                response.message = 'record stopped'
            else:
                response.success = False
                response.message = 'record already stopped'

            self.get_logger().info("Response: " + response.message)

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = RecordService()

    rclpy.spin(minimal_service)
    minimal_service.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()