#!/usr/bin/env python3

import rclpy
import psutil
import ros2_numpy as rnp
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


#I'm forced to use that if Wifi not detected by robot
class CPU_Record(Node):
    def __init__(self):
        super().__init__('topic_subscriber')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.cpu_pub = self.create_publisher(
            Float32,
            '/cpu_percent',
            qos_profile
        )

        self.ram_perc_pub = self.create_publisher(
            Float32,
            '/ram_percent',
            qos_profile
        )

        self.ram_used_pub = self.create_publisher(
            Float32,
            '/ram_used',
            qos_profile
        )

        self.cpu = 0
        self.ram_perc = 0
        self.ram_used = 0
        self.timer = self.create_timer(1.0, self.record_memory_usage)


    def record_memory_usage(self):
        self.cpu = float()
        self.cpu = psutil.cpu_percent(interval=1)
        ram = psutil.virtual_memory()
        print("CPU usage (%):", self.cpu)

        self.ram_perc = ram.percent
        self.ram_used = round(ram.used / 1e9, 2)

        msg = Float32()
        msg.data = self.cpu
        self.cpu_pub.publish(msg)
        msg.data = self.ram_perc
        self.ram_perc_pub.publish(msg)
        msg.data = self.ram_used
        self.ram_used_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    data_publisher = CPU_Record()

    try:
        rclpy.spin(data_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        data_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()