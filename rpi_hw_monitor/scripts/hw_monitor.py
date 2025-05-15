#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rpi_hw_monitor.msg import HardwareStatus


class HardwareMonitor(Node):
    def __init__(self):
        super().__init__("hw_monitor")
        self.declare_parameter("polling_frequency", 1)
        self.declare_parameter("cpu", True)
        self.declare_parameter("memory", True)
        self.declare_parameter("disk", True)
        self.declare_parameter("network", True)
        self.declare_parameter("power", True)
        self.declare_parameter("temperature", True)

        self.polling_frequency = self.get_parameter("polling_frequency").value
        self.monitor_flags = {
            "cpu": self.get_parameter("cpu").value,
            "memory": self.get_parameter("memory").value,
            "disk": self.get_parameter("disk").value,
            "network": self.get_parameter("network").value,
            "power": self.get_parameter("power").value,
            "temperature": self.get_parameter("temperature").value,
        }

        self.publisher = self.create_publisher(HardwareStatus, "status", 10)
        self.timer = self.create_timer(self.polling_frequency, self.poll_hardware)

    def poll_hardware(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = HardwareMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
