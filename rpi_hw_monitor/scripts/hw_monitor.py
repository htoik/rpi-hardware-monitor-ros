#!/usr/bin/env python3
import subprocess
import time

import psutil
import rclpy
from rclpy.node import Node

from rpi_hw_monitor.msg import HardwareStatus


class HardwareMonitor(Node):
    def __init__(self):
        super().__init__("hw_monitor")
        self.declare_parameter("polling_frequency", 1.0)
        self.declare_parameter("cpu", True)
        self.declare_parameter("memory", True)
        self.declare_parameter("disk", True)
        self.declare_parameter("network", True)
        self.declare_parameter("power", True)
        self.declare_parameter("temperature", True)

        self.polling_frequency = 1.0 / self.get_parameter("polling_frequency").value
        self.monitor_flags = {
            "cpu": self.get_parameter("cpu").value,
            "memory": self.get_parameter("memory").value,
            "disk": self.get_parameter("disk").value,
            "network": self.get_parameter("network").value,
            "power": self.get_parameter("power").value,
            "temperature": self.get_parameter("temperature").value,
        }

        self.prev_disk = psutil.disk_io_counters()
        self.prev_net = psutil.net_io_counters()
        self.prev_time = time.time()

        self.publisher = self.create_publisher(HardwareStatus, "hw_status", 10)
        self.timer = self.create_timer(self.polling_frequency, self.poll_hardware)

    def poll_hardware(self):
        msg = HardwareStatus()

        msg.cpu_usage = 0.0
        msg.memory_mb = 0.0
        msg.disk_rx_mb = 0.0
        msg.disk_tx_mb = 0.0
        msg.network_rx_mb = 0.0
        msg.network_tx_mb = 0.0
        msg.power_consumption = 0.0

        if self.monitor_flags["cpu"]:
            msg.cpu_usage = psutil.cpu_percent(interval=None)

        if self.monitor_flags["memory"]:
            mem = psutil.virtual_memory()
            msg.memory_mb = mem.used / (1024**2)

        if self.monitor_flags["disk"]:
            disk = psutil.disk_io_counters()
            msg.disk_rx_mb = (disk.read_bytes - self.prev_disk.read_bytes) / (1024**2)
            msg.disk_tx_mb = (disk.write_bytes - self.prev_disk.write_bytes) / (1024**2)
            self.prev_disk = disk

        if self.monitor_flags["network"]:
            net = psutil.net_io_counters()
            msg.network_rx_mb = (net.bytes_recv - self.prev_net.bytes_recv) / (1024**2)
            msg.network_tx_mb = (net.bytes_sent - self.prev_net.bytes_sent) / (1024**2)
            self.prev_net = net

        if self.monitor_flags["power"]:
            msg.power_consumption = 0.0

        if self.monitor_flags["temperature"]:
            try:
                with open("/sys/class/thermal/thermal_zone0/temp") as f:
                    temp_raw = int(f.read().strip())
                msg.temperature = temp_raw / 1000.0
            except FileNotFoundError:
                out = subprocess.check_output(["vcgencmd", "measure_temp"]).decode()
                msg.temperature = float(out.split("=")[1].split("'")[0])

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
