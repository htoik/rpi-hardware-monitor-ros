# Hardware Monitor ROS2 Node for Raspberry Pi 4

[![CI](https://github.com/htoik/rpi-hardware-monitor-ros/actions/workflows/ci.yml/badge.svg)](https://github.com/htoik/rpi-hardware-monitor-ros/actions/workflows/ci.yml)

This repository contains a hardware monitor node that provides statistics about hardware use such as CPU, disk and network utilization, power consumption and temperatures, which are published to the namespace `/hardware-status`.

## Installation

```bash
# (inside your project source directory)
git clone --recursive https://github.com/htoik/rpi-hardware-monitor-ros rpi-hardware-monitor-ros
cd rpi-hardware-monitor-ros

# conda (recommended)
conda env create -f conda/environment.yml
conda activate rpi-hardware-monitor-ros

colcon build --packages-select rpi-hardware-monitor-ros
```

## Installation (Developer)

```bash
. scripts/source.sh
colcon build --symlink-install --packages-select rpi-hardware-monitor-ros
```

## Configuration

The node can be configured to monitor specific hardware interfaces with additional parameters like the polling frequency. The default parameters can be modified directly in the configuration file in `rpi_hw_monitor/config/params.yml`. Alternatively, it is possible to pass a custom configuration file to the launcher directly. The config should have the following format:

```yaml
rpi_hw_monitor:
  ros__parameters:
    monitor_flags:
      cpu : true
      disk : true
      network : true
      power : true
      temperature : true
    polling_frequency: 5 # (Hz)
```

## Usage

```bash
ros2 run rpi_hw_monitor hw_monitor.py
ros2 launch rpi_hw_monitor hw_monitor_launch.py
```
