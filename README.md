# ROS2 Hardware Monitor Node for RPi4

[![Latest Version](https://img.shields.io/github/release/htoik/rpi-hardware-monitor-ros.svg?style=flat-square&logo=GitHub)](https://github.com/htoik/rpi-hardware-monitor-ros/releases)
[![Build Status](https://img.shields.io/github/actions/workflow/status/htoik/rpi-hardware-monitor-ros/ci.yml?label=ci%20build&style=flat-square&logo=GitHub)](https://github.com/htoik/rpi-hardware-monitor-ros/actions/workflows/ci.yml)

This repository contains a hardware monitor node that provides statistics about hardware use such as CPU, RAM, disk and network utilization, power consumption and temperatures, which are published to the namespace `/hw_status`.

The HardwareStatus message type has the following structure:

```
float32 cpu_usage
float32 memory_mb
float32 disk_rx_mb
float32 disk_tx_mb
float32 network_rx_mb
float32 network_tx_mb
float32 power_consumption
float32 temperature
```

## Configuration

The node can be configured to monitor specific hardware interfaces with additional parameters like the polling frequency. The default parameters can be modified directly in the configuration file in `rpi_hw_monitor/config/params.yml`. Alternatively, it is possible to pass a custom configuration file to the launcher directly. The config should have the following format:

```yaml
rpi_hw_monitor:
  ros__parameters:
    polling_frequency: 5 # (Hz)
    cpu : true
    memory: true
    disk : true
    network : true
    power : true
    temperature : true
```

## Usage

```bash
ros2 run rpi_hw_monitor hw_monitor.py
ros2 launch rpi_hw_monitor hw_monitor_launch.py
```

## Installation

```bash
# (inside your project source directory)
git clone --recursive https://github.com/htoik/rpi-hardware-monitor-ros rpi-hardware-monitor-ros
cd rpi-hardware-monitor-ros

colcon build --packages-select rpi_hw_monitor
```

## Installation (Developer)

```bash
conda env create -f scripts/conda-env.yml
conda activate rpi-hardware-monitor-ros

. scripts/source.sh
colcon build --symlink-install --packages-select rpi_hw_monitor
```

## Todo

- gpu stats

- cpu frequency, voltage

- load average

- swap memory usage

- connected devices

- no. processes
