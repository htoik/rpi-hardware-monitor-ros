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

## Usage

```bash
# TODO
```
