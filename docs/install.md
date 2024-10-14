# Installation

## Prerequisites

Kompass is built to be used with ROS2. All ROS2 distributions starting from _Foxy_ upto _Rolling_ are supported. Install ROS2 version of your choice by following the instructions on the [official site](https://docs.ros.org/).

## Instal Kompass (Ubuntu)

**Binary packages for Ubuntu will be released soon. Check this space.**

## Build Kompass from source

- Kompass Core provides optimized implementation of planning and control algorithms for Kompass. To install it see instructions [here](https://github.com/automatika-robotics/kompass-core).

- Finally, build ROS Sugar and Kompass from source as shown below:

```shell
mkdir -p kompass_ws/src
cd kompass_ws
git clone https://github.com/automatika-robotics/ros-sugar
git clone https://github.com/automatika-robotics/kompass-ros
colcon build
```
