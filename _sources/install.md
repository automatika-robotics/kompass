# Installation

## Prerequisites

Kompass is built to be used with ROS2. All ROS2 distributions starting from _Foxy_ upto _Rolling_ are supported. Install ROS2 version of your choice by following the instructions on the [official site](https://docs.ros.org/).

## Install kompass-core

kompass-core is a python package that provides highly optimized implementations of planning and control algorithms for Kompass. You can install it in the following ways:

### With GPU support (Recommended):

On any Ubuntu (including Jetpack) based machine, you can simply run the following:

`curl https://raw.githubusercontent.com/automatika-robotics/kompass-core/refs/heads/main/build_dependencies/install_gpu.sh | bash`

This script will install all relevant dependencies, including [AdaptiveCPP](https://github.com/AdaptiveCpp/AdaptiveCpp) and install the latest version of kompass-core from source. It is good practice to read the [script](https://github.com/automatika-robotics/kompass-core/blob/main/build_dependencies/install_gpu.sh) first.

### Installing with pip

Install kompass-core as follows:

`pip install kompass-core`

## Install Kompass

Install pre-built Kompass binary as follows:

`sudo apt install ros-$ROS_DISTRO-kompass`

Alternatively, grab deb packages (for kompass_interfaces and kompass) for your favourite distro from the [release page](https://github.com/automatika-robotics/kompass/releases) and install them as follows:

```bash
sudo dpkg -i ros-$ROS_DISTRO-kompass-interfaces_$version$DISTRO_$ARCHITECTURE.deb
sudo dpkg -i ros-$ROS_DISTRO-kompass_$version$DISTRO_$ARCHITECTURE.deb
```

### Build Kompass from source

You can build Kompass from source as follows:

```shell
mkdir -p kompass_ws/src
cd kompass_ws/src
git clone https://github.com/automatika-robotics/sugarcoat
git clone https://github.com/automatika-robotics/kompass
rosdep update
rosdep install -y --from-paths . --ignore-src
cd ..
colcon build
```
