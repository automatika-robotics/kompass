# Quick Start

Here we provide a quick recipe to get started with Kompass. The recipe is a single python script to build a complete navigation system. We will test the recipe in simulation. Lets first see how we can run the recipe and then we will go through it step by step.

## Run the recipe

To quickly launch the [recipe](https://github.com/automatika-robotics/kompass-ros/tree/dev/kompass_ros/recipes) using Kompass we use a simulation of the [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#notices) robot in [Webots](https://github.com/cyberbotics/webots_ros2) simulator.

- Install turtlebot3 and webots ROS2 packages:

```shell
sudo apt install ros-${ROS_DISTRO}-webots-ros2 ros-${ROS_DISTRO}-rviz2
```

- Install map server and localization packages:

```shell
sudo apt install ros-${ROS_DISTRO}-nav2-map-server ros-${ROS_DISTRO}-robot-localization
```

- Launch the simulation:

```shell
ros2 launch kompass webots_turtlebot3.launch.py
```

This will start webots simulator, Rviz and the robot localization and map server:

:::{figure-md} fig-webots

<img src="_static/images/webots_turtlebot3.png" alt="Webots Tutrlebot3 Simulation" width="700px">

Webots Tutrlebot3 Simulation
:::

:::{figure-md} fig-rviz
<img src="_static/images/rviz_webots_turtlebot3.png" alt="Rviz" width="700px">

Rviz
:::

- Open a new terminal and launch our recipe:

```shell
ros2 run kompass turtlebot3_test
```


```{include} tutorials/point_navigation.md
```
