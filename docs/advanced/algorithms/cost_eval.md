# Trajectory Cost Evaluation

Kompass includes built-in cost evaluation functions (implemented in [kompass_cpp](https://github.com/automatika-robotics/kompass-navigation/tree/dev/src/kompass_cpp)) and supports running custom cost evaluation functions to calculate the overall cost of a trajectory during navigation.

```{list-table}
:widths: 20 80
:header-rows: 1
* - Cost
  - Description

* - **Reference Path**
  - Measured using the average distance between the trajectory under evaluation and the closest reference (global) path segment. Added for favoring trajectories that remain close to the reference

* - **Goal Destination**
  - Evaluated based on the distance between the end of the trajectory and the final destination point of the navigation. Added for favoring trajectories that lead the robot closer to the goal point

* - **Obstacles Min-Distance**
  - Evaluated using the minimum distance between the trajectory points and the surrounding obstacles. Supports information from direct sensor data (LaserScan and PointCloud). Added for favoring trajectories that keep the robot away from nearby obstacles

* - **Smoothness**
  - Based on the average change in velocity along the trajectory under evaluation

* - **Jerk**
  - Based on the average change in acceleration along the trajectory under evaluation
```

## Costs Weights

The total cost of a trajectory is computed as the sum of all the default costs weighed by their respective cost weights.

```{list-table}
:widths: 10 10 10 70
:header-rows: 1
* - Name
  - Type
  - Default
  - Description

* - reference_path_distance_weight
  - `float`
  - `3.0`
  - Weight of the reference path cost. Must be between `0.0` and `1e3`.

* - goal_distance_weight
  - `float`
  - `3.0`
  - Weight of the goal position cost. Must be between `0.0` and `1e3`.
* - obstacles_distance_weight
  - `float`
  - `1.0`
  - Weight of the obstacles distance cost. Must be between `0.0` and `1e3`.
* - smoothness_weight
  - `float`
  - `0.0`
  - Weight of the trajectory smoothness cost. Must be between `0.0` and `1e3`.
* - jerk_weight
  - `float`
  - `0.0`
  - Weight of the trajectory jerk cost. Must be between `0.0` and `1e3`.

```

:::{tip} Set a cost weight to 0.0 to exclude that cost and not take it into consideration
:::
