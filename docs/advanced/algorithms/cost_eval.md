# Trajectory Cost Evaluation

**Scoring candidate paths for optimal selection.**

In sampling-based controllers like [DWA](dwa.md), dozens of candidate trajectories are generated at every time step. To choose the best one, Kompass uses a weighted sum of several cost functions.

The total cost $J$ for a given trajectory is calculated as:

$$
J = \sum (w_i \cdot C_i)
$$

Where $w_i$ is the configured weight and $C_i$ is the normalized cost value.

## Hardware Acceleration

To handle high-frequency control loops with large sample sets, Kompass leverages **SYCL** for massive parallelism.

::::{grid} 1 1 1 1
:gutter: 3

:::{grid-item-card} {material-regular}`speed;1.5em;sd-text-primary` SYCL Kernels
:link: ../benchmark
:link-type: doc
:class-card: sugar-card

**Vendor-Agnostic GPU Support**
Each cost function below is implemented as a specialized **SYCL kernel**. This allows the controller to evaluate thousands of trajectory points in parallel on **Nvidia, AMD, or Intel** GPUs, significantly reducing latency compared to CPU-only implementations.

See the performance gains in our Benchmarks →
:::
::::

## Built-in Cost Functions

Kompass includes optimized implementations for the following cost components:

| Cost Component | Description | Goal |
| :--- | :--- | :--- |
| **Reference Path** | Average distance between the candidate trajectory and the global reference path. | **Stay on track.** Keep the robot from drifting away from the global plan. |
| **Goal Destination** | Euclidean distance from the end of the trajectory to the final goal point. | **Make progress.** Favor trajectories that actually move the robot closer to the destination. |
| **Obstacle Distance**  | Inverse of the minimum distance to the nearest obstacle (from LaserScan/PointCloud). | **Stay safe.** Heavily penalize trajectories that come too close to walls or objects. |
| **Smoothness**| Average change in velocity (acceleration) along the trajectory. | **Drive smoothly.** Prevent jerky velocity changes. |
| **Jerk** | Average change in acceleration along the trajectory. | **Protect hardware.** Minimize mechanical stress and wheel slip. |


## Configuration Weights

You can tune the behavior of the robot by adjusting the weights ($w_i$) in your configuration.

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

:::{tip}
Setting a weight to `0.0` completely disables that specific cost calculation kernel, saving computational resources.
:::
