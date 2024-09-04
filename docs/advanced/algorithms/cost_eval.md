# Trajectory Cost Evaluation

Kompass includes built-in cost evaluation functions (implemented in [kompass_cpp](https://github.com/automatika-robotics/kompass-navigation/tree/dev/src/kompass_cpp)) and supports running custom cost evaluation functions to calculate the overall cost of a trajectory during navigation.

## Reference Path Cost

Measured using the average distance between the trajectory under evaluation and the closest reference (global) path segment. Added for favoring trajectories that remain close to the reference.

## Goal Destination Cost

Evaluated based on the distance between the end of the trajectory and the final destination point of the navigation. Added for favoring trajectories that lead the robot closer to the goal point.

## Obstacles Min-Distance Cost

Evaluated using the minimum distance between the trajectory points and the surrounding obstacles. Supports information from direct sensor data (LaserScan and PointCloud). Added for favoring trajectories that keep the robot away from nearby obstacles.

## Smoothness Cost

Based on the average change in velocity along the trajectory under evaluation.

## Jerk Cost

Based on the average change in acceleration along the trajectory under evaluation.