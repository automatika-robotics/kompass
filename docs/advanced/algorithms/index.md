# Control Algorithms

Kompass includes a set of modular navigation algorithms designed to work across a wide range of robotic platforms and environments:

- [Pure Pursuit](pure_pursuit.md) – A classic geometric path-tracking controller that calculates curvature to reach a lookahead point, featuring an integrated simple search layer for collision avoidance.

- [DWA (Dynamic Window Approach)](dwa.md) – A real-time velocity-space planner that dynamically adapts to changing environments while balancing a set of defined [cost goals](cost_eval.md) like goal-seeking, reference path following and obstacle avoidance.

- [Stanley Steering](stanley.md) – A robust path-tracking controller with smooth convergence, ideal for structured environments.

- [DVZ (Deformable Virtual Zone)](dvz.md) – A reactive local planner that dynamically adapts to nearby obstacles.

- [Vision Follower](vision_follower.md) – A vision-based object follower that enables robots to follow moving targets using onboard images (RGB or RGBD).

All algorithms are fully compatible with the the three primary robot motion models:

- ACKERMANN (e.g. car-like platforms)

- DIFFERENTIAL_DRIVE (e.g. skid-steer or two-wheeled robots)

- OMNI (e.g. holonomic systems)

Each algorithm is **easily parameterized**, allowing developers to tune behaviors such as lookahead distance, velocity limits, obstacle sensitivity, and control gains directly through configuration in the Python API. This makes it easy to adapt planning and control logic to the specific dynamics and operational goals of any robot—whether you're running in simulation or on real hardware.

```{toctree}
:maxdepth: 1
:caption: Control Algorithms
:hidden:

pure_pursuit
dwa
stanley
dvz
vision_follower
cost_eval
```
