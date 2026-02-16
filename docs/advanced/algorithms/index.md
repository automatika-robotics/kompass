# Control Algorithms

**Modular controllers for precise movement, tracking, and avoidance.**

Kompass provides a suite of battle-tested control algorithms. These range from classic geometric path-followers to modern, GPU-accelerated planners that handle dynamic obstacle avoidance or vision target following in real-time.


## Algorithm Suite

::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} {material-regular}`grid_on;1.5em;sd-text-primary` DWA
:link: dwa
:link-type: doc
:class-card: sugar-card

**Velocity Space Planning**
A real-time, GPU-accelerated planner that samples reachable velocities to balance goal progress and obstacle clearance.
:::

:::{grid-item-card} {material-regular}`timeline;1.5em;sd-text-primary` Pure Pursuit
:link: pure_pursuit
:link-type: doc
:class-card: sugar-card

**Geometric Path Tracking**
Calculates curvature to reach a lookahead point. Includes a reactive search layer to deviate for obstacles.
:::

:::{grid-item-card} {material-regular}`call_split;1.5em;sd-text-primary` Stanley Steering
:link: stanley
:link-type: doc
:class-card: sugar-card

**Axle-Based Feedback**
Optimized for Ackermann platforms, providing smooth exponential convergence to a reference path.
:::

:::{grid-item-card} {material-regular}`bubble_chart;1.5em;sd-text-primary` DVZ
:link: dvz
:link-type: doc
:class-card: sugar-card

**Reactive "Bubble" Control**
A highly efficient method that models safety zones as deformable perimeters for rapid avoidance.
:::

:::{grid-item-card} {material-regular}`videocam;1.5em;sd-text-primary` Vision Follower (RGB)
:link: vision_follower_rgb
:link-type: doc
:class-card: sugar-card

**2D Visual Servoing**
Maintains target centering and relative scale using standard monocular cameras.
:::

:::{grid-item-card} {material-regular}`blur_on;1.5em;sd-text-primary` Vision Follower (RGB-D)
:link: vision_follower_rgbd
:link-type: doc
:class-card: sugar-card

**3D Target Tracking**
Uses depth data and DWA-style planning to follow targets while actively avoiding obstacles.
:::
::::



## Kinematic Compatibility

Every algorithm in the Kompass stack is natively compatible with the three primary motion models. The internal logic automatically adapts to the specific constraints of your platform:



- **{material-regular}`directions_car` ACKERMANN**: Car-like platforms with steering constraints.
- **{material-regular}`adjust` DIFFERENTIAL_DRIVE**: Two-wheeled or skid-steer robots.
- **{material-regular}`open_with` OMNI**: Holonomic systems capable of lateral movement.



## Flexible Parameterization

Each algorithm is **fully parameterized**. Developers can tune behaviors such as lookahead gains, safety margins, and obstacle sensitivity directly through the Python API or YAML configuration. This modularity ensures the same code can run on a small indoor lab robot or a heavy-duty outdoor platform.




```{toctree}
:maxdepth: 1
:caption: Control Algorithms
:hidden:

pure_pursuit
dwa
stanley
dvz
vision_follower_rgb
vision_follower_rgbd
cost_eval

```
