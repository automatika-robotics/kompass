# Basic Navigation Patterns

The following tutorials provide hands-on recipes for implementing common robotic navigation behaviors. These patterns are production-ready and designed to be adapted to your specific robot platform.

## Navigation Recipes

Choose a pattern to start building your robot's capabilities.

::::{grid} 1 2 2 3
:gutter: 3

:::{grid-item-card} {material-regular}`map;1.5em;sd-text-primary` Point-to-Point
:link: point_navigation
:link-type: doc
:class-card: sugar-card

**Core Navigation**

The "Hello World" of robotics. Define goal coordinates, launch the planner, and execute a path on a static map.
:::

:::{grid-item-card} {material-regular}`videocam;1.5em;sd-text-primary` RGB Tracking
:link: vision_tracking
:link-type: doc
:class-card: sugar-card

**Visual Servoing**

Use standard computer vision (color blobs or ML object detection) to steer the robot toward a moving target.
:::

:::{grid-item-card} {material-regular}`sensors;1.5em;sd-text-primary` Depth Tracking
:link: vision_tracking_depth
:link-type: doc
:class-card: sugar-card

**RGB-D Following**

Upgrade your logic to maintain a specific distance from the target, enabling robust "stop-and-go" behavior.
:::
::::

<br>

## Advanced Topics

Ready to customize your stack or make it adaptive?

::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} {material-regular}`tune;1.5em;sd-text-primary` Component Configuration
:link: configuration
:link-type: doc
:class-card: sugar-card

Configure your navigation components using Python, YAML, TOML, or JSON to match your specific hardware.
:::

:::{grid-item-card} {material-regular}`flash_on;1.5em;sd-text-primary` Event-Driven Navigation
:link: events_index
:link-type: doc
:class-card: sugar-card

Learn how to make your robot adapt to changing environments dynamically without writing custom code or complex state machines.
:::
::::


```{toctree}
:maxdepth: 1
:caption: Basic Navigation Patterns
:hidden:

point_navigation
vision_tracking
vision_tracking_depth
configuration
```
