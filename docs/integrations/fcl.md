# FCL (Flexible Collision Library)

**High-speed geometric collision detection.**


[FCL](https://github.com/flexible-collision-library/fcl) is a generic library for performing proximity and collision queries on geometric models.

Kompass leverages FCL to perform precise collision checks between the robot's kinematic model and both static (map) and dynamic (sensor) obstacles during path planning and control.

## Wrappers

Kompass provides bindings to use FCL directly within your Python or C++ components.

::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} {material-regular}`code;1.5em;sd-text-primary` C++ Wrapper
:link: https://github.com/automatika-robotics/kompass-core/tree/main/src/kompass_cpp
:class-card: sugar-card

**Performance**
Low-level C++ bindings in `kompass_cpp`. Used internally by the Controller for high-frequency collision checks.
:::

:::{grid-item-card} {material-regular}`integration_instructions;1.5em;sd-text-primary` Python Wrapper
:link: https://github.com/automatika-robotics/kompass-core/tree/main/src/kompass_core
:class-card: sugar-card

**Ease of Use**
High-level Python bindings in `kompass_core`. Perfect for custom scripting, testing scenarios, or rapid prototyping.
:::
::::
