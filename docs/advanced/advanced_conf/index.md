# Advanced Configurations

**Fine-tune component streams and communication.**

Kompass provides advanced options for developers who need precise control over how components connect and communicate. Whether you are integrating custom hardware, optimizing for lossy networks, or ensuring strict data types, these guides help you go beyond the defaults.


::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} {material-regular}`swap_horiz;1.5em;sd-text-primary` Inputs & Outputs
:link: topics
:link-type: doc
:class-card: sugar-card

**Stream Management**
Learn how Kompass enforces strict interfaces to "lock" component functionality.
<br>
* Define and validate input/output types.
* Remap standard stream keys (e.g., `sensor_data`).
* Configure **Multi-Topic Fusion** (e.g., fusing LiDAR + Radar).
:::

:::{grid-item-card} {material-regular}`settings_input_antenna;1.5em;sd-text-primary` QoS Configuration
:link: qos
:link-type: doc
:class-card: sugar-card

**Communication Policies**
Fine-tune the reliability and persistence of your data streams using ROS 2 Quality of Service.
<br>
* **Reliability:** *Best Effort* (Sensor speed) vs. *Reliable* (Control safety).
* **Durability:** *Volatile* (Live data) vs. *Transient Local* (Static maps).
:::
::::



```{toctree}
:maxdepth: 2
:caption: Advanced Configurations
:hidden:

topics
qos

```
