# Installation

## Prerequisites

:::{admonition} *ROS2* Required
:class: note
Kompass supports all *ROS2* distributions from **Foxy** up to **Rolling**.
Please ensure you have a working [ROS2 installation](https://docs.ros.org/) before proceeding.
:::

<span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">Install Core Engine</span>

[`kompass-core`](https://github.com/automatika-robotics/kompass-core) is the package providing the highly optimized implementations of planning and control algorithms for Kompass. Choose the installation method that matches your hardware needs.

::::{tab-set}

:::{tab-item} {material-regular}`speed;1.2em;sd-text-primary` GPU Support (Recommended)
:sync: gpu
<br/>

**Best for production robots and high-performance simulation.**

This installs [AdaptiveCPP](https://github.com/AdaptiveCpp/AdaptiveCpp) and compiles the core engine from source to fully utilize your specific GPU (Nvidia, AMD, Intel, etc.).

Run the following on any Ubuntu-based machine (including Jetson):

```bash
curl [https://raw.githubusercontent.com/automatika-robotics/kompass-core/refs/heads/main/build_dependencies/install_gpu.sh](https://raw.githubusercontent.com/automatika-robotics/kompass-core/refs/heads/main/build_dependencies/install_gpu.sh) | bash

```

*It is good practice to [read the script](https://github.com/automatika-robotics/kompass-core/blob/main/build_dependencies/install_gpu.sh) before running it.*
:::

:::{tab-item} {material-regular}`terminal;1.2em;sd-text-primary` Standard (CPU)
:sync: cpu
<br/>

**Best for quick testing or lightweight environments.**

Installs the standard Python package along with the CPP package bindings via `pip`.

```bash
pip install kompass-core

```

:::

::::

## Install Kompass

Once the core engine is ready, install Kompass ROS2 packages (`kompass` and `kompass_interfaces`).

::::{tab-set}

:::{tab-item} {material-regular}`inventory_2;1.2em;sd-text-primary` Binary
:sync: binary
<br/>

**Option A: Install via APT (Recommended)**

```bash
sudo apt install ros-$ROS_DISTRO-kompass

```

**Option B: Manual `.deb` Install**
Download the specific version for your architecture from [GitHub Releases](https://github.com/automatika-robotics/kompass/releases).

```bash
# Replace variables with your version/distro/arch
sudo dpkg -i ros-$ROS_DISTRO-kompass-interfaces_$version$DISTRO_$ARCHITECTURE.deb
sudo dpkg -i ros-$ROS_DISTRO-kompass_$version$DISTRO_$ARCHITECTURE.deb

```

:::

:::{tab-item} {material-regular}`build;1.2em;sd-text-primary` Source (Advanced)
:sync: source
<br/>

**Best for contributors or debugging.**

```shell
# 1. Create workspace
mkdir -p kompass_ws/src
cd kompass_ws/src

# 2. Clone repositories
git clone [https://github.com/automatika-robotics/sugarcoat](https://github.com/automatika-robotics/sugarcoat)
git clone [https://github.com/automatika-robotics/kompass](https://github.com/automatika-robotics/kompass)

# 3. Install dependencies
rosdep update
rosdep install -y --from-paths . --ignore-src

# 4. Build
cd ..
colcon build
source install/setup.bash

```

:::

::::

## Next Steps

Now that you have Kompass installed, verify your setup or jump straight into the tutorials.

::::{grid} 1 2 2 2
:gutter: 2

:::{grid-item-card} {material-regular}`rocket_launch;1.2em;sd-text-primary` Quick Start
:link: tutorials/quick_start
:link-type: doc
:class-card: sugar-card

Run your first navigation task in simulation
:::

:::{grid-item-card} {material-regular}`menu_book;1.2em;sd-text-primary` Tutorials
:link: tutorials/index
:link-type: doc
:class-card: sugar-card

Learn how to create custom navigation capabilities with Kompass
:::
::::
