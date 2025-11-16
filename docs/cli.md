# Kompass CLI

The Kompass CLI provides an easy way to quickly inspect Kompass core algorithms and configurations from ROS2 command-line.

## CLI Commands

| Command                         | Description                                            |
| ------------------------------- | ------------------------------------------------------ |
| `controller list`               | List all available control algorithms.                 |
| `controller params <algorithm>` | Show default parameters for a control algorithm.       |
| `planner list`                  | List all available planning algorithms.                |
| `planner params <algorithm>`    | Show default parameters for a planning algorithm.      |
| `accelerators_support`          | List SYCL-compatible GPU accelerators on the system.   |
| `info`                          | Display CLI usage examples and links to documentation. |


## Usage Examples

```bash
ros2 run kompass cli --help
ros2 run kompass cli controller list
ros2 run kompass cli controller params DWA
ros2 run kompass cli planner list
ros2 run kompass cli planner params AITstar
ros2 run kompass cli accelerators_support
ros2 run kompass cli info
```
