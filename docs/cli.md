# Kompass CLI

**Inspect core algorithms and hardware support.**

The Kompass CLI provides a quick way to inspect available algorithms, view default parameters, and check for compatible hardware accelerators (GPUs) directly from the terminal without launching the full stack.

## Command Reference

### Algorithm Inspection

Commands to query the available Planner and Controller plugins.

| Command | Description |
| :--- | :--- |
| **`controller list`** | List all available **control** algorithms (e.g., `DWA`, `Stanley`). |
| **`controller params <algo>`** | Show the default configuration parameters for a specific control algorithm. |
| **`planner list`** | List all available **planning** algorithms (e.g., `RRTstar`, `PRM`). |
| **`planner params <algo>`** | Show the default configuration parameters for a specific planning algorithm. |

### System & Hardware

Commands to check the host system capabilities.

| Command | Description |
| :--- | :--- |
| **`accelerators_support`** | List all **SYCL-compatible** devices (GPUs/CPUs) detected on the system. Useful for verifying if the [Local Mapper](./navigation/mapper.md) can use hardware acceleration. |
| **`info`** | Display generic CLI usage examples and links to documentation. |

## Usage Examples

Run these commands using the standard `ros2 run` syntax:

```bash
# 1. View Help
ros2 run kompass cli --help

# 2. Check for GPU Support
ros2 run kompass cli accelerators_support

# 3. List available Controllers
ros2 run kompass cli controller list

# 4. Inspect DWA Parameters
ros2 run kompass cli controller params DWA

# 5. Inspect AITstar Planner Parameters
ros2 run kompass cli planner params AITstar

```
