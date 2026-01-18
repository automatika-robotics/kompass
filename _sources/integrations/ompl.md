# OMPL (Open Motion Planning Library)

[OMPL](https://github.com/ompl/ompl), the Open Motion Planning Library, is a generic C++ library for state-of-the-art motion planning algorithms. OMPL is compatible with different robot morphologies and planning spaces which make it widely used in robotics for motion planning problems both in 2D and 3D spaces. (for more details check the [official documentation of OMPL](https://ompl.kavrakilab.org/tutorials.html))

## OMPL with Kompass

Kompass provides Python bindings (Pybind11 bindings) for OMPL in its navigation core package (kompass_navigation). The bindings enable setting and solving a planning problem using:

- '[SE2StateSpace](https://ompl.kavrakilab.org/classompl_1_1base_1_1SE2StateSpace_1_1StateType.html#details)' which is convient for 2D motion planning. It provides an 'SE2State' consisting of a 2D position and rotation in the plane: ```SE(2): (x, y, yaw)```
- OMPL geometric planners (listed below)
- Built-in '[StateValidityChecker](https://ompl.kavrakilab.org/classompl_1_1base_1_1StateValidityChecker.html)' which implements a collision checker (using [FCL](fcl.md)) to insure finding collision free paths.

## Configuring OMPL in Kompass

The following is an example YAML configuration of OMPL. In the section [below](#planners-default-parameters) we list all the available configuration parameters for each of the planners along with their default values.

```yaml
ompl:
    log_level: 'WARN'
    planning_timeout: 10.0  # (secs) Return and consider the planning to fail if solving takes more than planning_timeout
    simplification_timeout: 0.01 # # (secs) abort path simplification if it takes more than simplification_timeout
    goal_tolerance: 0.01        # (meters) Distance to consider that the robot is at goal point, i.e. no planning is performed
    optimization_objective: 'PathLengthOptimizationObjective'
    planner_id: 'ompl.geometric.KPIECE1'
```

## Available algorithms from OMPL

The following geometric planners are supported in Kompass:

- [ABITstar](#abitstar)
- [AITstar](#aitstar)
- [BFMT](#bfmt)
- [BITstar](#bitstar)
- [BKPIECE1](#bkpiece1)
- [BiEST](#biest)
- [EST](#est)
- [FMT](#fmt)
- [InformedRRTstar](#informedrrtstar)
- [KPIECE1](#kpiece1)
- [LBKPIECE1](#lbkpiece1)
- [LBTRRT](#lbtrrt)
- [LazyLBTRRT](#lazylbtrrt)
- [LazyPRM](#lazyprm)
- [LazyPRMstar](#lazyprmstar)
- [LazyRRT](#lazyrrt)
- [PDST](#pdst)
- [PRM](#prm)
- [PRMstar](#prmstar)
- [ProjEST](#projest)
- [RRT](#rrt)
- [RRTConnect](#rrtconnect)
- [RRTXstatic](#rrtxstatic)
- [RRTsharp](#rrtsharp)
- [RRTstar](#rrtstar)
- [SBL](#sbl)
- [SST](#sst)
- [STRIDE](#stride)
- [TRRT](#trrt)

## Planners test results

A planning problem is simulated using the [Turtlebot3 Gazebo simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) Waffle map. The following table shows the planning results using each of the previous planners. The problem is simulated for 20 repetitions using each planners and the table below shows the average result values. 'solved' is set to False if in any repetition the method was unable to find a solution. The solution search timeout is set to 2seconds.


:::{tip} To check/run the test script and the associated test resources refer to [kompass_navigation ompl test](https://github.com/automatika-robotics/kompass-navigation/tree/dev/tests)
:::

method | solved | solution time (s) | solution length (m) | simplification time (s) | Conversion/Publishing ROS2 (s)
---|---|---|---|---|---
ABITstar | True | 1.071 | 2.948 | 0.0075 | 0.00056
BFMT | True | 0.1128 | 3.487 | 0.0066 | 0.00017
BITstar | True | 1.0732 | 2.962 | 0.0061 | 0.00015
BKPIECE1 | True | 0.0703 | 4.469 | 0.01777 | 0.00019
BiEST | True | 0.06189 | 4.418 | 0.01078 | 0.00014
EST | True | 0.0640 | 4.059 | 0.01074 | 0.00013
FMT | True | 0.1330 | 3.6277 | 0.00630 | 0.00016
InformedRRTstar | True | 1.0679 | 2.962 | 0.00457 | 0.00013
KPIECE1 | True | 0.0676 | 5.439 | 0.0148 | 0.00017
LBKPIECE1 | True | 0.07543 | 5.174 | 0.02 | 0.00017
LBTRRT | True | 1.0696 | 3.221 | 0.005 | 0.00012
LazyLBTRRT | True | 1.0672 | 3.305 | 0.0053 | 0.00013
LazyPRM | False | 1.0811 | 0.0 | 0.0 | 0.0
LazyPRMstar | True | 1.0701 | 3.03 | 0.0063 | 0.00012
LazyRRT | True | 0.0982 | 4.5196 | 0.016 | 0.00013
PDST | True | 0.06781 | 3.836 | 0.009 | 0.00013
PRM | True | 1.0672 | 3.306 | 0.0068 | 0.00012
PRMstar | True | 1.0740 | 3.72 | 0.0085 | 0.00012
ProjEST | True | 0.0683 | 4.19 | 0.0082 | 0.00013
RRT | True | 0.09062 | 4.86 | 0.019 | 0.00014
RRTConnect | True | 0.07535 | 4.78 | 0.014 | 0.00014
RRTXstatic | True | 1.0712 | 3.03 | 0.0041 | 0.00012
RRTsharp | True | 1.0680 | 3.01 | 0.0052 | 0.00013
RRTstar | True | 1.0672 | 2.96 | 0.0042 | 0.00013
SBL | True | 0.0800 | 4.039 | 0.0121 | 0.00015
SST | True | 1.0681 | 2.63 | 0.0012 | 0.00018
STRIDE | True | 0.0679 | 4.12 | 0.0098 | 0.00015
TRRT | True | 0.0798 | 4.11 | 0.01094 | 0.00015

## Planners Default Parameters

### ABITstar
*ABITstar Default Parameters:*

- delay_rewiring_to_first_solution: False
- drop_unconnected_samples_on_prune: False
- find_approximate_solutions: False
- inflation_scaling_parameter: 10.0
- initial_inflation_factor: 1000000.0
- prune_threshold_as_fractional_cost_change: 0.05
- rewire_factor: 1.1
- samples_per_batch: 100
- stop_on_each_solution_improvement: False
- truncation_scaling_parameter: 5.0
- use_graph_pruning: True
- use_just_in_time_sampling: False
- use_k_nearest: True
- use_strict_queue_ordering: True

### AITstar
*AITstar Default Parameters:*

- find_approximate_solutions: True
- rewire_factor: 1.0
- samples_per_batch: 100
- use_graph_pruning: True
- use_k_nearest: True

### BFMT
*BFMT Default Parameters:*

- balanced: False
- cache_cc: True
- extended_fmt: True
- heuristics: True
- nearest_k: True
- num_samples: 1000
- optimality: True
- radius_multiplier: 1.0

### BITstar
*BITstar Default Parameters:*

- delay_rewiring_to_first_solution: False
- drop_unconnected_samples_on_prune: False
- find_approximate_solutions: False
- prune_threshold_as_fractional_cost_change: 0.05
- rewire_factor: 1.1
- samples_per_batch: 100
- stop_on_each_solution_improvement: False
- use_graph_pruning: True
- use_just_in_time_sampling: False
- use_k_nearest: True
- use_strict_queue_ordering: True

### BKPIECE1
*BKPIECE1 Default Parameters:*

- border_fraction: 0.9
- range: 0.0

### BiEST
*BiEST Default Parameters:*

- range: 0.0

### EST
*EST Default Parameters:*

- goal_bias: 0.5
- range: 0.0

### FMT
*FMT Default Parameters:*

- cache_cc: True
- extended_fmt: True
- heuristics: False
- num_samples: 1000
- radius_multiplier: 1.1
- use_k_nearest: True

### InformedRRTstar
*InformedRRTstar Default Parameters:*

- delay_collision_checking: True
- goal_bias: 0.05
- number_sampling_attempts: 100
- ordered_sampling: False
- ordering_batch_size: 1
- prune_threshold: 0.05
- range: 0.0
- rewire_factor: 1.1
- use_k_nearest: True

### KPIECE1
*KPIECE1 Default Parameters:*

- border_fraction: 0.9
- goal_bias: 0.05
- range: 0.0

### LBKPIECE1
*LBKPIECE1 Default Parameters:*

- border_fraction: 0.9
- range: 0.0

### LBTRRT
*LBTRRT Default Parameters:*

- epsilon: 0.4
- goal_bias: 0.05
- range: 0.0

### LazyLBTRRT
*LazyLBTRRT Default Parameters:*

- epsilon: 0.4
- goal_bias: 0.05
- range: 0.0

### LazyPRM
*LazyPRM Default Parameters:*

- max_nearest_neighbors: 8
- range: 0.0

### LazyPRMstar
*LazyPRMstar Default Parameters:*


### LazyRRT
*LazyRRT Default Parameters:*

- goal_bias: 0.05
- range: 0.0

### PDST
*PDST Default Parameters:*

- goal_bias: 0.05

### PRM
*PRM Default Parameters:*

- max_nearest_neighbors: 8

### PRMstar
*PRMstar Default Parameters:*


### ProjEST
*ProjEST Default Parameters:*

- goal_bias: 0.05
- range: 0.0

### RRT
*RRT Default Parameters:*

- goal_bias: 0.05
- intermediate_states: False
- range: 0.0

### RRTConnect
*RRTConnect Default Parameters:*

- intermediate_states: False
- range: 0.0

### RRTXstatic
*RRTXstatic Default Parameters:*

- epsilon: 0.0
- goal_bias: 0.05
- informed_sampling: False
- number_sampling_attempts: 100
- range: 0.0
- rejection_variant: 0
- rejection_variant_alpha: 1.0
- rewire_factor: 1.1
- sample_rejection: False
- update_children: True
- use_k_nearest: True

### RRTsharp
*RRTsharp Default Parameters:*

- goal_bias: 0.05
- informed_sampling: False
- number_sampling_attempts: 100
- range: 0.0
- rejection_variant: 0
- rejection_variant_alpha: 1.0
- rewire_factor: 1.1
- sample_rejection: False
- update_children: True
- use_k_nearest: True

### RRTstar
*RRTstar Default Parameters:*

- delay_collision_checking: True
- focus_search: False
- goal_bias: 0.05
- informed_sampling: True
- new_state_rejection: False
- number_sampling_attempts: 100
- ordered_sampling: False
- ordering_batch_size: 1
- prune_threshold: 0.05
- pruned_measure: False
- range: 0.0
- rewire_factor: 1.1
- sample_rejection: False
- tree_pruning: False
- use_admissible_heuristic: True
- use_k_nearest: True

### SBL
*SBL Default Parameters:*

- range: 0.0

### SST
*SST Default Parameters:*

- goal_bias: 0.05
- pruning_radius: 3.0
- range: 5.0
- selection_radius: 5.0

### STRIDE
*STRIDE Default Parameters:*

- degree: 16
- estimated_dimension: 3.0
- goal_bias: 0.05
- max_degree: 18
- max_pts_per_leaf: 6
- min_degree: 12
- min_valid_path_fraction: 0.2
- range: 0.0
- use_projected_distance: False

### TRRT
*TRRT Default Parameters:*

- goal_bias: 0.05
- range: 0.0
- temp_change_factor: 0.1
