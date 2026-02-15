# OMPL (Open Motion Planning Library)

**State-of-the-art sampling-based motion planning.**

[OMPL (Open Motion Planning Library)](https://github.com/ompl/ompl) is the industry standard for generic motion planning algorithms. Kompass integrates OMPL to solve complex planning problems in 2D and 3D spaces for various robot morphologies.


## Kompass Bindings

Kompass provides efficient Nanobind bindings in the `kompass_navigation` package, bridging your Python components with OMPL's C++ core.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`architecture;1.5em;sd-text-primary` Geometric Planners - </span> **Algorithm Suite**. Direct access to over 20 geometric planners (RRT*, PRM, EST, etc.) configurable in your recipe or via a configuration file (YAML, TOML, JSON).

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`rotate_90_degrees_ccw;1.5em;sd-text-primary` SE2 State Space</span> **2D Navigation**. It provides an ['SE2State'](https://ompl.kavrakilab.org/classompl_1_1base_1_1SE2StateSpace.html) consisting of a 2D position and rotation in the plane: ```SE(2): (x, y, yaw)```

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`check_box;1.5em;sd-text-primary` Validity Checker - </span> **Collision Checking**. Built-in '[StateValidityChecker](https://ompl.kavrakilab.org/classompl_1_1base_1_1StateValidityChecker.html)' which implements a collision checker (using [FCL](fcl.md)) to insure finding collision free paths.



## Configuration

Configure OMPL behavior via your configuration file.

```yaml
ompl:
    # Verbosity
    log_level: 'WARN'

    # Timeouts
    planning_timeout: 10.0      # (s) Max time to search for a solution
    simplification_timeout: 0.01 # (s) Max time to optimize the path

    # Constraints
    goal_tolerance: 0.01        # (m) Goal reached threshold

    # Optimization
    optimization_objective: 'PathLengthOptimizationObjective'

    # Active Algorithm
    planner_id: 'ompl.geometric.KPIECE1'

```

## Planner Benchmarks

Planning performance simulated on a **Turtlebot3 Waffle** in Gazebo.

* **Repetitions:** 20 per planner
* **Timeout:** 2.0 seconds
* **Metric:** Average values

:::{tip}
To run this benchmark yourself, refer to the [kompass_navigation tests](https://github.com/automatika-robotics/kompass-navigation/tree/dev/tests).
:::

```{list-table}
:header-rows: 1
:widths: 15 10 15 15 15 20

* - Method
  - Solved
  - Time (s)
  - Length (m)
  - Simplify (s)
  - ROS 2 Pub (s)

* - **ABITstar**
  - ✅
  - 1.071
  - 2.948
  - 0.0075
  - 0.00056

* - **BFMT**
  - ✅
  - 0.1128
  - 3.487
  - 0.0066
  - 0.00017

* - **BITstar**
  - ✅
  - 1.0732
  - 2.962
  - 0.0061
  - 0.00015

* - **BKPIECE1**
  - ✅
  - 0.0703
  - 4.469
  - 0.0178
  - 0.00019

* - **BiEST**
  - ✅
  - 0.0619
  - 4.418
  - 0.0108
  - 0.00014

* - **EST**
  - ✅
  - 0.0640
  - 4.059
  - 0.0107
  - 0.00013

* - **FMT**
  - ✅
  - 0.1330
  - 3.628
  - 0.0063
  - 0.00016

* - **InformedRRT***
  - ✅
  - 1.0679
  - 2.962
  - 0.0046
  - 0.00013

* - **KPIECE1**
  - ✅
  - 0.0676
  - 5.439
  - 0.0148
  - 0.00017

* - **LBKPIECE1**
  - ✅
  - 0.0754
  - 5.174
  - 0.0200
  - 0.00017

* - **LBTRRT**
  - ✅
  - 1.0696
  - 3.221
  - 0.0050
  - 0.00012

* - **LazyLBTRRT**
  - ✅
  - 1.0672
  - 3.305
  - 0.0053
  - 0.00013

* - **LazyPRM**
  - ❌
  - 1.0811
  - 0.0
  - 0.0
  - 0.0

* - **LazyPRMstar**
  - ✅
  - 1.0701
  - 3.030
  - 0.0063
  - 0.00012

* - **LazyRRT**
  - ✅
  - 0.0982
  - 4.520
  - 0.0160
  - 0.00013

* - **PDST**
  - ✅
  - 0.0678
  - 3.836
  - 0.0090
  - 0.00013

* - **PRM**
  - ✅
  - 1.0672
  - 3.306
  - 0.0068
  - 0.00012

* - **PRMstar**
  - ✅
  - 1.0740
  - 3.720
  - 0.0085
  - 0.00012

* - **ProjEST**
  - ✅
  - 0.0683
  - 4.190
  - 0.0082
  - 0.00013

* - **RRT**
  - ✅
  - 0.0906
  - 4.860
  - 0.0190
  - 0.00014

* - **RRTConnect**
  - ✅
  - 0.0754
  - 4.780
  - 0.0140
  - 0.00014

* - **RRTXstatic**
  - ✅
  - 1.0712
  - 3.030
  - 0.0041
  - 0.00012

* - **RRTsharp**
  - ✅
  - 1.0680
  - 3.010
  - 0.0052
  - 0.00013

* - **RRTstar**
  - ✅
  - 1.0672
  - 2.960
  - 0.0042
  - 0.00013

* - **SBL**
  - ✅
  - 0.0800
  - 4.039
  - 0.0121
  - 0.00015

* - **SST**
  - ✅
  - 1.0681
  - 2.630
  - 0.0012
  - 0.00018

* - **STRIDE**
  - ✅
  - 0.0679
  - 4.120
  - 0.0098
  - 0.00015

* - **TRRT**
  - ✅
  - 0.0798
  - 4.110
  - 0.0109
  - 0.00015

```

## Supported Planners & Parameters

Click on a planner to view its default configuration parameters.

:::{dropdown} ABITstar

* `delay_rewiring_to_first_solution`: False
* `drop_unconnected_samples_on_prune`: False
* `find_approximate_solutions`: False
* `inflation_scaling_parameter`: 10.0
* `initial_inflation_factor`: 1000000.0
* `prune_threshold_as_fractional_cost_change`: 0.05
* `rewire_factor`: 1.1
* `samples_per_batch`: 100
* `stop_on_each_solution_improvement`: False
* `truncation_scaling_parameter`: 5.0
* `use_graph_pruning`: True
* `use_just_in_time_sampling`: False
* `use_k_nearest`: True
* `use_strict_queue_ordering`: True
:::

:::{dropdown} AITstar

* `find_approximate_solutions`: True
* `rewire_factor`: 1.0
* `samples_per_batch`: 100
* `use_graph_pruning`: True
* `use_k_nearest`: True
:::

:::{dropdown} BFMT

* `balanced`: False
* `cache_cc`: True
* `extended_fmt`: True
* `heuristics`: True
* `nearest_k`: True
* `num_samples`: 1000
* `optimality`: True
* `radius_multiplier`: 1.0
:::

:::{dropdown} BITstar

* `delay_rewiring_to_first_solution`: False
* `drop_unconnected_samples_on_prune`: False
* `find_approximate_solutions`: False
* `prune_threshold_as_fractional_cost_change`: 0.05
* `rewire_factor`: 1.1
* `samples_per_batch`: 100
* `stop_on_each_solution_improvement`: False
* `use_graph_pruning`: True
* `use_just_in_time_sampling`: False
* `use_k_nearest`: True
* `use_strict_queue_ordering`: True
:::

:::{dropdown} BKPIECE1

* `border_fraction`: 0.9
* `range`: 0.0
:::

:::{dropdown} BiEST

* `range`: 0.0
:::

:::{dropdown} EST

* `goal_bias`: 0.5
* `range`: 0.0
:::

:::{dropdown} FMT

* `cache_cc`: True
* `extended_fmt`: True
* `heuristics`: False
* `num_samples`: 1000
* `radius_multiplier`: 1.1
* `use_k_nearest`: True
:::

:::{dropdown} InformedRRTstar

* `delay_collision_checking`: True
* `goal_bias`: 0.05
* `number_sampling_attempts`: 100
* `ordered_sampling`: False
* `ordering_batch_size`: 1
* `prune_threshold`: 0.05
* `range`: 0.0
* `rewire_factor`: 1.1
* `use_k_nearest`: True
:::

:::{dropdown} KPIECE1

* `border_fraction`: 0.9
* `goal_bias`: 0.05
* `range`: 0.0
:::

:::{dropdown} LBKPIECE1

* `border_fraction`: 0.9
* `range`: 0.0
:::

:::{dropdown} LBTRRT

* `epsilon`: 0.4
* `goal_bias`: 0.05
* `range`: 0.0
:::

:::{dropdown} LazyLBTRRT

* `epsilon`: 0.4
* `goal_bias`: 0.05
* `range`: 0.0
:::

:::{dropdown} LazyPRM

* `max_nearest_neighbors`: 8
* `range`: 0.0
:::

:::{dropdown} LazyPRMstar
*(No specific default parameters listed)*
:::

:::{dropdown} LazyRRT

* `goal_bias`: 0.05
* `range`: 0.0
:::

:::{dropdown} PDST

* `goal_bias`: 0.05
:::

:::{dropdown} PRM

* `max_nearest_neighbors`: 8
:::

:::{dropdown} PRMstar
*(No specific default parameters listed)*
:::

:::{dropdown} ProjEST

* `goal_bias`: 0.05
* `range`: 0.0
:::

:::{dropdown} RRT

* `goal_bias`: 0.05
* `intermediate_states`: False
* `range`: 0.0
:::

:::{dropdown} RRTConnect

* `intermediate_states`: False
* `range`: 0.0
:::

:::{dropdown} RRTXstatic

* `epsilon`: 0.0
* `goal_bias`: 0.05
* `informed_sampling`: False
* `number_sampling_attempts`: 100
* `range`: 0.0
* `rejection_variant`: 0
* `rejection_variant_alpha`: 1.0
* `rewire_factor`: 1.1
* `sample_rejection`: False
* `update_children`: True
* `use_k_nearest`: True
:::

:::{dropdown} RRTsharp

* `goal_bias`: 0.05
* `informed_sampling`: False
* `number_sampling_attempts`: 100
* `range`: 0.0
* `rejection_variant`: 0
* `rejection_variant_alpha`: 1.0
* `rewire_factor`: 1.1
* `sample_rejection`: False
* `update_children`: True
* `use_k_nearest`: True
:::

:::{dropdown} RRTstar

* `delay_collision_checking`: True
* `focus_search`: False
* `goal_bias`: 0.05
* `informed_sampling`: True
* `new_state_rejection`: False
* `number_sampling_attempts`: 100
* `ordered_sampling`: False
* `ordering_batch_size`: 1
* `prune_threshold`: 0.05
* `pruned_measure`: False
* `range`: 0.0
* `rewire_factor`: 1.1
* `sample_rejection`: False
* `tree_pruning`: False
* `use_admissible_heuristic`: True
* `use_k_nearest`: True
:::

:::{dropdown} SBL

* `range`: 0.0
:::

:::{dropdown} SST

* `goal_bias`: 0.05
* `pruning_radius`: 3.0
* `range`: 5.0
* `selection_radius`: 5.0
:::

:::{dropdown} STRIDE

* `degree`: 16
* `estimated_dimension`: 3.0
* `goal_bias`: 0.05
* `max_degree`: 18
* `max_pts_per_leaf`: 6
* `min_degree`: 12
* `min_valid_path_fraction`: 0.2
* `range`: 0.0
* `use_projected_distance`: False
:::

:::{dropdown} TRRT

* `goal_bias`: 0.05
* `range`: 0.0
* `temp_change_factor`: 0.1
:::
