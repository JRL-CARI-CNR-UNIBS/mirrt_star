# mIRRT* (`mirrt_star`)

## Introduction
`mirrt_star` is a high-performance C++ [`graph_core`](https://github.com/JRL-CARI-CNR-UNIBS/graph_core) plugin implementing **mIRRT\***, a multi-goal, informed, asymptotically-optimal sampling-based planner. Given a start configuration and a set of candidate goal configurations, the planner searches for a path to each goal concurrently, focusing sampling effort on the goals that are most promising according to a pluggable **goal-selection policy** (a multi-armed-bandit strategy by default), while pruning goals whose best-case ("utopia") cost can no longer beat the current best solution.

The solver is exposed as a `graph::core::TreeSolverPlugin`, loadable at runtime through [`cnr_class_loader`](https://github.com/JRL-CARI-CNR-UNIBS/cnr_class_loader) like any other `graph_core` solver (RRT, RRT*, BiRRT, Anytime-RRT, etc.).

---

## How It Works

- **Multiple Goals, One Start Tree**: `MultigoalSolver` (`include/mirrt_star/solvers/mirrt_star.h`) extends `graph::core::TreeSolver`. Each goal added via `addGoal()` gets its own status (`search`, `refine`, `done`, `discard`), a dedicated informed/tube sampler, and, optionally, its own exploration tree for bidirectional search.
- **Goal Selection**: At every iteration, `GoalSelectionManager` (`include/mirrt_star/multi_goal_selection`) assigns each active goal a sampling probability using a policy/reward pair (e.g., an ε-greedy multi-armed bandit driven by relative path-cost improvement), steering exploration toward goals that converge fastest.
- **Goal Weighting (`apple_weight`)**: In harvesting and targeted manipulation tasks, candidate goals may carry intrinsic costs or values (e.g. proximity to foliage, reachability, fruit quality). The solver combines motion distance with goal cost:

$$
\text{cost}(g) = \text{cost}_{\text{path}}(g) + \text{cost}_{\text{goal}}(g) \times \text{weight}_{\text{apple}}
$$

- **Direct Candidate Shortcut Checks**: When expanding the start tree towards a sample, the planner checks direct collision-free line connections to candidate goals. If a valid connection is discovered whose total cost improves the incumbent solution, the path is instantly registered without waiting for bidirectional trees to meet.
- **Pruning**: As soon as a goal's utopia cost exceeds the current best solution cost, that goal is marked `discard` and its samples/tree nodes are pruned (`cleanTree()` / `purgeNodesOutsideEllipsoids`), keeping memory usage bounded and the search focused.
- **Tube-Informed Sampling**: Once a solution is found for a goal, its search transitions to `refine` status using a `TubeInformedSampler`. Sampling is restricted to an ellipsoid and a local tube surrounding the incumbent trajectory, dynamically balancing global exploration and local refinement via an adaptive `local_bias`.

---

## Package Layout

```
mirrt_star/
├── CMakeLists.txt                            # Build definitions, optimization flags, C++20 standard
├── include/mirrt_star/
│   ├── solvers/mirrt_star.h                  # MultigoalSolver definition and public getters
│   ├── plugins/solvers/mirrt_star_plugin.h   # cnr_class_loader plugin wrapper
│   └── multi_goal_selection/
│       ├── goal_selection_manager.h          # Policy and reward manager
│       ├── goal_selection_util.h
│       ├── policies/                         # Bandit and heuristic policies (e-greedy, UCB1, etc.)
│       └── rewards/                          # Reward models (relative improvement, best cost, etc.)
├── src/mirrt_star/
│   ├── solvers/mirrt_star.cpp                # MultigoalSolver core planning loops and pruning logic
│   └── multi_goal_selection/                 # Matching .cpp implementations
└── README.md                                 # Documentation and branch changelog
```

---

## Dependencies & Building

`mirrt_star` depends on `graph_core` and its ecosystem:
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [cnr_logger](https://github.com/CNR-STIIMA-IRAS/cnr_logger)
- [cnr_param](https://github.com/CNR-STIIMA-IRAS/cnr_param)
- [cnr_class_loader](https://github.com/JRL-CARI-CNR-UNIBS/cnr_class_loader)
- [graph_core](https://github.com/JRL-CARI-CNR-UNIBS/graph_core)

### Building with colcon (ROS 2 Jazzy)
```bash
cd /path/to/your/ros2_ws
colcon build --packages-select mirrt_star
source install/setup.bash
```

### Standalone CMake Build
```bash
mkdir -p build/mirrt_star
cmake -S src/mirrt_star -B build/mirrt_star -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/path/to/install
make -C build/mirrt_star -j$(nproc) install
```

---

## Usage

Load dynamically via `cnr_class_loader` or instantiate directly in C++:

```cpp
#include <mirrt_star/solvers/mirrt_star.h>

auto solver = std::make_shared<graph::mirrt_star::MultigoalSolver>(
    metrics, checker, sampler, goal_cost_fcn, logger);

solver->config("mirrt_star_ns");   // Loads parameters from YAML/cnr_param
solver->addStart(start_node);

// Add candidate goals with optional target costs (e.g. apple harvesting cost)
solver->addGoal(goal_node_1, max_time, 0.5);
solver->addGoal(goal_node_2, max_time, 1.2);

solver->finalizeProblem();         // Initializes MAB goal selector and samplers

graph::core::PathPtr solution;
while (solver->canImprove())
{
  solver->update(solution);
}
```

---

## Configuration Parameters

Parameters are configured via YAML under the solver namespace:

| Parameter | Type | Default | Description |
|---|---|---|---|
| `apple_weight` | `double` | `10.0` | Multiplier for the intrinsic goal cost: `total = path_cost + goal_cost * apple_weight`. |
| `rewire_radius` | `double` | `2 * max_distance` | Radius used to rewire the start tree while refining a solution. |
| `mixed_strategy` | `bool` | `true` | If `true`, samples from tube-informed sampler (ellipsoid + local tube); if `false`, uses standard informed sampler. |
| `bidirectional` | `bool` | `true` | If `true`, grows trees from both start and unsolved goals to connect them. |
| `k_nearest` | `bool` | `false` | If `true`, rewiring queries the K nearest neighbors instead of a fixed Euclidean ball. |
| `local_bias` | `double` | `0.3` | Initial probability of sampling within the local tube surrounding the incumbent solution. |
| `tube_radius` | `double` | `0.01` | Normalized radius factor for the local tube around the incumbent solution. |
| `forgetting_factor`| `double` | `0.999` | Exponential decay factor applied to `local_bias` after each iteration. |
| `reward` | `double` | `1.0` | Multiplier scaling the local bias reward upon cost reduction. |
| `utopia_tolerance` | `double` | `1.0` | Termination threshold factor: when `cost <= utopia * utopia_tolerance`, goal is marked `done`. |
| `policy_type` | `string` | `"MultiArmedBandit"`| Goal-selection manager policy family. |
| `policy_name` | `string` | `"eGreedy"` | Bandit exploration policy (`eGreedy`). |
| `reward_fcn` | `string` | `"RelativeImprovement"`| Reward feedback metric for goal sampling. |
| `warm_start_reward`| `bool` | `false` | If `true`, initializes initial reward estimates from initial goal costs. |

---

## Modifications from `master` (`apple_project` branch)

The `apple_project` branch incorporates major architectural additions, algorithmic optimizations, and latency fixes developed for the autonomous apple picking project.

### Summary of Changes

| Component / File | Modification | Purpose / Impact |
|---|---|---|
| `CMakeLists.txt` | C++20, `-Ofast -flto -O3 -funroll-loops`, `EIGEN_MAX_ALIGN_BYTES=16` | Maximizes runtime performance through compiler vectorization, loop unrolling, and link-time optimization; ensures memory alignment compatibility. |
| `include/.../mirrt_star.h` | Extended `addGoal(node, time, apple_cost)` | Allows associating arbitrary target utility/cost metrics with each goal. |
| `include/.../mirrt_star.h` | Diagnostics & State Getters | Added `getGoals()`, `getGoalTrees()`, `getBestUtopia()`, `getUtopiaTolerance()`, `best_utopia_goal_index_`, `iter_`. |
| `src/.../mirrt_star.cpp` | Weighted Goal Cost Optimization | Integrated `apple_weight_` into goal evaluation and selection criteria. |
| `src/.../mirrt_star.cpp` | Fast Direct Goal Shortcut Connection | Start tree expansion tests direct collision-free shortcuts to candidate goals, drastically cutting time-to-first-solution. |
| `src/.../mirrt_star.cpp` | **Eliminated Solution Freeze during Tree Purging** | Fixed quadratic/recursive tree purging stalls: replaced repeated `cleanTree()` calls inside goal loops with cost hysteresis and size guards. |
| `src/.../mirrt_star.cpp` | **O(1) Goal Tree Connection Reset** | Replaced expensive recursive `goal_trees_.at(igoal)->cleanTree()` upon bidirectional connection with instantaneous O(1) shared pointer reassignment. |
| `src/.../mirrt_star.cpp` | Dynamic Local Bias Adaptation | Implemented adaptive tube sampling bias update proportional to relative cost progress. |

### In-Depth Details of Key Modifications

#### 1. High-Performance Compiler Optimizations (`CMakeLists.txt`)
- Configured default build type to `Release` with compiler flags `-funroll-loops -Wall -Ofast -flto -O3`.
- Set C++20 standard (`CMAKE_CXX_STANDARD 20`) to align with ROS 2 Jazzy and modern template features.
- Added `add_compile_definitions(EIGEN_MAX_ALIGN_BYTES=16)` to guarantee safe memory alignment for Eigen vector math.

#### 2. Goal Cost & Apple Weight Integration (`mirrt_star.h`, `mirrt_star.cpp`)
- Candidate goals can now receive an `apple_cost` specifying goal desirability:
  ```cpp
  virtual bool addGoal(const NodePtr &goal_node, const double &max_time = inf, const double &apple_cost = 0.0);
  ```
- Cost calculation balances robot travel distance and target selection preference:
  ```cpp
  costs_.at(igoal) = path_costs_.at(igoal) + goal_costs_.at(igoal) * apple_weight_;
  ```

#### 3. Direct Goal Shortcut Checking (`mirrt_star.cpp`)
- In `MultigoalSolver::update()`: when a new start tree node `new_start_node` is created, the solver checks whether it can connect directly to `goal_nodes_.at(igoal)`:
  ```cpp
  double cost_to_goal = metrics_->cost(new_start_node, goal_nodes_.at(igoal));
  if (cost_to_goal < max_distance_ && (start_tree_->costToNode(new_start_node) + cost_to_goal) < path_costs_.at(igoal))
  {
    if (checker_->checkConnection(new_start_node->getConfiguration(), goal_nodes_.at(igoal)->getConfiguration()))
    {
      // Instantly register shortcut solution to goal
    }
  }
  ```
  This enables rapid solution discovery without waiting for goal trees to expand backwards.

#### 4. Tree Purging Freeze Elimination & Pruning Hysteresis (`mirrt_star.cpp`)
- **Problem**: When a new best solution was found, the planner would often freeze for hundreds of milliseconds to several seconds. Profiling revealed:
  1. `cleanTree()` was called inside a loop over discarded goals, triggering 20–30 full tree traversals consecutively.
  2. Every connection of a goal tree invoked `goal_trees_.at(igoal)->cleanTree()`, recursively deleting nodes from the KD-tree one by one and constantly rebuilding it.
- **Solution**:
  - **Pruning Hysteresis**: In `MultigoalSolver::isBestSolution()`, `cleanTree()` was moved outside the loop and is only triggered when at least one goal was discarded AND cost improved by at least 10%:
    ```cpp
    if (any_discarded && cost_ < 0.90 * cost_at_last_clean)
    {
      cleanTree();
      cost_at_last_clean = cost_;
    }
    ```
  - **Tree Size Guard**: In `cleanTree()`, pruning is skipped if the tree has fewer than 500 nodes (`start_tree_->getNumberOfNodes() < 500`), avoiding useless overhead early in planning.
  - **O(1) Goal Tree Reset**: Replaced recursive node-by-node deletion on connected goal trees with instant tree object recreation:
    ```cpp
    goal_trees_.at(igoal) = std::make_shared<Tree>(goal_nodes_.at(igoal), max_distance_, checker_, metrics_, logger_, use_kdtree_);
    ```
  These modifications completely eliminate the solver stalls upon discovering improved paths.

#### 5. Adaptive Local Tube Bias (`mirrt_star.cpp`)
In `MultigoalSolver::update()`, whenever an improvement occurs, the sampler's local bias is updated:

$$
\text{bias}_{\text{local}} = \min\left( \gamma \cdot \text{bias}_{\text{local}} + R \cdot \frac{\text{cost}_{\text{old}} - \text{cost}}{\text{cost}_{\text{old}} - \text{utopia}_{\text{best}}}, 1.0 \right)
$$

This concentrates sampling near the solution trajectory while progress is rapid, and gradually widens search to the global informed ellipsoid when local refinements plateau.

---

## License
`mirrt_star` is licensed under the BSD-3-Clause License.

## Authors & Maintainers
- Manuel Beschi ([manuel.beschi@unibs.it](mailto:manuel.beschi@unibs.it))
- Cesare Tonola ([cesare.tonola@unibs.it](mailto:cesare.tonola@unibs.it))
- Marco Faroni ([marco.faroni@polimi.it](mailto:marco.faroni@polimi.it))
