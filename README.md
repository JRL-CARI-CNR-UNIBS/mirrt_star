# mirrt_star

Multi-goal informed RRT* (**MI-RRT\***) path planner for [graph_core](https://github.com/JRL-CARI-CNR-UNIBS/graph_core).

MI-RRT\* plans from one start configuration towards **many candidate goal configurations at once**
(for example all the inverse-kinematics solutions of a pose) and returns the path to the goal with the
lowest total cost:

```
total cost = path cost + goal cost
```

Instead of running one planner per goal, MI-RRT\* shares a single start tree among all goals and uses a
goal selection policy (a multi-armed bandit) to spend more effort on the goals that are more promising.
Goals that cannot beat the current best solution are discarded early.

## Features

- One start tree shared by all goals, plus an optional backward tree for each goal (bidirectional search).
- Informed sampling per goal, mixed with sampling inside a tube around the current solution;
  the mix (`local_bias`) adapts online to the progress of the solution.
- Goal status tracking: `search` → `refine` → `done`, or `discard` when the goal utopia (lower bound of
  its cost) is worse than the best cost found so far.
- Goal selection with an epsilon-greedy multi-armed bandit and a relative-improvement reward.
- Optional per-goal cost (`apple_cost` in `addGoal()`), e.g. to prefer some IK solutions.
- Plugin (`graph::mirrt_star::MIRRTStarPlugin`) loadable with `cnr_class_loader` as a
  `graph::core::TreeSolverPlugin`.

## Package layout

| Path | Content |
|---|---|
| `include/mirrt_star/solvers/mirrt_star.h` | `graph::mirrt_star::MultigoalSolver`, the planner |
| `include/mirrt_star/plugins/solvers/mirrt_star_plugin.h` | plugin wrapper |
| `include/mirrt_star/multi_goal_selection/` | goal selection manager, policies and rewards |
| `docs/TUTORIAL.md` | step-by-step tutorial |

## Dependencies

- [graph_core](https://github.com/JRL-CARI-CNR-UNIBS/graph_core) (and its dependencies:
  Eigen3, `cnr_logger`, `cnr_param`, `cnr_class_loader`)
- C++20 compiler, CMake ≥ 3.16

## Build

In a colcon workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/mirrt_star.git
cd ~/ros2_ws
colcon build --packages-up-to mirrt_star
```

As a plain CMake package (graph_core must be installed and findable by CMake):

```bash
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=<install_dir>
make install
```

Then, in your CMake project:

```cmake
find_package(mirrt_star REQUIRED)
target_link_libraries(my_target PUBLIC mirrt_star::mirrt_star)
```

## Quick example

```cpp
#include <mirrt_star/solvers/mirrt_star.h>

auto solver = std::make_shared<graph::mirrt_star::MultigoalSolver>(metrics, checker, sampler,
                                                                   goal_cost_fcn, logger);
solver->config("/my_planner/mirrt_star");   // read parameters with cnr_param
solver->addStart(std::make_shared<graph::core::Node>(q_start, logger));
for (const Eigen::VectorXd& q : goal_configurations)
  solver->addGoal(std::make_shared<graph::core::Node>(q, logger), 0.1 /* direct connection time */);
solver->finalizeProblem();                  // must be called after the last addGoal()

graph::core::PathPtr solution;
while (elapsed < max_time && solver->canImprove())
  solver->update(solution);                 // returns true when the best solution improves
```

See [docs/TUTORIAL.md](docs/TUTORIAL.md) for a complete walkthrough, including the parameters.

## Parameters

Parameters are read by `MultigoalSolver::config(param_ns)` under `param_ns`.

| Name | Type | Default | Description |
|---|---|---|---|
| `max_distance` | double | 1.0 | Maximum tree extension step (from `graph_core::TreeSolver`) |
| `use_kdtree` | bool | true | Use a KD-tree for nearest-neighbour search (from `TreeSolver`) |
| `extend` | bool | false | Use *extend* instead of *connect* when growing trees (from `TreeSolver`) |
| `utopia_tolerance` | double | 0.01 | A goal is `done` when cost ≤ utopia × (1 + `utopia_tolerance`) (from `TreeSolver`) |
| `rewire_radius` | double | 2 × `max_distance` | RRT\* rewiring radius |
| `bidirectional` | bool | true | Grow a backward tree from each goal |
| `mixed_strategy` | bool | true | Sample also inside a tube around the current solution |
| `k_nearest` | bool | false | Use k-nearest rewiring instead of a fixed radius |
| `local_bias` | double | 0.3 | Initial probability of sampling in the tube, clamped to [0, 1] |
| `tube_radius` | double | 0.3 | Tube radius, relative to the solution cost |
| `forgetting_factor` | double | 0.01 | Forgetting factor of the `local_bias` adaptation |
| `apple_weight` | double | 10.0 | Read but currently unused |
| `policy_type` | string | `MultiArmedBandit` | Goal selection policy family |
| `policy_name` | string | `eGreedy` | Goal selection policy |
| `reward_fcn` | string | `RelativeImprovement` | Reward used to update the policy |
| `warm_start_reward` | bool | false | Initialize the policy with rewards computed from the goal utopias |
| `epsilon_coef` | double | 0.2 | Epsilon-greedy exploration probability |
| `epsilon_exp` | double | 1.0 | If < 1, epsilon is multiplied by this value at each update |
| `reward_gain` | double | 1.0 | Epsilon-greedy reward gain |
| `egreedy_reward_forgetting_factor` | double | 0.0 | If > 0, expected rewards are an exponential moving average |

Other policies (UCB1, Thompson sampling, KF-MANB, uniform on goals/volume) are present in
`include/mirrt_star/multi_goal_selection/policies/` but are not yet ported from ROS 1 and are disabled.

## API documentation

The API reference of the classes and functions is in [docs/API.md](docs/API.md) (generated from the
Doxygen comments).

To browse it as HTML, run Doxygen from the package folder:

```bash
doxygen Doxyfile
xdg-open doxygen/html/index.html
```

## How it works

- **Multiple Goals, One Start Tree**: `MultigoalSolver` (`include/mirrt_star/solvers/mirrt_star.h`) extends `graph::core::TreeSolver`. Each goal added via `addGoal()` gets its own status (`search`, `refine`, `done`, `discard`), a dedicated informed/tube sampler, and, optionally, its own exploration tree for bidirectional search.
- **Goal Selection**: At every iteration, `GoalSelectionManager` (`include/mirrt_star/multi_goal_selection`) assigns each active goal a sampling probability using a policy/reward pair (e.g., an ε-greedy multi-armed bandit driven by relative path-cost improvement), steering exploration toward goals that converge fastest.
- **Goal Cost**: In harvesting and targeted manipulation tasks, candidate goals may carry intrinsic costs
  (e.g. proximity to foliage, reachability, fruit quality), passed as `apple_cost` to `addGoal()`. The solver
  minimizes `path cost + goal cost`. The `apple_weight` parameter is read but not yet applied.
- **Direct Candidate Shortcut Checks**: When expanding the start tree towards a sample, the planner checks direct collision-free line connections to candidate goals. If a valid connection is discovered whose total cost improves the incumbent solution, the path is instantly registered without waiting for bidirectional trees to meet.
- **Pruning**: As soon as a goal's utopia cost exceeds the current best solution cost, that goal is marked `discard` and its samples/tree nodes are pruned (`cleanTree()` / `purgeNodesOutsideEllipsoids`), keeping memory usage bounded and the search focused.
- **Tube-Informed Sampling**: Once a solution is found for a goal, its search transitions to `refine` status using a `TubeInformedSampler`. Sampling is restricted to an ellipsoid and a local tube surrounding the incumbent trajectory, dynamically balancing global exploration and local refinement via an adaptive `local_bias`.

## Modifications from `master` (`apple_project` branch)

The `apple_project` branch incorporates major architectural additions, algorithmic optimizations, and latency fixes developed for the autonomous apple picking project.

### Summary of Changes

| Component / File | Modification | Purpose / Impact |
|---|---|---|
| `CMakeLists.txt` | C++20, `-Ofast -flto -O3 -funroll-loops`, `EIGEN_MAX_ALIGN_BYTES=16` | Maximizes runtime performance through compiler vectorization, loop unrolling, and link-time optimization; ensures memory alignment compatibility. |
| `include/.../mirrt_star.h` | Extended `addGoal(node, time, apple_cost)` | Allows associating arbitrary target utility/cost metrics with each goal. |
| `include/.../mirrt_star.h` | Diagnostics & State Getters | Added `getGoals()`, `getGoalTrees()`, `getBestUtopia()`, `getUtopiaTolerance()`, `best_utopia_goal_index_`, `iter_`. |
| `src/.../mirrt_star.cpp` | Goal Cost | Goal cost added to the path cost in goal evaluation and selection (`apple_weight` is read but not yet applied). |
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
- The total cost of a goal is `path_costs_.at(igoal) + goal_costs_.at(igoal)`. The `apple_weight` parameter is
  read in `config()` but not yet used as a multiplier.

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
  - **Pruning Hysteresis**: In `MultigoalSolver::isBestSolution()`, `cleanTree()` was moved outside the loop and is called at most once, when a goal was discarded, the cost improved by at least 10%, or the tree asks for it:
    ```cpp
    if (any_discarded || (cost_ < 0.90 * cost_at_last_clean) || start_tree_->needCleaning())
    {
      cost_at_last_clean = cost_;
      cleanTree();
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

## License

BSD 3-Clause, see [LICENSE](LICENSE).

## Authors & Maintainers
- Manuel Beschi ([manuel.beschi@unibs.it](mailto:manuel.beschi@unibs.it))
- Cesare Tonola ([cesare.tonola@unibs.it](mailto:cesare.tonola@unibs.it))
- Marco Faroni ([marco.faroni@polimi.it](mailto:marco.faroni@polimi.it))
