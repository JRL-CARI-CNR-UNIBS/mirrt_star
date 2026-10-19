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

## License

BSD 3-Clause, see [LICENSE](LICENSE).
