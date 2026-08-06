## Introduction
`mirrt_star` is a C++ [`graph_core`](https://github.com/JRL-CARI-CNR-UNIBS/graph_core) plugin implementing **mIRRT\***, a multi-goal, informed, asymptotically-optimal sampling-based planner. Given a start configuration and a set of candidate goal configurations, the planner searches for a path to each goal concurrently, focusing sampling effort on the goals that are most promising according to a pluggable **goal-selection policy** (a multi-armed-bandit strategy by default), while pruning goals whose best-case ("utopia") cost can no longer beat the current best solution.

The solver is exposed as a `graph::core::TreeSolverPlugin`, loadable at runtime through [`cnr_class_loader`](https://github.com/JRL-CARI-CNR-UNIBS/cnr_class_loader) like any other `graph_core` solver (RRT, RRT*, BiRRT, Anytime-RRT, ...).

## How it works
- **Multiple goals, one start tree.** `MultigoalSolver` (`include/mirrt_star/solvers/mirrt_star.h`) extends `graph::core::TreeSolver`. Each goal added via `addGoal()` gets its own status (`search`, `refine`, `done`, `discard`), a dedicated informed/tube sampler, and, optionally, its own exploration tree for bidirectional search.
- **Goal selection.** At every iteration, `GoalSelectionManager` (`include/mirrt_star/multi_goal_selection`) assigns each active goal a sampling probability using a policy/reward pair (e.g. an ε-greedy multi-armed bandit driven by relative path-cost improvement), so search effort is steered toward the goals converging fastest.
- **Pruning.** As soon as a goal's utopia cost exceeds the current best solution cost, that goal is marked `discard` and its samples/tree nodes are pruned (`cleanTree()` / `purgeNodesOutsideEllipsoids`), keeping the search focused.
- **Informed sampling.** Each goal uses a `TubeInformedSampler`/`InformedSampler` (from `graph_core`) restricted to the current solution's cost ellipsoid, mixing global informed sampling with local ("tube") sampling around the incumbent path.

## Package layout
```
include/mirrt_star/
├── solvers/mirrt_star.h                     # MultigoalSolver
├── plugins/solvers/mirrt_star_plugin.h       # cnr_class_loader plugin wrapper
└── multi_goal_selection/
    ├── goal_selection_manager.h              # builds the policy + reward pair from parameters
    ├── goal_selection_util.h
    ├── policies/                             # policy_base, policy_mab(+e-greedy/UCB1/Thompson/KFMANB), policy_uniform_on_*, policy_custom_example
    └── rewards/                              # reward_base, reward_relative_improvement, reward_bernoulli, reward_best_cost
src/mirrt_star/                               # matching .cpp implementations
```

> Note: currently only the `MultiArmedBandit` / `eGreedy` policy and the `RelativeImprovement` reward are wired up in `GoalSelectionManager` (`src/mirrt_star/multi_goal_selection/goal_selection_manager.cpp`); the other policies/rewards exist in the codebase but are commented out pending further testing.

## Dependencies
`mirrt_star` builds on top of [`graph_core`](https://github.com/JRL-CARI-CNR-UNIBS/graph_core), and inherits its dependencies:

- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [cnr_logger](https://github.com/CNR-STIIMA-IRAS/cnr_logger)
- [cnr_param](https://github.com/CNR-STIIMA-IRAS/cnr_param)
- [cnr_class_loader](https://github.com/JRL-CARI-CNR-UNIBS/cnr_class_loader)

Install `graph_core` first (see its README for full instructions, including how to pull in the dependencies above via CPM or manually):

```bash
sudo apt update
sudo apt -y install libeigen3-dev libboost-all-dev libyaml-cpp-dev libpoco-dev liblog4cxx-dev libgtest-dev
```

## Building
`mirrt_star` uses plain CMake, like `graph_core`.

```bash
export PATH_TO_WS=path_to_your_ws

cd $PATH_TO_WS
mkdir -p build/mirrt_star
cmake -S src/mirrt_star -B build/mirrt_star -DCMAKE_INSTALL_PREFIX=$PATH_TO_WS/install
make -C build/mirrt_star install
```

Make sure `graph_core` is discoverable by CMake, e.g. by having sourced/exported its install path beforehand:

```bash
export CMAKE_PREFIX_PATH="$PATH_TO_WS/install:$CMAKE_PREFIX_PATH"
```

### Building within a Catkin/ROS workspace
As with `graph_core`, configure the workspace with `catkin config --install` and source `install/setup.bash` afterwards; no manual path export is needed in that case.

## Usage
`mirrt_star` registers `graph::mirrt_star::MIRRTStarPlugin` as a `graph::core::TreeSolverPlugin`, so it can be loaded dynamically wherever `graph_core` expects a solver plugin (e.g. via `cnr_class_loader`), or the underlying `graph::mirrt_star::MultigoalSolver` can be used directly:

```cpp
#include <mirrt_star/solvers/mirrt_star.h>

auto solver = std::make_shared<graph::mirrt_star::MultigoalSolver>(
    metrics, checker, sampler, goal_cost_fcn, logger);

solver->config(param_ns);       // reads mirrt_star parameters (see below) + base TreeSolver ones
solver->addStart(start_node);
solver->addGoal(goal_node_1);
solver->addGoal(goal_node_2);
// ... add all candidate goals ...
solver->finalizeProblem();      // initializes the goal-selection manager

graph::core::PathPtr solution;
while (solver->canImprove())
{
  solver->update(solution);
}
```

## Configuration
Parameters are read through [`cnr_param`](https://github.com/CNR-STIIMA-IRAS/cnr_param) under the namespace passed to `config()`, in the same way as other `graph_core` solvers. In addition to the base `TreeSolver` parameters (`max_distance`, `use_kdtree`, `extend`, `utopia_tolerance`), `MultigoalSolver` reads:

| Parameter | Default | Description |
|---|---|---|
| `rewire_radius` | `2 * max_distance` | Radius used to rewire the start tree while refining a solution. |
| `mixed_strategy` | `true` | If `true`, sample from the tube-informed sampler (informed ellipsoid + local tube around the incumbent path); if `false`, use the plain informed sampler. |
| `bidirectional` | `true` | If `true`, grow a tree from both the start and each unsolved goal and connect them; if `false`, only the start tree grows and periodically attempts to connect to each goal. |
| `k_nearest` | `false` | If `true`, rewiring uses the K nearest neighbours instead of a fixed radius. |
| `local_bias` | `0.3` | Initial probability (clamped to `[0, 1]`) of sampling from the local tube rather than the global informed region; adapted online based on solution improvement. |
| `tube_radius` | `0.3` | Radius (relative to path cost) of the local tube around the incumbent solution used by the informed sampler. |
| `forgetting_factor` | `0.01` | Decay factor applied to `local_bias` at each successful update. |

The goal-selection manager additionally reads, under the same namespace:

| Parameter | Default | Description |
|---|---|---|
| `policy_type` | `"MultiArmedBandit"` | Family of the goal-selection policy. |
| `policy_name` | `"eGreedy"` | Specific policy within the family (currently only `eGreedy` is enabled). |
| `reward_fcn` | `"RelativeImprovement"` | Reward function used to score a goal after it is sampled (currently only `RelativeImprovement` is enabled). |
| `warm_start_reward` | `false` | If `true`, initializes the policy's reward estimates from the goals' costs/utopias before planning starts. |

## License
`mirrt_star` is licensed under the BSD-3-Clause License (see the license header in each source file).

## Maintainers
- Manuel Beschi ([manuel.beschi@unibs.it](mailto:manuel.beschi@unibs.it))
- Cesare Tonola ([cesare.tonola@unibs.it](mailto:cesare.tonola@unibs.it))
- Marco Faroni ([marco.faroni@polimi.it](mailto:marco.faroni@polimi.it))
