# MI-RRT\* tutorial

This tutorial shows how to plan a path from one start configuration to the best of several goal
configurations with `graph::mirrt_star::MultigoalSolver`.

It uses only `graph_core` components (a toy collision checker in a 3D configuration space), so it runs
without ROS. In a real application you replace the collision checker with one for your robot, for
example the MoveIt-based checkers of `moveit_collision_checker`.

If you are new to `graph_core`, read its
[introductory tutorial](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/blob/master/docs/tutorial/tutorial_intro.md)
first: logger, parameters, metrics, samplers and collision checkers are created in the same way.

## 1. Create a package that uses mirrt_star

`CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(mirrt_star_tutorial)
set(CMAKE_CXX_STANDARD 20)

find_package(mirrt_star REQUIRED)

add_executable(mirrt_star_tutorial src/main.cpp)
target_link_libraries(mirrt_star_tutorial PRIVATE mirrt_star::mirrt_star)
```

## 2. Write the parameters

Parameters are read with `cnr_param`. Create `config/mirrt_star_tutorial.yaml`:

```yaml
mirrt_star_tutorial:
  # graph_core TreeSolver parameters
  max_distance: 0.5          # maximum extension step
  use_kdtree: true
  extend: false              # false = connect (greedy), true = extend (one step)
  utopia_tolerance: 0.01     # a goal is "done" when cost <= utopia * 1.01

  # MI-RRT* parameters
  bidirectional: true        # one backward tree per goal
  mixed_strategy: true       # also sample in a tube around the current solution
  local_bias: 0.3            # initial probability of sampling in the tube
  tube_radius: 0.3           # tube radius, relative to the solution cost
  forgetting_factor: 0.01
  rewire_radius: 1.0

  # goal selection
  policy_type: "MultiArmedBandit"
  policy_name: "eGreedy"
  reward_fcn: "RelativeImprovement"
  epsilon_coef: 0.2
```

Load the file in the parameter server before running the program:

```bash
cnr_param_server --path-to-file config/mirrt_star_tutorial.yaml
```

Every parameter has a default value, so you can also start with an empty namespace and tune later.

## 3. Build the problem

`src/main.cpp`:

```cpp
#include <mirrt_star/solvers/mirrt_star.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/samplers/uniform_sampler.h>
#include <graph_core/collision_checkers/cube_3d_collision_checker.h>
#include <graph_core/metrics/goal_cost_function_base.h>

int main(int argc, char** argv)
{
  // Logger (see cnr_logger for the configuration file format)
  auto logger = std::make_shared<cnr_logger::TraceLogger>("mirrt_star_tutorial", "config/logger_param.yaml");

  // 3-DOF configuration space in [-2.5, 2.5]^3
  const size_t dof = 3;
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(dof, -2.5);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(dof, 2.5);

  // Toy collision checker: configurations with all |q_i| < 1 are in collision (a cube in the middle)
  graph::core::CollisionCheckerPtr checker =
      std::make_shared<graph::core::Cube3dCollisionChecker>(logger, 1.0, 0.01);
  graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);
  graph::core::SamplerPtr sampler = std::make_shared<graph::core::UniformSampler>(lb, ub, logger);

  // Goal cost function: the base class assigns zero cost to every goal
  graph::core::GoalCostFunctionPtr goal_cost_fcn = std::make_shared<graph::core::GoalCostFunctionBase>();

  auto solver = std::make_shared<graph::mirrt_star::MultigoalSolver>(metrics, checker, sampler,
                                                                     goal_cost_fcn, logger);
  if (!solver->config("/mirrt_star_tutorial"))
    return 1;
```

The `sampler` passed to the constructor only defines the bounds of the search space: MI-RRT\* builds
its own informed sampler for each goal.

## 4. Add start and goals

```cpp
  Eigen::VectorXd q_start(dof);
  q_start << -1.5, -1.5, -1.5;
  if (!solver->addStart(std::make_shared<graph::core::Node>(q_start, logger)))
    return 1;

  // Three candidate goals, e.g. three IK solutions of the same pose
  std::vector<Eigen::VectorXd> goals(3, Eigen::VectorXd(dof));
  goals[0] << 1.5, 1.5, 1.5;
  goals[1] << 1.5, -1.5, 1.5;
  goals[2] << -1.5, 1.5, 1.5;

  const double direct_connection_time = 0.05;  // time spent trying to connect start and goal directly
  for (size_t i = 0; i < goals.size(); i++)
  {
    // Optional extra cost of the goal: here we slightly penalize the last goal
    double goal_cost = (i == 2) ? 0.5 : 0.0;
    solver->addGoal(std::make_shared<graph::core::Node>(goals[i], logger), direct_connection_time, goal_cost);
  }

  // Must be called after the last addGoal(): creates the goal selection policy
  solver->finalizeProblem();
```

`addGoal()` returns `false`, and the goal is skipped, when:

- the goal is in collision;
- its utopia (straight-line cost + goal cost) is already worse than the best solution found so far.

When the start and the goal can be connected directly, the goal immediately has a solution.

## 5. Run the planner

```cpp
  graph::core::PathPtr solution;
  const auto t0 = std::chrono::steady_clock::now();
  const double max_time = 2.0;

  while (solver->canImprove())
  {
    if (solver->update(solution))
      CNR_INFO(logger, "Improved solution, cost = " << solution->cost());

    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (elapsed > max_time)
      break;
  }

  if (!solver->solved())
  {
    CNR_ERROR(logger, "No solution found");
    return 1;
  }

  CNR_INFO(logger, "Best cost: " << solution->cost() << "\n" << *solution);
  CNR_INFO(logger, "Solver state:\n" << *solver);  // status of each goal
  return 0;
}
```

Each call to `update()` is one iteration. It returns `true` only when the global best solution
improves, and `solution` always holds the best path found so far. The loop stops when the time is over
or when `canImprove()` becomes `false`, which happens when the best cost is within `utopia_tolerance`
of the best utopia.

## 6. Read the output

Printing the solver (`<< *solver`) shows, for every goal:

- **status**:
  - `search`: no path yet;
  - `refine`: a path exists and is being optimized;
  - `done`: the cost reached its utopia;
  - `discard`: the goal cannot beat the best solution.
- **cost**, **path cost**, **goal cost** and **utopia**.

With the goals above you will see that the goal with the extra cost of 0.5 is often discarded early,
because its utopia becomes worse than the best solution: MI-RRT\* stops spending time on it.

## 7. Tuning tips

| Symptom | Try |
|---|---|
| The first solution takes too long | Increase `max_distance`; keep `bidirectional: true`; increase the direct connection time in `addGoal()` |
| The solution improves slowly | Increase `local_bias` or `tube_radius`; lower `utopia_tolerance` only if you need near-optimal paths |
| Too much time on bad goals | Decrease `epsilon_coef` (less random exploration); give goal costs that reflect your preference, so bad goals are discarded earlier |
| Memory grows with many goals | Set `bidirectional: false` (no backward trees) |

## 8. Use it as a plugin

`MIRRTStarPlugin` exposes the solver as a `graph::core::TreeSolverPlugin`, so applications that load
solvers by name with `cnr_class_loader` can use it without linking to `mirrt_star` directly:

```cpp
#include <cnr_class_loader/multi_library_class_loader.hpp>
#include <graph_core/plugins/solvers/tree_solver_plugin.h>

cnr_class_loader::MultiLibraryClassLoader loader(false);
loader.loadLibrary("libmirrt_star.so");
auto plugin = loader.createInstance<graph::core::TreeSolverPlugin>("graph::mirrt_star::MIRRTStarPlugin");
plugin->init(param_ns, metrics, checker, sampler, goal_cost_fcn, logger);
graph::core::TreeSolverPtr solver = plugin->getSolver();
```

Through the base class interface you can only add goals with `addGoal(node, max_time)`, which uses no
extra goal cost. Remember to cast to `graph::mirrt_star::MultigoalSolver` to call `finalizeProblem()`.

## See also

- `parallel_multigoal_planner`: a ROS 2 action server that plans with MI-RRT\* towards all the IK
  solutions of a set of poses.
