/*
Copyright (c) 2020, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file mirrt_star.h
 * @brief Multi-goal informed RRT* solver (MI-RRT*).
 */
#pragma once
#include <graph_core/solvers/tree_solver.h>
#include <graph_core/samplers/tube_informed_sampler.h>
#include <mirrt_star/multi_goal_selection/goal_selection_manager.h>

using namespace graph::core;
using namespace graph;

namespace graph
{
  namespace mirrt_star
  {

    /**
     * @brief Forward declaration of graph::mirrt_star::MultigoalSolver.
     */
    class MultigoalSolver;
    /**
     * @brief Shared pointer to a MultigoalSolver.
     */
    typedef std::shared_ptr<MultigoalSolver> MultigoalSolverPtr;

    /**
     * @class MultigoalSolver
     * @brief Multi-goal informed RRT* (MI-RRT*) path planner.
     *
     * The solver plans from one start configuration towards a set of candidate goal
     * configurations at the same time and returns the path to the goal with the lowest
     * total cost (path cost + goal cost). It keeps a single start tree and, when
     * bidirectional search is enabled, one backward tree per goal.
     *
     * Every goal has a status:
     * - @c search: no path to the goal is known yet;
     * - @c refine: a path exists and it is being optimized (RRT* rewiring);
     * - @c done: the cost is within @c utopia_tolerance of the goal utopia (lower bound), so it cannot improve much more;
     * - @c discard: the goal utopia is worse than the best cost found so far, so the goal is ignored.
     *
     * At each iteration a goal selection policy (see multi_goal_selection::GoalSelectionManager)
     * decides which goals are expanded. Samples are drawn from an informed set around each goal
     * (graph::core::InformedSampler), optionally mixed with a tube around the current solution
     * (graph::core::TubeInformedSampler).
     *
     * Typical use:
     * @code
     * auto solver = std::make_shared<graph::mirrt_star::MultigoalSolver>(metrics, checker, sampler, goal_cost_fcn, logger);
     * solver->config("/my_planner/mirrt_star");
     * solver->addStart(start_node);
     * for (const auto& goal : goals)
     *   solver->addGoal(goal, direct_connection_time, goal_cost);
     * solver->finalizeProblem();
     * graph::core::PathPtr solution;
     * while (time_left)
     *   solver->update(solution);  // true when the best solution improved
     * @endcode
     *
     * Parameters read by config() under @c param_ns (besides those of graph::core::TreeSolver):
     * - @c rewire_radius (double, default 2*max_distance): RRT* rewiring radius;
     * - @c mixed_strategy (bool, default true): sample also inside a tube around the current solution;
     * - @c bidirectional (bool, default true): grow one tree from each goal as well;
     * - @c k_nearest (bool, default false): use k-nearest rewiring instead of a fixed radius;
     * - @c local_bias (double in [0,1], default 0.3): initial probability of sampling inside the tube;
     * - @c tube_radius (double, default 0.3): tube radius, relative to the solution cost;
     * - @c forgetting_factor (double, default 0.01): forgetting factor of the adaptive local bias;
     * - @c apple_weight (double, default 10.0): weight of the goal cost (currently unused by the solver);
     * - goal selection parameters, see multi_goal_selection::GoalSelectionManager.
     */
    class MultigoalSolver : public TreeSolver
    {
    protected:
      /**
       * @brief Search status of a single goal.
       */
      enum GoalStatus
      {
        search,  ///< no path to this goal has been found yet
        refine,  ///< a path exists and is being optimized
        done,  ///< cost is within utopia tolerance, the goal is not expanded anymore
        discard  ///< utopia is worse than the best cost, the goal is ignored
      };
      std::vector<NodePtr> goal_nodes_;  ///< Candidate goal nodes.
      std::vector<double> costs_;  ///< Total cost (path + goal cost) for each goal; infinity if not solved.
      std::vector<double> utopias_;  ///< Lower bound of the total cost for each goal.
      std::vector<double> goal_probabilities_;  ///< Probability of expanding each goal at the next iteration.
      std::vector<bool> were_goals_sampled_;  ///< True for goals expanded during the last iteration.
      std::vector<PathPtr> solutions_;  ///< Best path found for each goal (nullptr if none).
      std::vector<double> goal_costs_;  ///< Cost assigned to each goal by the goal cost function (plus the extra cost given to addGoal()).
      std::vector<double> path_costs_;  ///< Path cost for each goal.

      std::vector<TreePtr> goal_trees_;  ///< Backward tree rooted at each goal (used when bidirectional).
      std::vector<TubeInformedSamplerPtr> tube_samplers_;  ///< Tube samplers around the current solution of each goal.
      std::vector<SamplerPtr> samplers_;  ///< Informed samplers for each goal.
      std::vector<GoalStatus> status_;  ///< Search status of each goal.

      std::random_device rd_;
      std::mt19937 gen_;
      std::uniform_real_distribution<double> ud_;

      double cost_at_last_clean = std::numeric_limits<double>::infinity();  ///< Best cost when the start tree was last pruned.
      unsigned int iter_ = 0;  ///< Number of calls to update().
      int best_goal_index = -1;  ///< Index of the goal with the best solution (-1 if none).
      int best_utopia_goal_index_ = -1;  ///< Index of the goal with the lowest utopia (-1 if none).
      double r_rewire_ = 1.;  ///< RRT* rewiring radius.
      double local_bias_ = 0.3;  ///< Probability of sampling inside the tube (adapted online).
      double reward_ = 1.0;  ///< Gain of the local bias adaptation.
      double forgetting_factor_ = 0.999;  ///< Forgetting factor of the local bias adaptation.
      double tube_radius_ = 0.01;  ///< Tube radius, relative to the solution cost.
      double goal_bias_ = 0.05;  ///< Probability of trying a direct connection to the goal.
      bool bidirectional_ = true;  ///< Grow a backward tree from each goal.
      bool mixed_strategy_ = true;  ///< Sample from both the informed set and the tube.
      bool knearest_ = false;  ///< Use k-nearest rewiring.

      double apple_weight_ = 10.0;  ///< Weight of the goal cost (read from parameters, currently unused).
      /**
       * @brief Check that the problem has a start tree and at least one goal.
       * @param max_time Unused.
       * @return True if start and goals are defined.
       */
      virtual bool setProblem(const double &max_time = std::numeric_limits<double>::infinity()) override;
      /**
       * @brief Check if the solution of a goal is the new global best and, if so, update the solver state.
       *
       * When a new best is found, goals whose utopia exceeds the new cost are discarded,
       * the informed samplers are shrunk, and the start tree is pruned if needed.
       * @param index Goal index.
       * @return True if the goal solution is the new best one.
       */
      bool isBestSolution(const int &index);

      multi_goal_selection::GoalSelectionManagerPtr goal_manager_;  ///< Policy that decides which goals to expand. Created by finalizeProblem().

      /**
       * @brief Print best cost, tree size and status of each goal.
       */
      virtual void printMyself(std::ostream &os) const override;

    public:
      /**
       * @brief Constructor. config() must be called before adding start and goals.
       * @param metrics Metrics used to compute path costs.
       * @param checker Collision checker.
       * @param sampler Sampler; only its bounds are used to build the per-goal informed samplers.
       * @param goal_cost_fcn Function that assigns a cost to each goal.
       * @param logger Logger.
       */
      MultigoalSolver(const MetricsPtr &metrics,
                      const CollisionCheckerPtr &checker,
                      const SamplerPtr &sampler,
                      const GoalCostFunctionPtr &goal_cost_fcn,
                      const cnr_logger::TraceLoggerPtr &logger) : TreeSolver(metrics, checker, sampler, goal_cost_fcn, logger),
                                                                  gen_(time(0))
      {
        ud_ = std::uniform_real_distribution<double>(0, 1);
      }

      /**
       * @brief Read the parameters (see class description).
       * @param param_ns Parameter namespace.
       * @return False if the base TreeSolver configuration fails.
       */
      virtual bool config(const std::string &param_ns) override;
      /**
       * @brief Create the goal selection manager for the current set of goals.
       * Called by finalizeProblem().
       * @return Always true.
       */
      virtual bool initGoalSelector();
      /**
       * @brief Run one iteration of the planner.
       *
       * Selects the goals to expand, samples a configuration for each of them, grows
       * the trees, and updates goal status and costs.
       * @param[out] solution Best solution found so far (nullptr if none).
       * @return True if the best solution improved during this iteration, false otherwise
       * (also when the solver is not initialized or the best cost reached the utopia tolerance).
       */
      virtual bool update(PathPtr &solution) override;

      /**
       * @brief Set the start node and create the start tree.
       * @param start_node Start node.
       * @param max_time Unused.
       * @return False if the solver is not configured.
       */
      virtual bool addStart(const NodePtr &start_node, const double &max_time = std::numeric_limits<double>::infinity()) override;

      /**
       * @brief Register a candidate goal.
       *
       * The goal is skipped if it is in collision or if its utopia is worse than the
       * current best cost. The solver first tries a direct connection from the start
       * tree; if it succeeds the goal starts in @c refine (or @c done) status, otherwise
       * a backward tree is created and the goal starts in @c search status.
       * @param goal_node Goal node.
       * @param max_time Maximum time for the direct connection attempt.
       * @param apple_cost Extra cost added to the goal cost (e.g. a task-dependent cost).
       * @return True if the goal was registered.
       */
      virtual bool addGoal(const NodePtr &goal_node, const double &max_time = std::numeric_limits<double>::infinity(), const double &apple_cost = 0.0);
      /**
       * @brief Register a candidate goal with no extra cost.
       * @see addGoal(const NodePtr&, const double&, const double&)
       */
      virtual bool addGoal(const NodePtr &goal_node, const double &max_time = std::numeric_limits<double>::infinity()) override
      {
        return addGoal(goal_node, max_time, 0.0);
      }

      /**
       * @brief Use an existing tree as start tree.
       * @param start_tree Tree whose root is the start configuration.
       * @param max_time Unused.
       * @return Always true.
       */
      virtual bool addStartTree(const TreePtr &start_tree, const double &max_time = std::numeric_limits<double>::infinity()) override;
      /**
       * @brief Remove start, goals and solutions. Parameters are kept.
       */
      virtual void resetProblem() override;

      /**
       * @brief Finish the problem setup. Call it after the last addGoal() and before update().
       *
       * Creates the goal selection manager and, if requested, warm-starts it.
       * @return True on success.
       */
      virtual bool finalizeProblem();

      /**
       * @brief Get the backward trees (one per goal, nullptr if not created).
       */
      std::vector<TreePtr> getGoalTrees();
      /**
       * @brief Get the registered goal nodes.
       */
      std::vector<NodePtr> getGoals()
      {
        return goal_nodes_;
      }
      /**
       * @brief Get the lowest utopia (lower bound of the cost) among the goals.
       */
      double getBestUtopia() const
      {
        return best_utopia_;
      }
      /**
       * @brief Get the utopia tolerance used to discard goals.
       */
      double getUtopiaTolerance() const
      {
        return utopia_tolerance_;
      }
      /**
       * @brief Prune the start tree, removing nodes outside the informed sets of the active goals.
       */
      void cleanTree();
    };

  } //  end namespace mirrt_star
} //  end namespace graph
