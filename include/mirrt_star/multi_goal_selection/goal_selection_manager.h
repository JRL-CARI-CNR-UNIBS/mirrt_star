/**
 * @file goal_selection_manager.h
 * @brief Goal selection manager used by MI-RRT* to choose which goals to expand.
 */
#pragma once
/*
Copyright (c) 2021, Marco Faroni CNR-STIIMA marco.faroni@stiima.cnr.it
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

#include <mirrt_star/multi_goal_selection/policies/policy_base.h>
#include <mirrt_star/multi_goal_selection/rewards/reward_base.h>
#include <cnr_logger/cnr_logger.h>
#include <graph_core/util.h>

using namespace graph::core;
using namespace graph;

/**
 * @brief Goal selection policies and rewards for multi-goal planning.
 */
namespace multi_goal_selection
{
/**
 * @brief Forward declaration of multi_goal_selection::GoalSelectionManager.
 */
class GoalSelectionManager;
/**
 * @brief Shared pointer to a GoalSelectionManager.
 */
typedef std::shared_ptr<GoalSelectionManager> GoalSelectionManagerPtr;

/**
 * @brief Chooses which goals MI-RRT* expands at each iteration.
 *
 * It owns a policy (multi_goal_selection::PolicyBase) and a reward function
 * (multi_goal_selection::RewardBase). After each iteration the reward is computed
 * from the goal costs and given to the policy for the goals that were expanded;
 * then the policy returns the new expansion probabilities.
 *
 * Parameters read under the namespace @p name:
 * - @c policy_type (string, default "MultiArmedBandit"): only "MultiArmedBandit" is supported;
 * - @c policy_name (string, default "eGreedy"): only "eGreedy" is currently enabled;
 * - @c reward_fcn (string, default "RelativeImprovement"): only "RelativeImprovement" is currently enabled;
 * - @c warm_start_reward (bool, default false): initialize the policy with rewards computed from the utopias;
 * - the parameters of the selected policy (e.g. PolicyMABEGreedy).
 */
class GoalSelectionManager
{
public:
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief Constructor. Reads the parameters and creates policy and reward function.
   * @param name Parameter namespace.
   * @param logger Logger.
   * @param n_goals Number of goals.
   * @param n_dof Number of degrees of freedom (used by volume-based policies).
   */
  GoalSelectionManager(const std::string& name,
                       const cnr_logger::TraceLoggerPtr &logger,
                       const unsigned int& n_goals,
                       const unsigned int& n_dof);
  /**
   * @brief Update the policy with the last reward and compute the new expansion probabilities.
   * @param were_goals_selected True for each goal expanded in the last iteration.
   * @param costs Current cost of each goal.
   * @param utopias Utopia of each goal.
   * @param best_cost Current best cost.
   * @return Probability of expanding each goal.
   */
  std::vector<double> calculateProbabilities(const std::vector<bool>& were_goals_selected, const std::vector<double>& costs, const std::vector<double>& utopias, const double& best_cost);
  /**
   * @brief Initialize the policy with one reward per goal, computed using the utopia as best cost.
   * @param costs Current cost of each goal.
   * @param utopias Utopia of each goal.
   * @param best_cost Current best cost (unused).
   */
  void warmStart(const std::vector<double>& costs, const std::vector<double>& utopias, const double& best_cost);
  /**
   * @brief True if the @c warm_start_reward parameter is set.
   */
  bool isWarmStartSet(){return do_warm_start_;};

protected:
  int goal_number_;  ///< Number of goals.
  std::vector<double> goal_probabilities_;  ///< Last computed probabilities.
  std::string policy_type_;  ///< Policy type (parameter @c policy_type).
  std::string policy_name_;  ///< Policy name (parameter @c policy_name).
  std::string reward_fcn_name_;  ///< Reward function name (parameter @c reward_fcn).
  bool do_warm_start_;  ///< Warm start flag (parameter @c warm_start_reward).

  std::shared_ptr<multi_goal_selection::PolicyBase> policy_;  ///< Goal selection policy.
  std::shared_ptr<multi_goal_selection::RewardBase> reward_fcn_;  ///< Reward function.

  /**
   * @brief Pointer to a TraceLogger instance for logging.
   *
   * This member variable represents a pointer to a TraceLogger instance, allowing
   * to perform logging operations. TraceLogger is a part of the cnr_logger library.
   * Ensure that the logger is properly configured and available for use.
   */
  cnr_logger::TraceLoggerPtr logger_;
};

}
