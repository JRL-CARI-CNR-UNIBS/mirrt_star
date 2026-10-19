/**
 * @file policy_base.h
 * @brief Base class of the goal selection policies.
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

#include <random>
#include <time.h>
#include <memory>
#include <graph_core/util.h>

using namespace graph::core;
using namespace graph;

namespace multi_goal_selection
{

/**
 * @brief Base class of a goal selection policy.
 *
 * A policy returns, at each iteration, the probability of expanding each goal,
 * and it is updated with the reward obtained after each expansion.
 */
class PolicyBase
{
public:
  /**
   * @brief Constructor.
   * @param name Parameter namespace of the policy.
   * @param n_goals Number of goals (arms).
   * @param logger Logger.
   */
  PolicyBase(const std::string& name,
             const int& n_goals,
             const cnr_logger::TraceLoggerPtr &logger):
    gen_(time(0)),
    logger_(logger)
  {
    n_goals_ = n_goals;
    goals_probabilities_.resize(n_goals_);
    std::fill(goals_probabilities_.begin(), goals_probabilities_.end(), 0.0);
  }

  /**
   * @brief Probability of expanding each goal at the next iteration.
   */
  virtual std::vector<double> getProbabilities() = 0;
  /**
   * @brief Update the policy after goal @p i_goal was expanded.
   * @param i_goal Goal index.
   * @param reward Reward obtained.
   */
  virtual void updateState(const int& i_goal, const double& reward) = 0;
  /**
   * @brief Short description of the policy.
   */
  virtual std::string toString() = 0;

protected:
  int n_goals_ = 0;  ///< Number of goals.
  std::vector<double> goals_probabilities_;  ///< Last computed probabilities.
  std::mt19937 gen_;  ///< Random number generator.
  cnr_logger::TraceLoggerPtr logger_;  ///< Logger.
};
/**
 * @brief Shared pointer to a PolicyBase.
 */
typedef std::shared_ptr<PolicyBase> PolicyBasePtr;


}
