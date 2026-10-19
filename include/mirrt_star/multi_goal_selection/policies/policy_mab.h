/**
 * @file policy_mab.h
 * @brief Base class of the multi-armed bandit (MAB) goal selection policies.
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
#include <mirrt_star/multi_goal_selection/goal_selection_util.h>

namespace multi_goal_selection
{

/**
 * @brief Base class of the multi-armed bandit policies.
 *
 * Each goal is an arm. At every iteration exactly one arm, chosen by
 * selectNextArm(), gets probability 1 and the others get 0.
 */
class PolicyMAB: public PolicyBase
{
public:
  /**
   * @brief Constructor.
   * @param name Parameter namespace of the policy.
   * @param n_goals Number of goals (arms).
   * @param logger Logger.
   */
  PolicyMAB(const std::string& name, const int& n_goals,
            const cnr_logger::TraceLoggerPtr &logger) :
    PolicyBase(name, n_goals,logger)
  {
    pull_counter_.resize(n_goals);
    std::fill(pull_counter_.begin(), pull_counter_.end(), 0);
    expected_reward_.resize(n_goals);
    std::fill(expected_reward_.begin(), expected_reward_.end(), 0.0);
  }

  /**
   * @brief Select one arm and return a one-hot probability vector.
   */
  std::vector<double> getProbabilities()
  {
    std::fill(goals_probabilities_.begin(), goals_probabilities_.end(), 0.0);
    int arm = selectNextArm();
    goals_probabilities_.at(arm) = 1.0;
    return goals_probabilities_;
  }

  /**
   * @brief Choose the arm to pull at the next iteration.
   */
  virtual int selectNextArm() = 0;

  /**
   * @brief Reset the expected rewards to given values.
   * @param rewards Initial reward of each arm (size n_goals).
   * @param std_devs Standard deviation of each arm (unused here).
   * @return False if the vector size is wrong.
   */
  virtual bool reinitRewards(std::vector<double> rewards, std::vector<double> std_devs)
  {
    if (rewards.size() != (std::size_t)n_goals_)
    {
      CNR_FATAL(logger_, "Wrong size of vector rewards.");
      return false;
    }
    initial_reward_ = rewards;
    expected_reward_ = initial_reward_;

    std::fill(pull_counter_.begin(), pull_counter_.end(), 1);

    return true;
  }

  /**
   * @brief Update with a reward and its variance. The base version ignores the variance.
   * @param i_goal Arm index.
   * @param reward Reward.
   * @param variance Reward variance.
   */
  virtual void updateState(const int& i_goal, const double& reward, const double& variance)
  {
    updateState(i_goal, reward);
  }

  /**
   * @brief Default update: increment the pull counter and accumulate the reward.
   */
  virtual void updateState(const int& i_goal, const double& reward)
  {
    pull_counter_[i_goal]+=1;
    expected_reward_[i_goal]+=reward;
  }

  /**
   * @brief Short description of the policy.
   */
  virtual std::string toString() = 0;

  /**
   * @brief Print rewards and pull counters on stdout.
   */
  virtual void print()
  {
    std::cout << toString().c_str() << "\n";
    std::cout << "Number of arms: " << n_goals_ << "\n";
    std::cout << "Rewards: ";
    for (int idx=0;idx<n_goals_;idx++)
    {
      std::cout << expected_reward_[idx] << ", ";
    }
    std::cout << "\n";
    std::cout << "Pull counters: ";
    for (int idx=0;idx<n_goals_;idx++)
    {
      std::cout << pull_counter_[idx] << ", ";
    }
    std::cout << "\n";
  }

  /**
   * @brief Expected reward of each arm.
   */
  std::vector<double> getExpectedRewards(){return expected_reward_;}

  /**
   * @brief Reward variance of each arm (not implemented in the base class).
   */
  virtual std::vector<double> getVariance()
  {
    std::cout << "NOT IMPLEMENTED" << std::endl;
    return std::vector<double>(0);
  }


  /**
   * @brief Number of pulls of each arm.
   */
  std::vector<int> getPullCounters(){return pull_counter_;}

protected:
  std::vector<int> pull_counter_;  ///< Number of pulls of each arm.
  std::vector<double> expected_reward_;  ///< Expected (or accumulated) reward of each arm.
  std::vector<double> initial_reward_;  ///< Rewards set by reinitRewards().


};
/**
 * @brief Shared pointer to a PolicyMAB.
 */
typedef std::shared_ptr<PolicyMAB> PolicyMABPtr;

}
