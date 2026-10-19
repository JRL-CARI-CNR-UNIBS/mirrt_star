/**
 * @file policy_mab_ts.h
 * @brief Thompson sampling multi-armed bandit policy.
 * @warning Not yet ported from ROS1 (uses ros::NodeHandle): it does not compile and is not used
 * by GoalSelectionManager.
 */
#pragma once

#include <mirrt_star/multi_goal_selection/policies/policy_mab.h>
#include <mirrt_star/multi_goal_selection/distributions.h>

namespace multi_goal_selection
{

/**
 * @brief Thompson sampling with Beta priors and Bernoulli rewards (reward > 0.5 counts as a success).
 *
 * Parameters: @c ts_alpha and @c ts_beta (double, default 1.0), the prior of each arm.
 */
class PolicyMABTS : public PolicyMAB
{

protected:
  std::vector<double> alphas_, betas_, thetas_;

public:
  PolicyMABTS(const std::string& name, const int& n_goals) : PolicyMAB(name, n_goals)
  {
    pull_counter_ = std::vector<int>(n_goals_, 0);
    expected_reward_ = std::vector<double>(n_goals_, 0.0);

    double alpha=1.0;
    if (!nh_.getParam("ts_alpha", alpha))
    {
      CNR_DEBUG(logger_,"%s/ts_alpha is not set. Deafult: 1.0",nh_.getNamespace().c_str());
    }

    double beta=1.0;
    if (!nh_.getParam("ts_beta", beta))
    {
      CNR_DEBUG(logger_,"%s/ts_beta is not set. Deafult: 1.0",nh_.getNamespace().c_str());
    }

    alphas_ = std::vector<double>(n_goals_, alpha);
    betas_  = std::vector<double>(n_goals_, beta );
    thetas_ = std::vector<double>(n_goals_, 0.0);

  }
  
  virtual int selectNextArm()
  {
    for(int idx=0; idx<n_goals_; idx++)
    {
      thetas_[idx] = beta_distribution<double>(alphas_[idx], betas_[idx])(gen_) ;
    }
    return vectorMaxIndex(thetas_);
  }

  virtual void updateState(const int& i_goal, const double& reward)
  {
    pull_counter_[i_goal]+=1;
    if (reward>0.5)
      alphas_[i_goal]+=1;
    else
      betas_[i_goal] +=1;
  }

  virtual std::string toString()
  {
    std::string str="Thomson Sampling Policy";
    return str;
  }

};
/**
 * @brief Shared pointer to a PolicyMABTS.
 */
typedef std::shared_ptr<PolicyMABTS> PolicyMABTSPtr;

} //namespace
