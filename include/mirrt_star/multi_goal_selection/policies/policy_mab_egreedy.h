/**
 * @file policy_mab_egreedy.h
 * @brief Epsilon-greedy multi-armed bandit policy.
 */
#pragma once

#include <mirrt_star/multi_goal_selection/policies/policy_mab.h>


namespace multi_goal_selection
{

/**
 * @brief Epsilon-greedy multi-armed bandit policy.
 *
 * With probability epsilon a random arm is pulled, otherwise the arm with the highest
 * expected reward (arms never pulled are tried first).
 *
 * Parameters read under the policy namespace:
 * - @c epsilon_coef (double, default 0.2): exploration probability epsilon;
 * - @c epsilon_exp (double, default 1.0): if < 1, epsilon is multiplied by this value at every update;
 * - @c reward_gain (double, default 1.0): gain used by the adaptive epsilon;
 * - @c egreedy_forgetting_factor (double, default 0.0): adaptive epsilon (currently forced to 0);
 * - @c egreedy_reward_forgetting_factor (double, default 0.0): if > 0, expected rewards are an
 *   exponential moving average; otherwise they are the sample mean.
 */
class PolicyMABEGreedy : public PolicyMAB
{

protected:
  double epsilon_coef_; //epsilon (random play prob) - epsilon_base/(current)t  ///< Exploration probability.
                      //epsilonbase = cK/d2
  double epsilon_exp_;  ///< Decay factor of epsilon.
  double egreedy_forgetting_factor_;  ///< Forgetting factor of the adaptive epsilon.
  double egreedy_reward_forgetting_factor_;  ///< Forgetting factor of the expected rewards.
  double reward_gain_;  ///< Reward gain of the adaptive epsilon.
  std::vector<int> pulled_arms_;  ///< Unused.

// @MARCO: essendo ros free non ci sono, ti servono? se si occorre pensare a un modo per portarli fuori
//  ros::Publisher eps_pub_ = nh_.advertise<std_msgs::Float64>("eps_greedy", 1000);
//  ros::Publisher reward_max_pub_ = nh_.advertise<std_msgs::Float64>("max_reward", 1000);
//  ros::Publisher reward_second_max_pub_ = nh_.advertise<std_msgs::Float64>("second_max_reward", 1000);
//  std_msgs::Float64 msg_;

public:
  /**
   * @brief Constructor. Reads the parameters listed in the class description.
   * @param name Parameter namespace.
   * @param n_goals Number of arms.
   * @param logger Logger.
   */
  PolicyMABEGreedy(const std::string& name,
                   const int& n_goals,
                   const cnr_logger::TraceLoggerPtr &logger) :
    PolicyMAB(name, n_goals,logger)
  {
    pull_counter_ = std::vector<int>(n_goals_, 0);
    expected_reward_ = std::vector<double>(n_goals_, 0.0);


    get_param(logger_,name,"epsilon_coef",epsilon_coef_,0.2);
    get_param(logger_,name,"reward_gain",reward_gain_,1.0);
    get_param(logger_,name,"egreedy_forgetting_factor",egreedy_forgetting_factor_,0.0);
    get_param(logger_,name,"egreedy_reward_forgetting_factor",egreedy_reward_forgetting_factor_,0.0);
    get_param(logger_,name,"epsilon_exp",epsilon_exp_,1.0);

    if(egreedy_forgetting_factor_!=0.0)
    {
      CNR_ERROR(logger_,"egreedy_forgetting_factor_ > 0 and epsilon_exp < 1. Setting egreedy_forgetting_factor_=0.");
      egreedy_forgetting_factor_=0.0;
    }

    CNR_WARN(logger_,"E-greedy initialized with epsilon=%f, "
             "reward_gain=%f, "
             "egreedy_forgetting_factor_=%f, "
             "egreedy_reward_forgetting_factor_=%f, "
             "epsilon_exp_=%f",
             epsilon_coef_,
             reward_gain_,
             egreedy_forgetting_factor_,
             egreedy_reward_forgetting_factor_,
             epsilon_exp_);

  }
  
  /**
   * @brief Epsilon-greedy arm selection.
   */
  virtual int selectNextArm()
  {

    double rand = std::uniform_real_distribution<double>(0.0,1.0)(gen_);
    if(epsilon_coef_ > rand){ //random choice
      return std::uniform_int_distribution<int>(0, n_goals_-1)(gen_);
    }
    else
    {
      double max_expectation=-std::numeric_limits<double>::infinity();
      unsigned int i_goal_best=0;
      for(int i_goal=0; i_goal<n_goals_; i_goal++)
      {
        if(pull_counter_[i_goal]==0)
          return i_goal;

        if (expected_reward_[i_goal] > max_expectation)
        {
          max_expectation = expected_reward_[i_goal];
          i_goal_best = i_goal;
        }
      }
      return i_goal_best;
    }
  }

  /**
   * @brief Update the expected reward of the pulled arm and decay epsilon.
   * @param i_goal Pulled arm.
   * @param reward Reward obtained.
   */
  virtual void updateState(const int& i_goal, const double& reward)
  {
    if (epsilon_exp_<1.0)
      epsilon_coef_=epsilon_coef_*epsilon_exp_;

    if (egreedy_reward_forgetting_factor_<=0.0)
    {
      pull_counter_[i_goal]+=1;
      expected_reward_[i_goal]+=(reward-expected_reward_[i_goal])/double(pull_counter_[i_goal]);
    }
    else
    {
      for (unsigned int idx=0;idx<expected_reward_.size();idx++)
        expected_reward_[idx]=(1-egreedy_reward_forgetting_factor_)*expected_reward_[idx];
      expected_reward_[i_goal]+=egreedy_reward_forgetting_factor_*reward;
    }
    if (egreedy_forgetting_factor_>0.0 && epsilon_exp_==1.0)
    {
      epsilon_coef_=1-std::min((1-egreedy_forgetting_factor_)*(1-epsilon_coef_)+egreedy_forgetting_factor_*reward_gain_*reward,1.0);
    }

    /* @MARCO: essendo ros free non ci sono, ti servono? se si occorre pensare a un modo per portarli fuori

    // publish to topic

    msg_.data = epsilon_coef_;
    eps_pub_.publish(msg_);


    std::vector<double> tmp = expected_reward_;
    if (tmp.size()>=2)
    {
      msg_.data =  *std::max_element(tmp.begin(),tmp.end());
      reward_max_pub_.publish(msg_);
      std::sort(tmp.begin(),tmp.end(),std::greater<int>());

      msg_.data =  tmp[1];
      reward_second_max_pub_.publish(msg_);
    }
    */
  }

  /**
   * @brief Policy name and current epsilon.
   */
  virtual std::string toString()
  {
    std::string str="Egreedy Policy with epsilon_coef_=";
    str+=std::to_string(epsilon_coef_);
    return str;
  }

};
/**
 * @brief Shared pointer to a PolicyMABEGreedy.
 */
typedef std::shared_ptr<PolicyMABEGreedy> PolicyMABEGreedyPtr;

} //namespace
