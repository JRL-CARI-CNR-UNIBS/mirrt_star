#pragma once

#include <mirrt_star/multi_goal_selection/policies/policy_mab.h>


namespace multi_goal_selection
{

class PolicyMABEGreedy : public PolicyMAB
{

protected:
  double epsilon_coef_; //epsilon (random play prob) - epsilon_base/(current)t
                      //epsilonbase = cK/d2
  double epsilon_exp_;
  double egreedy_forgetting_factor_;
  double egreedy_reward_forgetting_factor_;
  double reward_gain_;
  std::vector<int> pulled_arms_;

// @MARCO: essendo ros free non ci sono, ti servono? se si occorre pensare a un modo per portarli fuori
//  ros::Publisher eps_pub_ = nh_.advertise<std_msgs::Float64>("eps_greedy", 1000);
//  ros::Publisher reward_max_pub_ = nh_.advertise<std_msgs::Float64>("max_reward", 1000);
//  ros::Publisher reward_second_max_pub_ = nh_.advertise<std_msgs::Float64>("second_max_reward", 1000);
//  std_msgs::Float64 msg_;

public:
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

  virtual std::string toString()
  {
    std::string str="Egreedy Policy with epsilon_coef_=";
    str+=std::to_string(epsilon_coef_);
    return str;
  }

};
typedef std::shared_ptr<PolicyMABEGreedy> PolicyMABEGreedyPtr;

} //namespace
