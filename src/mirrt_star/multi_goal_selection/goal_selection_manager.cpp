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

#include <mirrt_star/multi_goal_selection/goal_selection_manager.h>

//#include <mirrt_star/multi_goal_selection/policies/policy_mab.h>
//#include <mirrt_star/multi_goal_selection/policies/policy_mab_example.h>
//#include <mirrt_star/multi_goal_selection/policies/policy_uniform_on_goals.h>
//#include <mirrt_star/multi_goal_selection/policies/policy_custom_example.h>
//#include <mirrt_star/multi_goal_selection/policies/policy_uniform_on_volume.h>
#include <mirrt_star/multi_goal_selection/policies/policy_mab_egreedy.h>
//#include <mirrt_star/multi_goal_selection/policies/policy_mab_ucb.h>
//#include <mirrt_star/multi_goal_selection/policies/policy_mab_ts.h>
#include <mirrt_star/multi_goal_selection/rewards/reward_relative_improvement.h>
//#include <mirrt_star/multi_goal_selection/rewards/reward_bernoulli.h>
//#include <mirrt_star/multi_goal_selection/rewards/reward_best_cost.h>


namespace multi_goal_selection
{

GoalSelectionManager::GoalSelectionManager(const std::string& name,
                                           const cnr_logger::TraceLoggerPtr& logger,
                                           const unsigned int& n_goals,
                                           const unsigned int& n_dof)
{
  logger_=logger;
  goal_number_ = n_goals;


  get_param(logger_,name,"policy_type",policy_type_,std::string("MultiArmedBandit"));
  get_param(logger_,name,"policy_name",policy_name_,std::string("eGreedy"));
  get_param(logger_,name,"reward_fcn",reward_fcn_name_,std::string("RelativeImprovement"));
  get_param(logger_,name,"warm_start_reward",do_warm_start_,false);

  if (!policy_type_.compare("MultiArmedBandit"))
  {
    CNR_INFO(logger_,"Policy type: " << policy_type_);
    if (!policy_name_.compare("eGreedy"))
    {
      policy_ = std::make_shared<multi_goal_selection::PolicyMABEGreedy>(name,goal_number_,logger_);
      CNR_INFO(logger_,"Policy name: " << policy_name_);
    }
    // @MARCO: UNCOMMENT THIS WHEN HEADERS ARE OK
    //    else if (!policy_name_.compare("UCB1"))
    //    {
    //      policy_ = std::make_shared<multi_goal_selection::PolicyMABUCB>(name,goal_number_,logger_);
    //      CNR_INFO(logger_,"Policy name: " << policy_name_);
    //    }
    //    else if (!policy_name_.compare("Thomson"))
    //    {
    //      policy_ = std::make_shared<multi_goal_selection::PolicyMABTS>(name,goal_number_,logger_);
    //      CNR_INFO(logger_,"Policy name: " << policy_name_);
    //    }
    //    else if (!policy_name_.compare("PolicyUniformOnGoals"))
    //    {
    //      policy_ = std::make_shared<multi_goal_selection::PolicyUniformOnGoals>(name,goal_number_,logger_);
    //      CNR_INFO(logger_,"Policy name: " << policy_name_);
    //    }
    //    else if (!policy_name_.compare("PolicyUniformOnVolume"))
    //    {
    //      policy_ = std::make_shared<multi_goal_selection::PolicyUniformOnVolume>(name,goal_number_,n_dof,logger_);
    //      if (reward_fcn_name_.compare("BestCost"))
    //      {
    //        reward_fcn_name_ = "BestCost";
    //        CNR_WARN(logger_,"Reward fcn automatically set to BestCost because policy required by UniformOnVolume policy.");
    //      }
    //      if (!do_warm_start_)
    //      {
    //        do_warm_start_ = true;
    //        CNR_WARN(logger_,"Warm start automatically set because required by UniformOnVolume policy.");
    //      }
    //      CNR_INFO(logger_,"Policy name: " << policy_name_);
    //    }
    //    else
    //    {
    //      CNR_FATAL(logger_,"unexpected policy_name_ : " << policy_name_);
    //    }
    //  }
    //  else if (!policy_type_.compare("Custom"))
    //  {
    //    CNR_INFO(logger_,"Policy type: " << policy_type_);
    //    if (!policy_name_.compare("Custom1"))
    //    {
    //      policy_ = std::make_shared<multi_goal_selection::PolicyCustomExample>(name,goal_number_,logger_);
    //      CNR_INFO(logger_,"Policy name: " << policy_name_);
    //    }
    //    else
    //    {
    //      CNR_FATAL(logger_,"unexpected policy_name_ : " << policy_name_);
    //    }
  }
  else
  {
    CNR_FATAL(logger_,"unexpected policy_type_ : " << policy_type_);
  }


  if (!reward_fcn_name_.compare("RelativeImprovement"))
  {
    reward_fcn_ = std::make_shared<multi_goal_selection::RewardRelativeImprovement>(logger_);
    CNR_INFO(logger_,"Reward name: RelativeImprovement");
  }
  // @MARCO: UNCOMMENT THIS WHEN HEADERS ARE OK
  //  else if (!reward_fcn_name_.compare("Bernoulli"))
  //  {
  //    reward_fcn_ = std::make_shared<multi_goal_selection::RewardBernoulli>(logger_);
  //    CNR_INFO(logger_,"Reward name: Bernoulli");
  //  }
  //  else if (!reward_fcn_name_.compare("BestCost"))
  //  {
  //    reward_fcn_ = std::make_shared<multi_goal_selection::RewardBestCost>(logger_);
  //    CNR_INFO(logger_,"Reward name: BestCost");
  //  }
  else
  {
    CNR_FATAL(logger_,"unexpected reward_fcn_name_ : " << reward_fcn_name_);
  }
}

std::vector<double> GoalSelectionManager::calculateProbabilities(const std::vector<bool>& were_goals_selected,
                                                                 const std::vector<double>& costs,
                                                                 const std::vector<double>& utopias,
                                                                 const double& best_cost)
{
  double reward = reward_fcn_->getReward(costs,utopias,best_cost);

  for (int i_goal=0; i_goal<goal_number_;i_goal++)
  {
    if (were_goals_selected.at(i_goal))
    {
      policy_->updateState(i_goal,reward);
    }
  }


  goal_probabilities_ = policy_->getProbabilities();
  return goal_probabilities_;
}

void GoalSelectionManager::warmStart(const std::vector<double>& costs, const std::vector<double>& utopias, const double& best_cost)
{
  for (int i_goal=0; i_goal<goal_number_;i_goal++)
  {
    double reward = reward_fcn_->getReward(costs,utopias,utopias.at(i_goal));
    policy_->updateState(i_goal,reward);
  }
}


}
