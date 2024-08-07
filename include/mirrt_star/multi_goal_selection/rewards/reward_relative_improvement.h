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

#include <mirrt_star/multi_goal_selection/rewards/reward_base.h>

namespace multi_goal_selection
{

class RewardRelativeImprovement: public RewardBase
{
public:
  RewardRelativeImprovement(const cnr_logger::TraceLoggerPtr &logger):
    RewardBase(logger)
  {};

  double getReward(const std::vector<double>& costs, const std::vector<double>& utopias, const double& best_cost)
  {
    double reward = 0.0;
    if (last_best_cost_!=std::numeric_limits<double>::infinity())
      reward = 100*(last_best_cost_ - best_cost)/last_best_cost_; // reward \in [0,1] assuming best_cost>=0 and best_cost<=last_best_cost_

    last_best_cost_ = best_cost;
    return reward;

  };

};
typedef std::shared_ptr<RewardRelativeImprovement> RewardRelativeImprovementPtr;


}
