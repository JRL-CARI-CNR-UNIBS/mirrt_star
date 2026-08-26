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

#include <mirrt_star/solvers/mirrt_star.h>

namespace graph
{
  namespace mirrt_star
  {

    /**
     * Adds a start node to the multigoal solver.
     *
     * @param start_node The start node to be added.
     * @param max_time The maximum time allowed for solving.
     *
     * @returns True if the start node was successfully added, false otherwise.
     */
    bool MultigoalSolver::addStart(const NodePtr &start_node, const double &max_time)
    {
      if (!configured_)
      {
        CNR_ERROR(logger_, "Solver is not configured.");
        return false;
      }
      start_tree_ = std::make_shared<Tree>(start_node, max_distance_, checker_, metrics_, logger_, use_kdtree_);
      solved_ = false;

      setProblem(max_time);
      CNR_WARN(logger_, "[MIRRT* Setup] Start configuration set: [ " << start_node->getConfiguration().transpose() << " ]");
      return true;
    }

    bool MultigoalSolver::addStartTree(const TreePtr &start_tree, const double &max_time)
    {
      assert(start_tree);
      start_tree_ = start_tree;
      solved_ = false;
      setProblem(max_time);
      CNR_WARN(logger_, "[MIRRT* Setup] Start tree initialized with " << start_tree_->getNumberOfNodes() << " nodes.");
      return true;
    }

    bool MultigoalSolver::addGoal(const NodePtr &goal_node, const double &max_time, const double &apple_cost)
    {
      if (!start_tree_)
      {
        CNR_ERROR(logger_, "You have to specify first the start goal");
        return false;
      }

      double goal_cost = goal_cost_fcn_->cost(goal_node) + apple_cost;
      double utopia = goal_cost;
      utopia += metrics_->utopia(start_tree_->getRoot(), goal_node);
      unsigned int index = goal_nodes_.size();

      if ((utopia) > cost_)
      {
        CNR_WARN(logger_, "[MIRRT* Goal Registration] Goal %u skipped: utopia cost (%f) is worse than current best solution cost (%f).", index, utopia, cost_);
        return false;
      }

      if (!checker_->check(goal_node->getConfiguration()))
      {
        CNR_WARN(logger_, "[MIRRT* Goal Registration] Goal %u skipped: configuration is in collision.", index);
        return false;
      }

      if (utopia < best_utopia_)
        best_utopia_ = utopia;

      PathPtr solution;
      double path_cost = std::numeric_limits<double>::infinity();
      double cost = std::numeric_limits<double>::infinity();
      TreePtr goal_tree;
      GoalStatus status;
      NodePtr new_node;

      // TODO IF REALISE O BENCHMARK
      if (start_tree_->connectToNode(goal_node, new_node, max_time))
      {

        solution = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node), metrics_, checker_, logger_);
        solution->setTree(start_tree_);

        path_cost = solution->cost();
        cost = path_cost + goal_cost;

        solved_ = true;

        if (cost <= (utopia * utopia_tolerance_))
        {
          CNR_WARN(logger_, "[MIRRT* Goal Registration] Goal %u: Direct connection reached utopia tolerance! Cost: %f (Path: %f, Goal: %f, Utopia: %f) -> Status: DONE", index, cost, path_cost, goal_cost, utopia);
          status = GoalStatus::done;
        }
        else
        {
          CNR_WARN(logger_, "[MIRRT* Goal Registration] Goal %u: Direct connection found! Cost: %f (Path: %f, Goal: %f, Utopia: %f) -> Status: REFINE", index, cost, path_cost, goal_cost, utopia);
          status = GoalStatus::refine;
        }

        CNR_DEBUG(logger_, "A direct solution is found\n"
                               << *solution);
      }
      else
      {

        path_cost = cost = std::numeric_limits<double>::infinity();
        goal_tree = std::make_shared<Tree>(goal_node, max_distance_, checker_, metrics_, logger_, use_kdtree_);
        status = GoalStatus::search;
        CNR_WARN(logger_, "[MIRRT* Goal Registration] Goal %u: No direct connection. Added to search tree -> Status: SEARCH (Exploring space, Goal cost: %f, Utopia: %f)", index, goal_cost, utopia);
      }

      // da cnr_class_loader
      SamplerPtr sampler = std::make_shared<InformedSampler>(start_tree_->getRoot()->getConfiguration(),
                                                             goal_node->getConfiguration(),
                                                             sampler_->getLB(),
                                                             sampler_->getUB(),
                                                             logger_);

      TubeInformedSamplerPtr tube_sampler = std::make_shared<TubeInformedSampler>(sampler, metrics_);
      tube_sampler->setLocalBias(local_bias_);
      tube_sampler->setRadius(tube_radius_);
      if (solution)
        tube_sampler->setPath(solution);

      goal_nodes_.push_back(goal_node);
      goal_trees_.push_back(goal_tree);
      path_costs_.push_back(path_cost);
      goal_costs_.push_back(goal_cost);
      costs_.push_back(cost);
      utopias_.push_back(utopia);
      solutions_.push_back(solution);
      tube_samplers_.push_back(tube_sampler);
      samplers_.push_back(sampler);
      status_.push_back(status);
      were_goals_sampled_.push_back(false);
      goal_probabilities_.push_back(1.0);

      if (isBestSolution(goal_nodes_.size() - 1))
        CNR_DEBUG(logger_, "Goal " << goal_node->getConfiguration().transpose() << " is the best one with a cost = " << cost);

      initialized_ = true;
      return true;
    }

    bool MultigoalSolver::isBestSolution(const int &index)
    {
      assert(status_.at(index) != GoalStatus::discard);

      if (costs_.at(index) >= (cost_ - 1e-8))
        return false;
      double previous_best_cost = cost_;
      path_cost_ = path_costs_.at(index);
      goal_cost_ = goal_costs_.at(index);
      cost_ = costs_.at(index);
      best_goal_index = index;
      solution_ = solutions_.at(index);

      CNR_WARN(logger_, "[MIRRT* Solution Update] >>> NEW GLOBAL BEST SOLUTION! Goal %d | Total Cost: %f (Path Cost: %f, Goal Cost: %f, Previous Best: %f, Utopia: %f)",
               best_goal_index, cost_, path_cost_, goal_cost_, previous_best_cost, utopias_.at(index));

      for (unsigned int igoal = 0; igoal < goal_nodes_.size(); igoal++)
      {

        if (status_.at(igoal) == GoalStatus::discard)
          continue;
        if (utopias_.at(igoal) > cost_)
        {
          status_.at(igoal) = GoalStatus::discard;
          CNR_WARN(logger_, "[MIRRT* Pruning] Goal %u DISCARDED: Utopia (%f) > Current best cost (%f)", igoal, utopias_.at(igoal), cost_);
          cleanTree();
          continue;
        }
        if (status_.at(igoal) == GoalStatus::done)
          continue;
        if (mixed_strategy_)
        {
          tube_samplers_.at(igoal)->setCost(path_cost_ + goal_cost_ - goal_costs_.at(igoal));
          samplers_.at(igoal)->setCost(path_cost_ + goal_cost_ - goal_costs_.at(igoal));
        }
        else
        {
          samplers_.at(igoal)->setCost(path_cost_ + goal_cost_ - goal_costs_.at(igoal));
        }
      }

      if ((cost_ < 0.9999 * cost_at_last_clean) || start_tree_->needCleaning())
      {
        cost_at_last_clean = cost_;
        cleanTree();
      }

      return true;
    }

    void MultigoalSolver::resetProblem()
    {
      goal_nodes_.clear();
      goal_trees_.clear();
      path_costs_.clear();
      goal_costs_.clear();
      utopias_.clear();
      costs_.clear();
      were_goals_sampled_.clear();
      goal_probabilities_.clear();
      solutions_.clear();
      tube_samplers_.clear();
      samplers_.clear();
      status_.clear();
      start_tree_.reset();
      solved_ = false;
      can_improve_ = true;
      best_utopia_ = std::numeric_limits<double>::infinity();
      path_cost_ = std::numeric_limits<double>::infinity();
      goal_cost_ = std::numeric_limits<double>::infinity();
      cost_ = std::numeric_limits<double>::infinity();
      cost_at_last_clean = std::numeric_limits<double>::infinity();
      iter_ = 0;
    }

    bool MultigoalSolver::finalizeProblem()
    {
      bool res = initGoalSelector();
      CNR_WARN(logger_, "[MIRRT* Setup] Finalized problem with %zu candidate goals. Initial best cost: %f (Best Goal: %d, Utopia tolerance: %.2f)",
               goal_nodes_.size(), cost_, best_goal_index, utopia_tolerance_);
      return res;
    }

    bool MultigoalSolver::setProblem(const double &max_time)
    {
      if (!start_tree_)
        return false;
      if (goal_nodes_.size() == 0)
        return false;

      return true;
    }

    bool MultigoalSolver::config(const std::string &param_ns)
    {
      if (not TreeSolver::config(param_ns))
        return false;

      get_param(logger_, param_ns_, "rewire_radius", r_rewire_, 2.0 * max_distance_);
      get_param(logger_, param_ns_, "mixed_strategy", mixed_strategy_, true);
      get_param(logger_, param_ns_, "bidirectional", bidirectional_, true);
      get_param(logger_, param_ns_, "k_nearest", knearest_, false);
      get_param(logger_, param_ns_, "local_bias", local_bias_, 0.3);
      get_param(logger_, param_ns_, "apple_weight", apple_weight_, 10.0);

      if (local_bias_ < 0)
      {
        CNR_WARN(logger_, "local_bias cannot be negative, set equal to 0");
        local_bias_ = 0.0;
      }
      if (local_bias_ > 1)
      {
        CNR_WARN(logger_, "local_bias cannot be greater than 1.0, set equal to 1.0");
        local_bias_ = 1.0;
      }

      get_param(logger_, param_ns_, "tube_radius", tube_radius_, 0.3);

      if (tube_radius_ <= 0)
      {
        CNR_WARN(logger_, "%s/tube_radius cannot be negative, set equal to 0.01");
        tube_radius_ = 0.01;
      }

      get_param(logger_, param_ns_, "forgetting_factor", forgetting_factor_, 0.01);

      if (forgetting_factor_ <= 0)
      {
        CNR_WARN(logger_, "forgetting_factor cannot be negative, set equal to 0.0");
        forgetting_factor_ = 0.0;
      }

      configured_ = true;
      return true;
    }

    bool MultigoalSolver::initGoalSelector()
    {
      goal_manager_ = std::make_shared<multi_goal_selection::GoalSelectionManager>(param_ns_,
                                                                                   logger_,
                                                                                   goal_nodes_.size(),
                                                                                   sampler_->getDimension());
      if (goal_manager_->isWarmStartSet())
        goal_manager_->warmStart(costs_, utopias_, cost_);
      return true;
    }

    bool MultigoalSolver::update(PathPtr &solution)
    {
      if (!initialized_)
      {
        CNR_WARN(logger_, "MultigoalSolver is not initialized");
        return false;
      }
      if (cost_ <= utopia_tolerance_ * best_utopia_)
      {
        CNR_WARN(logger_, "[MIRRT* Convergence] Global best cost (%f) reached utopia threshold (%f * %f = %f). Solution cannot be improved further!",
                 cost_, utopia_tolerance_, best_utopia_, utopia_tolerance_ * best_utopia_);
        solution = solution_;
        can_improve_ = false;
        return false;
      }

      iter_++;

      if (iter_ == 1 || iter_ % 500 == 0)
      {
        unsigned int n_search = 0, n_refine = 0, n_done = 0, n_discard = 0;
        for (const auto &st : status_)
        {
          if (st == GoalStatus::search)
            n_search++;
          else if (st == GoalStatus::refine)
            n_refine++;
          else if (st == GoalStatus::done)
            n_done++;
          else if (st == GoalStatus::discard)
            n_discard++;
        }
        CNR_WARN(logger_, "[MIRRT* Iter " << iter_ << "] Status -> SEARCH (exploring): " << n_search
                                          << ", REFINE (optimizing): " << n_refine
                                          << ", DONE (optimal): " << n_done
                                          << ", DISCARDED: " << n_discard
                                          << " | Tree Nodes: " << (start_tree_ ? start_tree_->getNumberOfNodes() : 0)
                                          << " | Best Cost: " << cost_ << " (Goal " << best_goal_index << ")"
                                          << " | Local Bias: " << local_bias_);
      }

      //  CNR_INFO(logger_,"update probabilities");

      if (goal_probabilities_.size() == 1)
        goal_probabilities_.at(0) = 1.0;
      else
        goal_probabilities_ = goal_manager_->calculateProbabilities(were_goals_sampled_, costs_, utopias_, cost_);

      std::fill(were_goals_sampled_.begin(), were_goals_sampled_.end(), false);

      bool global_improvement = false;
      double old_cost = cost_;

      for (unsigned int igoal = 0; igoal < goal_nodes_.size(); igoal++)
      {
        if (utopias_.at(igoal) > cost_)
          continue;
        if (goal_probabilities_.at(igoal) <= 0.0)
          continue;
        else if (goal_probabilities_.at(igoal) >= 1.0)
          ;
        else if (ud_(gen_) > goal_probabilities_.at(igoal))
          continue;

        were_goals_sampled_.at(igoal) = true;

        NodePtr new_start_node, new_goal_node;
        bool add_to_start, add_to_goal;
        Eigen::VectorXd configuration;
        if (mixed_strategy_)
          configuration = tube_samplers_.at(igoal)->sample();
        else
          configuration = samplers_.at(igoal)->sample();
        bool is_goal_biased = ud_(gen_) < goal_bias_;

        bool improved = false;

        if (costs_.at(igoal) < utopias_.at(igoal))
        {
          CNR_ERROR(logger_, "GOAL %d, COST %f, UTOPIA %f", igoal, costs_.at(igoal), utopias_.at(igoal));
          solved_ = false;
          return false;
        }

        switch (status_.at(igoal))
        {

        case GoalStatus::search:

          if (not bidirectional_) // if only start tree grows
          {
            if (is_goal_biased) // if it is goal biased try to connect goal
            {
              if (extend_)
                add_to_start = start_tree_->extendToNode(goal_nodes_.at(igoal), new_start_node);
              else
                add_to_start = start_tree_->connectToNode(goal_nodes_.at(igoal), new_start_node);

              if (new_start_node == goal_nodes_.at(igoal))
              {
                improved = true;
                goal_trees_.at(igoal)->cleanTree();
              }
            }
            else // not is_goal_bias, add a new random node
            {
              if (extend_)
                add_to_start = start_tree_->extend(configuration, new_start_node);
              else
                add_to_start = start_tree_->connect(configuration, new_start_node);
              if (add_to_start) // if a new node is added, check if it near to the goal
              {
                if (metrics_->utopia(new_start_node, goal_nodes_.at(igoal)) < max_distance_)
                {
                  // if yes, try to extend to the goal
                  add_to_start = start_tree_->extendToNode(goal_nodes_.at(igoal), new_start_node);
                  if (new_start_node == goal_nodes_.at(igoal)) // a solution is found
                  {
                    improved = true;
                    goal_trees_.at(igoal)->cleanTree();
                  }
                }
              }
            }
          }
          else // birectional
          {
            if (is_goal_biased) // if it is goal biased try to connect goal
            {
              if (extend_)
                add_to_start = start_tree_->extendToNode(goal_nodes_.at(igoal), new_start_node);
              else
                add_to_start = start_tree_->connectToNode(goal_nodes_.at(igoal), new_start_node);

              if (new_start_node == goal_nodes_.at(igoal))
              {
                improved = true;
                goal_trees_.at(igoal)->cleanTree();
              }
            }
            else // not is_goal_bias, add a new random node
            {
              // add node to the start tree
              add_to_start = extend_ ? start_tree_->extend(configuration, new_start_node) : start_tree_->connect(configuration, new_start_node);

              add_to_start = add_to_start ? (configuration - new_start_node->getConfiguration()).norm() < 1e-6 : false;

              add_to_goal = extend_ ? goal_trees_.at(igoal)->extend(configuration, new_goal_node) : goal_trees_.at(igoal)->connect(configuration, new_goal_node);

              add_to_goal = add_to_goal ? (configuration - new_goal_node->getConfiguration()).norm() < 1e-6 : false;

              if (add_to_start && add_to_goal) // a solution is found
              {
                NodePtr parent = new_goal_node->getParents().at(0);
                double cost_to_parent = new_goal_node->parentConnection(0)->getCost();
                new_goal_node->disconnect();

                // goal_trees_.at(igoal)->keepOnlyThisBranch(goal_trees_.at(igoal)->getConnectionToNode(parent));

                std::vector<ConnectionPtr> connections = goal_trees_.at(igoal)->getConnectionToNode(parent);
                for (ConnectionPtr &conn : connections)
                  conn->flip();

                ConnectionPtr conn_to_goal_parent = std::make_shared<Connection>(new_start_node, parent, logger_);
                conn_to_goal_parent->add();
                conn_to_goal_parent->setCost(cost_to_parent);
                start_tree_->addNode(parent, false);
                for (ConnectionPtr &conn : connections)
                  start_tree_->addNode(conn->getChild(), false);
                improved = true;
              }
            }
          }

          if (improved)
          {
            solutions_.at(igoal) = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_nodes_.at(igoal)), metrics_, checker_, logger_);
            solutions_.at(igoal)->setTree(start_tree_);

            double cost_1 = solutions_.at(igoal)->cost();

            //        if (first_warp_)
            //        {
            //          solutions_.at(igoal)->warp();
            //          double cost_0=solutions_.at(igoal)->cost();
            //          solutions_.at(igoal)->simplify();
            //        }
            tube_samplers_.at(igoal)->setPath(solutions_.at(igoal));
            tube_samplers_.at(igoal)->setRadius(tube_radius_ * solutions_.at(igoal)->cost());
            path_costs_.at(igoal) = solutions_.at(igoal)->cost();
            costs_.at(igoal) = path_costs_.at(igoal) + goal_costs_.at(igoal);

            solved_ = true;
            if (costs_.at(igoal) <= (utopias_.at(igoal) * utopia_tolerance_))
            {
              CNR_WARN(logger_, "[MIRRT* Iter " << iter_ << "] Goal " << igoal << " (EXPLORING): First valid path found! Reached Utopia -> Cost: " << costs_.at(igoal) << " (Path: " << path_costs_.at(igoal) << ", Apple: " << goal_costs_.at(igoal) << ", Utopia: " << utopias_.at(igoal) << ") -> Status: DONE");
              status_.at(igoal) = GoalStatus::done;
            }
            else
            {
              CNR_WARN(logger_, "[MIRRT* Iter " << iter_ << "] Goal " << igoal << " (EXPLORING): First valid path found! Cost: " << costs_.at(igoal) << " (Path: " << path_costs_.at(igoal) << ", Apple: " << goal_costs_.at(igoal) << ", Utopia: " << utopias_.at(igoal) << ") -> Status: REFINE (Now optimizing)");
              status_.at(igoal) = GoalStatus::refine;
            }
            global_improvement = isBestSolution(igoal) || global_improvement;
          }

          break;
        case GoalStatus::refine:

          if (not knearest_)
          {
            //        double r_rrt=1.1*  std::pow(2.0*(1.0+1.0/dimension_),1.0/dimension_)*std::pow(sampler_->getSpecificVolume(),1.0/dimension_);
            //        double cardDbl=start_tree_->getNumberOfNodes()+1.0;
            //        r_rewire_=r_rrt * std::pow(log(cardDbl) / cardDbl, 1.0 /dimension_);
            improved = start_tree_->rewire(configuration, r_rewire_, new_start_node);
          }
          else
          {
            improved = start_tree_->rewire(configuration, -1); // by setting the rewiring radius <=0 the nearest nodes considered are the nearest K neighbours
          }
          if (improved)
          {
            if (start_tree_->costToNode(goal_nodes_.at(igoal)) >= (path_costs_.at(igoal) - 1e-8))
              continue;

            solutions_.at(igoal) = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_nodes_.at(igoal)), metrics_, checker_, logger_);
            solutions_.at(igoal)->setTree(start_tree_);
            double cost_1 = solutions_.at(igoal)->cost();
            //        if (warp_)
            //        {
            //          solutions_.at(igoal)->warp();
            //          solutions_.at(igoal)->simplify();
            //        }

            tube_samplers_.at(igoal)->setPath(solutions_.at(igoal));
            tube_samplers_.at(igoal)->setRadius(tube_radius_ * solutions_.at(igoal)->cost());
            double old_goal_cost = costs_.at(igoal);
            path_costs_.at(igoal) = solutions_.at(igoal)->cost();
            costs_.at(igoal) = path_costs_.at(igoal) + goal_costs_.at(igoal);
            if (costs_.at(igoal) <= (utopias_.at(igoal) * utopia_tolerance_))
            {
              CNR_WARN(logger_, "[MIRRT* Iter " << iter_ << "] Goal " << igoal << " (REFINING): Rewiring improved cost: " << old_goal_cost << " -> " << costs_.at(igoal) << " (Path: " << path_costs_.at(igoal) << ", Utopia: " << utopias_.at(igoal) << ") -> Status: DONE (Utopia reached)");
              cleanTree();
              status_.at(igoal) = GoalStatus::done;
            }
            else
            {
              CNR_WARN(logger_, "[MIRRT* Iter " << iter_ << "] Goal " << igoal << " (REFINING): Rewiring improved cost: " << old_goal_cost << " -> " << costs_.at(igoal) << " (Path: " << path_costs_.at(igoal) << ", Apple: " << goal_costs_.at(igoal) << ", Utopia: " << utopias_.at(igoal) << ")");
            }
            global_improvement = isBestSolution(igoal) || global_improvement;
          }
          else if (new_start_node) // is an improvement?????
          {
            // try connect with the goal
            double cost_to_goal = metrics_->cost(new_start_node, goal_nodes_.at(igoal));
            if (cost_to_goal < max_distance_ && (start_tree_->costToNode(new_start_node) + cost_to_goal) < path_costs_.at(igoal))
            {
              if (checker_->checkConnection(new_start_node->getConfiguration(),
                                            goal_nodes_.at(igoal)->getConfiguration()))
              {
                goal_nodes_.at(igoal)->parentConnection(0)->remove();
                ConnectionPtr conn = std::make_shared<Connection>(new_start_node, goal_nodes_.at(igoal), logger_);
                conn->setCost(cost_to_goal);
                conn->add();
                solutions_.at(igoal) = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_nodes_.at(igoal)), metrics_, checker_, logger_);
                solutions_.at(igoal)->setTree(start_tree_);
                double cost_1 = solutions_.at(igoal)->cost();
                //            if (warp_)
                //            {
                //              for (int iwarp=0;iwarp<10;iwarp++)
                //              {
                //                solutions_.at(igoal)->warp();
                //              }
                //              double cost_0=solutions_.at(igoal)->cost();
                //              solutions_.at(igoal)->simplify();
                //            }

                tube_samplers_.at(igoal)->setPath(solutions_.at(igoal));
                tube_samplers_.at(igoal)->setRadius(tube_radius_ * solutions_.at(igoal)->cost());
                double old_goal_cost = costs_.at(igoal);
                path_costs_.at(igoal) = solutions_.at(igoal)->cost();
                costs_.at(igoal) = path_costs_.at(igoal) + goal_costs_.at(igoal);
                if (costs_.at(igoal) <= (utopias_.at(igoal) * utopia_tolerance_))
                {
                  CNR_WARN(logger_, "[MIRRT* Iter " << iter_ << "] Goal " << igoal << " (REFINING): Direct connect improved cost: " << old_goal_cost << " -> " << costs_.at(igoal) << " (Path: " << path_costs_.at(igoal) << ", Utopia: " << utopias_.at(igoal) << ") -> Status: DONE (Utopia reached)");
                  cleanTree();
                  status_.at(igoal) = GoalStatus::done;
                }
                else
                {
                  CNR_WARN(logger_, "[MIRRT* Iter " << iter_ << "] Goal " << igoal << " (REFINING): Direct connect improved cost: " << old_goal_cost << " -> " << costs_.at(igoal) << " (Path: " << path_costs_.at(igoal) << ", Apple: " << goal_costs_.at(igoal) << ", Utopia: " << utopias_.at(igoal) << ")");
                }
                global_improvement = isBestSolution(igoal) || global_improvement;
              }
            }
          }
          break;
        case GoalStatus::discard:
        case GoalStatus::done:
          break;
        }
      }

      if (solved_)
      {
        local_bias_ = std::min(forgetting_factor_ * local_bias_ + reward_ * (old_cost - cost_) / (old_cost - best_utopia_), 1.0);
        for (TubeInformedSamplerPtr &sampler : tube_samplers_)
        {
          sampler->setLocalBias(local_bias_);
        }
      }

      if (global_improvement)
      {
        CNR_WARN(logger_, "[MIRRT* Iter " << iter_ << "] >>> CURRENT SOLUTION IMPROVED: " << old_cost << " -> " << cost_
                                          << " (Best Goal: " << best_goal_index << ", Path Cost: " << path_cost_ << ", Goal Cost: " << goal_cost_ << ")");
      }

      solution = solution_;
      return global_improvement;
    }

    void MultigoalSolver::printMyself(std::ostream &os) const
    {
      os << "Best cost: " << cost_;
      os << " path cost: " << path_cost_;
      os << " goal cost: " << goal_cost_;
      os << ". Nodes of start tree: " << start_tree_->getNumberOfNodes() << "\nGoals:\n";

      for (unsigned int igoal = 0; igoal < goal_nodes_.size(); igoal++)
      {
        os << igoal << ". Status: ";
        switch (status_.at(igoal))
        {
        case GoalStatus::done:
          os << "done";
          break;
        case GoalStatus::refine:
          os << "refine";
          break;
        case GoalStatus::search:
          os << "search";
          break;
        case GoalStatus::discard:
          os << "discard";
          break;
        }
        os << ". cost = " << costs_.at(igoal);
        os << ". path cost = " << path_costs_.at(igoal);
        os << ". goal cost = " << goal_costs_.at(igoal);
        os << ". utopia = " << utopias_.at(igoal);
        os << ". ellisse = " << path_cost_ + goal_cost_ - goal_costs_.at(igoal);

        os << std::endl;
      }
    }

    void MultigoalSolver::cleanTree()
    {
      CNR_WARN(logger_, "[MIRRT* Pruning] Cleaning start tree outside informed ellipsoids (Nodes before purge: " << (start_tree_ ? start_tree_->getNumberOfNodes() : 0) << ")...");
      std::vector<NodePtr> white_list = goal_nodes_;
      white_list.push_back(start_tree_->getRoot());
      std::vector<SamplerPtr> samplers;
      for (unsigned int igoal = 0; igoal < goal_nodes_.size(); igoal++)
      {
        if ((status_.at(igoal) == GoalStatus::refine) || (status_.at(igoal) == GoalStatus::search))
          samplers.push_back(tube_samplers_.at(igoal));
      }

      for (const PathPtr &sol : solutions_)
      {
        if (sol)
          for (const ConnectionPtr &conn : sol->getConnections())
            white_list.push_back(conn->getChild());
      }
      start_tree_->purgeNodesOutsideEllipsoids(samplers, white_list);
      CNR_WARN(logger_, "[MIRRT* Pruning] Cleaning complete. Start tree remaining nodes: " << (start_tree_ ? start_tree_->getNumberOfNodes() : 0));
    }

    std::vector<TreePtr> MultigoalSolver::getGoalTrees()
    {
      return goal_trees_;
    }

  } //  end namespace mirrt_star
} //  end namespace graph
