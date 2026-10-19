# mirrt_star API reference

This page is generated from the Doxygen comments in the source code. To browse the same
documentation as HTML, run `doxygen Doxyfile` in the repository folder and open
`doxygen/html/index.html`.

## Contents

- [`include/mirrt_star/multi_goal_selection/distributions.h`](#includemirrt_starmulti_goal_selectiondistributionsh)
  - [`multi_goal_selection::beta_distribution`](#class-multi_goal_selectionbeta_distribution)
  - [`multi_goal_selection::beta_distribution::param_type`](#class-multi_goal_selectionbeta_distributionparam_type)
- [`include/mirrt_star/multi_goal_selection/goal_selection_manager.h`](#includemirrt_starmulti_goal_selectiongoal_selection_managerh)
  - [`multi_goal_selection::GoalSelectionManager`](#class-multi_goal_selectiongoalselectionmanager)
- [`include/mirrt_star/multi_goal_selection/goal_selection_util.h`](#includemirrt_starmulti_goal_selectiongoal_selection_utilh)
- [`include/mirrt_star/multi_goal_selection/policies/policy_base.h`](#includemirrt_starmulti_goal_selectionpoliciespolicy_baseh)
  - [`multi_goal_selection::PolicyBase`](#class-multi_goal_selectionpolicybase)
- [`include/mirrt_star/multi_goal_selection/policies/policy_custom_example.h`](#includemirrt_starmulti_goal_selectionpoliciespolicy_custom_exampleh)
  - [`multi_goal_selection::PolicyCustomExample`](#class-multi_goal_selectionpolicycustomexample)
- [`include/mirrt_star/multi_goal_selection/policies/policy_mab.h`](#includemirrt_starmulti_goal_selectionpoliciespolicy_mabh)
  - [`multi_goal_selection::PolicyMAB`](#class-multi_goal_selectionpolicymab)
- [`include/mirrt_star/multi_goal_selection/policies/policy_mab_egreedy.h`](#includemirrt_starmulti_goal_selectionpoliciespolicy_mab_egreedyh)
  - [`multi_goal_selection::PolicyMABEGreedy`](#class-multi_goal_selectionpolicymabegreedy)
- [`include/mirrt_star/multi_goal_selection/policies/policy_mab_example.h`](#includemirrt_starmulti_goal_selectionpoliciespolicy_mab_exampleh)
  - [`multi_goal_selection::PolicyMABExample`](#class-multi_goal_selectionpolicymabexample)
- [`include/mirrt_star/multi_goal_selection/policies/policy_mab_kfmanb.h`](#includemirrt_starmulti_goal_selectionpoliciespolicy_mab_kfmanbh)
  - [`multi_goal_selection::PolicyMABKFMANB`](#class-multi_goal_selectionpolicymabkfmanb)
- [`include/mirrt_star/multi_goal_selection/policies/policy_mab_ts.h`](#includemirrt_starmulti_goal_selectionpoliciespolicy_mab_tsh)
  - [`multi_goal_selection::PolicyMABTS`](#class-multi_goal_selectionpolicymabts)
- [`include/mirrt_star/multi_goal_selection/policies/policy_mab_ucb.h`](#includemirrt_starmulti_goal_selectionpoliciespolicy_mab_ucbh)
  - [`multi_goal_selection::PolicyMABUCB`](#class-multi_goal_selectionpolicymabucb)
- [`include/mirrt_star/multi_goal_selection/policies/policy_uniform_on_goals.h`](#includemirrt_starmulti_goal_selectionpoliciespolicy_uniform_on_goalsh)
  - [`multi_goal_selection::PolicyUniformOnGoals`](#class-multi_goal_selectionpolicyuniformongoals)
- [`include/mirrt_star/multi_goal_selection/policies/policy_uniform_on_volume.h`](#includemirrt_starmulti_goal_selectionpoliciespolicy_uniform_on_volumeh)
  - [`multi_goal_selection::PolicyUniformOnVolume`](#class-multi_goal_selectionpolicyuniformonvolume)
- [`include/mirrt_star/multi_goal_selection/rewards/reward_base.h`](#includemirrt_starmulti_goal_selectionrewardsreward_baseh)
  - [`multi_goal_selection::RewardBase`](#class-multi_goal_selectionrewardbase)
- [`include/mirrt_star/multi_goal_selection/rewards/reward_bernoulli.h`](#includemirrt_starmulti_goal_selectionrewardsreward_bernoullih)
  - [`multi_goal_selection::RewardBernoulli`](#class-multi_goal_selectionrewardbernoulli)
- [`include/mirrt_star/multi_goal_selection/rewards/reward_best_cost.h`](#includemirrt_starmulti_goal_selectionrewardsreward_best_costh)
  - [`multi_goal_selection::RewardBestCost`](#class-multi_goal_selectionrewardbestcost)
- [`include/mirrt_star/multi_goal_selection/rewards/reward_relative_improvement.h`](#includemirrt_starmulti_goal_selectionrewardsreward_relative_improvementh)
  - [`multi_goal_selection::RewardRelativeImprovement`](#class-multi_goal_selectionrewardrelativeimprovement)
- [`include/mirrt_star/plugins/solvers/mirrt_star_plugin.h`](#includemirrt_starpluginssolversmirrt_star_pluginh)
  - [`graph::mirrt_star::MIRRTStarPlugin`](#class-graphmirrt_starmirrtstarplugin)
- [`include/mirrt_star/solvers/mirrt_star.h`](#includemirrt_starsolversmirrt_starh)
  - [`graph::mirrt_star::MultigoalSolver`](#class-graphmirrt_starmultigoalsolver)

## include/mirrt_star/multi_goal_selection/distributions.h

Beta distribution compatible with the C++ &lt;random&gt; interface.

### class `multi_goal_selection::beta_distribution`

```cpp
template <typename RealType>
class beta_distribution
```

Beta(a, b) distribution, sampled as X/(X+Y) with X~Gamma(a), Y~Gamma(b).

Follows the interface of the standard distributions in &lt;random&gt;. Used by the Thompson sampling policy (`PolicyMABTS`).

**Template parameters:**

- `RealType`: Floating point type.

#### Public types

##### `result_type`

```cpp
typedef RealType multi_goal_selection::beta_distribution< RealType >::result_type
```

#### Public functions

##### `beta_distribution`

```cpp
beta_distribution(RealType a=2.0, RealType b=2.0)
```

Constructor.

**Parameters:**

- `a`: First shape parameter (&gt; 0).
- `b`: Second shape parameter (&gt; 0).

##### `beta_distribution`

```cpp
beta_distribution(const param_type &param)
```

##### `reset`

```cpp
void reset()
```

##### `param`

```cpp
param_type param() const
```

##### `param`

```cpp
void param(const param_type &param)
```

##### `operator()`

```cpp
template <typename URNG>
result_type operator()(URNG &engine)
```

Draw a sample in [0, 1].

**Parameters:**

- `engine`: Random number engine.

##### `operator()`

```cpp
template <typename URNG>
result_type operator()(URNG &engine, const param_type &param)
```

##### `min`

```cpp
result_type min() const
```

##### `max`

```cpp
result_type max() const
```

##### `a`

```cpp
result_type a() const
```

##### `b`

```cpp
result_type b() const
```

##### `operator==`

```cpp
bool operator==(const beta_distribution< result_type > &other) const
```

##### `operator!=`

```cpp
bool operator!=(const beta_distribution< result_type > &other) const
```

### class `multi_goal_selection::beta_distribution::param_type`

Parameter set (a, b) of the distribution.

#### Public types

##### `distribution_type`

```cpp
typedef beta_distribution multi_goal_selection::beta_distribution< RealType >::param_type::distribution_type
```

#### Public functions

##### `param_type`

```cpp
param_type(RealType a=2.0, RealType b=2.0)
```

##### `a`

```cpp
RealType a() const
```

##### `b`

```cpp
RealType b() const
```

##### `operator==`

```cpp
bool operator==(const param_type &other) const
```

##### `operator!=`

```cpp
bool operator!=(const param_type &other) const
```

### Free functions and types

#### `operator<<`

```cpp
template <typename CharT, typename RealType>
std::basic_ostream< CharT > & operator<<(std::basic_ostream< CharT > &os, const beta_distribution< RealType > &beta)
```

Write the distribution as "~Beta(a,b)".

#### `operator>>`

```cpp
template <typename CharT, typename RealType>
std::basic_istream< CharT > & operator>>(std::basic_istream< CharT > &is, beta_distribution< RealType > &beta)
```

Read a distribution written as "~Beta(a,b)".

## include/mirrt_star/multi_goal_selection/goal_selection_manager.h

Goal selection manager used by MI-RRT* to choose which goals to expand.

### class `multi_goal_selection::GoalSelectionManager`

Chooses which goals MI-RRT* expands at each iteration.

It owns a policy (`multi_goal_selection::PolicyBase`) and a reward function (`multi_goal_selection::RewardBase`). After each iteration the reward is computed from the goal costs and given to the policy for the goals that were expanded; then the policy returns the new expansion probabilities.

Parameters read under the namespace `name`:

- `policy_type` (string, default "MultiArmedBandit"): only "MultiArmedBandit" is supported;
- `policy_name` (string, default "eGreedy"): only "eGreedy" is currently enabled;
- `reward_fcn` (string, default "RelativeImprovement"): only "RelativeImprovement" is currently enabled;
- `warm_start_reward` (bool, default false): initialize the policy with rewards computed from the utopias;
- the parameters of the selected policy (e.g. `PolicyMABEGreedy`).

#### Public functions

##### `GoalSelectionManager`

```cpp
GoalSelectionManager(const std::string &name, const cnr_logger::TraceLoggerPtr &logger, const unsigned int &n_goals, const unsigned int &n_dof)
```

Constructor.

Reads the parameters and creates policy and reward function.

**Parameters:**

- `name`: Parameter namespace.
- `logger`: Logger.
- `n_goals`: Number of goals.
- `n_dof`: Number of degrees of freedom (used by volume-based policies).

##### `calculateProbabilities`

```cpp
std::vector< double > calculateProbabilities(const std::vector< bool > &were_goals_selected, const std::vector< double > &costs, const std::vector< double > &utopias, const double &best_cost)
```

Update the policy with the last reward and compute the new expansion probabilities.

**Parameters:**

- `were_goals_selected`: True for each goal expanded in the last iteration.
- `costs`: Current cost of each goal.
- `utopias`: Utopia of each goal.
- `best_cost`: Current best cost.

**Returns:** Probability of expanding each goal.

##### `warmStart`

```cpp
void warmStart(const std::vector< double > &costs, const std::vector< double > &utopias, const double &best_cost)
```

Initialize the policy with one reward per goal, computed using the utopia as best cost.

**Parameters:**

- `costs`: Current cost of each goal.
- `utopias`: Utopia of each goal.
- `best_cost`: Current best cost (unused).

##### `isWarmStartSet`

```cpp
bool isWarmStartSet()
```

True if the `warm_start_reward` parameter is set.

#### Protected attributes

| Member | Type | Description |
|---|---|---|
| `goal_number_` | `int` | Number of goals. |
| `goal_probabilities_` | `std::vector< double >` | Last computed probabilities. |
| `policy_type_` | `std::string` | Policy type (parameter `policy_type`). |
| `policy_name_` | `std::string` | Policy name (parameter `policy_name`). |
| `reward_fcn_name_` | `std::string` | Reward function name (parameter `reward_fcn`). |
| `do_warm_start_` | `bool` | Warm start flag (parameter `warm_start_reward`). |
| `policy_` | `std::shared_ptr< multi_goal_selection::PolicyBase >` | Goal selection policy. |
| `reward_fcn_` | `std::shared_ptr< multi_goal_selection::RewardBase >` | Reward function. |
| `logger_` | `cnr_logger::TraceLoggerPtr` | Pointer to a TraceLogger instance for logging. |

### Free functions and types

#### `GoalSelectionManagerPtr`

```cpp
typedef std::shared_ptr<GoalSelectionManager> multi_goal_selection::GoalSelectionManagerPtr
```

Shared pointer to a `GoalSelectionManager`.

## include/mirrt_star/multi_goal_selection/goal_selection_util.h

Small vector helpers used by the goal selection policies.

### Free functions and types

#### `vectorMaxIndex`

```cpp
template <class T>
int vectorMaxIndex(const std::vector< T > &elems)
```

Index of the largest element.

**Parameters:**

- `elems`: Non-empty vector.

**Returns:** Index of the first maximum.

#### `vectorMax`

```cpp
template <class T>
T vectorMax(std::vector< T > &elems)
```

Largest element.

**Parameters:**

- `elems`: Non-empty vector.

**Returns:** Maximum value.

#### `vectorSum`

```cpp
template <class T>
T vectorSum(const std::vector< T > &elems)
```

Sum of the elements.

**Parameters:**

- `elems`: Vector.

**Returns:** Sum (0 for an empty vector).

## include/mirrt_star/multi_goal_selection/policies/policy_base.h

Base class of the goal selection policies.

### class `multi_goal_selection::PolicyBase`

Base class of a goal selection policy.

A policy returns, at each iteration, the probability of expanding each goal, and it is updated with the reward obtained after each expansion.

#### Public functions

##### `PolicyBase`

```cpp
PolicyBase(const std::string &name, const int &n_goals, const cnr_logger::TraceLoggerPtr &logger)
```

Constructor.

**Parameters:**

- `name`: Parameter namespace of the policy.
- `n_goals`: Number of goals (arms).
- `logger`: Logger.

##### `getProbabilities`

```cpp
virtual std::vector< double > getProbabilities()=0
```

Probability of expanding each goal at the next iteration.

##### `updateState`

```cpp
virtual void updateState(const int &i_goal, const double &reward)=0
```

Update the policy after goal `i_goal` was expanded.

**Parameters:**

- `i_goal`: Goal index.
- `reward`: Reward obtained.

##### `toString`

```cpp
virtual std::string toString()=0
```

Short description of the policy.

#### Protected attributes

| Member | Type | Description |
|---|---|---|
| `n_goals_` | `int` | Number of goals. |
| `goals_probabilities_` | `std::vector< double >` | Last computed probabilities. |
| `gen_` | `std::mt19937` | Random number generator. |
| `logger_` | `cnr_logger::TraceLoggerPtr` | Logger. |

### Free functions and types

#### `PolicyBasePtr`

```cpp
typedef std::shared_ptr<PolicyBase> multi_goal_selection::PolicyBasePtr
```

Shared pointer to a `PolicyBase`.

## include/mirrt_star/multi_goal_selection/policies/policy_custom_example.h

Example of a custom (non-bandit) goal selection policy.

> **Warning:** Its constructor does not match PolicyBase (missing logger): it does not compile and is not used.

### class `multi_goal_selection::PolicyCustomExample`

Inherits from `multi_goal_selection::PolicyBase`.

Example policy that always expands the last goal.

#### Public functions

##### `PolicyCustomExample`

```cpp
PolicyCustomExample(const std::string &name, const int &n_goals)
```

##### `getProbabilities`

```cpp
virtual std::vector< double > getProbabilities()
```

Probability of expanding each goal at the next iteration.

##### `updateState`

```cpp
virtual void updateState(const int &i_goal, const double &reward)
```

Update the policy after goal `i_goal` was expanded.

**Parameters:**

- `i_goal`: Goal index.
- `reward`: Reward obtained.

##### `toString`

```cpp
virtual std::string toString()
```

Short description of the policy.

### Free functions and types

#### `PolicyCustomExamplePtr`

```cpp
typedef std::shared_ptr<PolicyCustomExample> multi_goal_selection::PolicyCustomExamplePtr
```

Shared pointer to a `PolicyCustomExample`.

## include/mirrt_star/multi_goal_selection/policies/policy_mab.h

Base class of the multi-armed bandit (MAB) goal selection policies.

### class `multi_goal_selection::PolicyMAB`

Inherits from `multi_goal_selection::PolicyBase`.

Base class of the multi-armed bandit policies.

Each goal is an arm. At every iteration exactly one arm, chosen by `selectNextArm()`, gets probability 1 and the others get 0.

#### Public functions

##### `PolicyMAB`

```cpp
PolicyMAB(const std::string &name, const int &n_goals, const cnr_logger::TraceLoggerPtr &logger)
```

Constructor.

**Parameters:**

- `name`: Parameter namespace of the policy.
- `n_goals`: Number of goals (arms).
- `logger`: Logger.

##### `getProbabilities`

```cpp
virtual std::vector< double > getProbabilities()
```

Select one arm and return a one-hot probability vector.

##### `selectNextArm`

```cpp
virtual int selectNextArm()=0
```

Choose the arm to pull at the next iteration.

##### `reinitRewards`

```cpp
virtual bool reinitRewards(std::vector< double > rewards, std::vector< double > std_devs)
```

Reset the expected rewards to given values.

**Parameters:**

- `rewards`: Initial reward of each arm (size n_goals).
- `std_devs`: Standard deviation of each arm (unused here).

**Returns:** False if the vector size is wrong.

##### `updateState`

```cpp
virtual void updateState(const int &i_goal, const double &reward, const double &variance)
```

Update with a reward and its variance.

The base version ignores the variance.

**Parameters:**

- `i_goal`: Arm index.
- `reward`: Reward.
- `variance`: Reward variance.

##### `updateState`

```cpp
virtual void updateState(const int &i_goal, const double &reward)
```

Default update: increment the pull counter and accumulate the reward.

##### `toString`

```cpp
virtual std::string toString()=0
```

Short description of the policy.

##### `print`

```cpp
virtual void print()
```

Print rewards and pull counters on stdout.

##### `getExpectedRewards`

```cpp
std::vector< double > getExpectedRewards()
```

Expected reward of each arm.

##### `getVariance`

```cpp
virtual std::vector< double > getVariance()
```

Reward variance of each arm (not implemented in the base class).

##### `getPullCounters`

```cpp
std::vector< int > getPullCounters()
```

Number of pulls of each arm.

#### Protected attributes

| Member | Type | Description |
|---|---|---|
| `pull_counter_` | `std::vector< int >` | Number of pulls of each arm. |
| `expected_reward_` | `std::vector< double >` | Expected (or accumulated) reward of each arm. |
| `initial_reward_` | `std::vector< double >` | Rewards set by `reinitRewards()`. |

### Free functions and types

#### `PolicyMABPtr`

```cpp
typedef std::shared_ptr<PolicyMAB> multi_goal_selection::PolicyMABPtr
```

Shared pointer to a `PolicyMAB`.

## include/mirrt_star/multi_goal_selection/policies/policy_mab_egreedy.h

Epsilon-greedy multi-armed bandit policy.

### class `multi_goal_selection::PolicyMABEGreedy`

Inherits from `multi_goal_selection::PolicyMAB`.

Epsilon-greedy multi-armed bandit policy.

With probability epsilon a random arm is pulled, otherwise the arm with the highest expected reward (arms never pulled are tried first).

Parameters read under the policy namespace:

- `epsilon_coef` (double, default 0.2): exploration probability epsilon;
- `epsilon_exp` (double, default 1.0): if &lt; 1, epsilon is multiplied by this value at every update;
- `reward_gain` (double, default 1.0): gain used by the adaptive epsilon;
- `egreedy_forgetting_factor` (double, default 0.0): adaptive epsilon (currently forced to 0);
- `egreedy_reward_forgetting_factor` (double, default 0.0): if &gt; 0, expected rewards are an exponential moving average; otherwise they are the sample mean.

#### Public functions

##### `PolicyMABEGreedy`

```cpp
PolicyMABEGreedy(const std::string &name, const int &n_goals, const cnr_logger::TraceLoggerPtr &logger)
```

Constructor.

Reads the parameters listed in the class description.

**Parameters:**

- `name`: Parameter namespace.
- `n_goals`: Number of arms.
- `logger`: Logger.

##### `selectNextArm`

```cpp
virtual int selectNextArm()
```

Epsilon-greedy arm selection.

##### `updateState`

```cpp
virtual void updateState(const int &i_goal, const double &reward)
```

Update the expected reward of the pulled arm and decay epsilon.

**Parameters:**

- `i_goal`: Pulled arm.
- `reward`: Reward obtained.

##### `toString`

```cpp
virtual std::string toString()
```

Policy name and current epsilon.

#### Protected attributes

| Member | Type | Description |
|---|---|---|
| `epsilon_exp_` | `double` | Decay factor of epsilon. |
| `egreedy_forgetting_factor_` | `double` | Forgetting factor of the adaptive epsilon. |
| `egreedy_reward_forgetting_factor_` | `double` | Forgetting factor of the expected rewards. |
| `reward_gain_` | `double` | Reward gain of the adaptive epsilon. |
| `pulled_arms_` | `std::vector< int >` | Unused. |

### Free functions and types

#### `PolicyMABEGreedyPtr`

```cpp
typedef std::shared_ptr<PolicyMABEGreedy> multi_goal_selection::PolicyMABEGreedyPtr
```

Shared pointer to a `PolicyMABEGreedy`.

## include/mirrt_star/multi_goal_selection/policies/policy_mab_example.h

Example of a custom multi-armed bandit policy.

> **Warning:** Not yet ported from ROS1 (uses ROS_ERROR): it does not compile and is not used.

### class `multi_goal_selection::PolicyMABExample`

Inherits from `multi_goal_selection::PolicyMAB`.

Example MAB policy: selects an arm with probability proportional to its squared expected reward.

#### Public functions

##### `PolicyMABExample`

```cpp
PolicyMABExample(const std::string &name, const int &n_goals)
```

##### `selectNextArm`

```cpp
virtual int selectNextArm()
```

Choose the arm to pull at the next iteration.

##### `updateState`

```cpp
virtual void updateState(const int &i_goal, const double &reward)
```

Default update: increment the pull counter and accumulate the reward.

##### `toString`

```cpp
virtual std::string toString()
```

Short description of the policy.

### Free functions and types

#### `PolicyMABExamplePtr`

```cpp
typedef std::shared_ptr<PolicyMABExample> multi_goal_selection::PolicyMABExamplePtr
```

Shared pointer to a `PolicyMABExample`.

## include/mirrt_star/multi_goal_selection/policies/policy_mab_kfmanb.h

Kalman filter multi-armed bandit (KF-MANB) policy.

> **Warning:** Not yet ported from ROS1 (uses ros::NodeHandle and ROS1 messages): it does not compile and is not used by GoalSelectionManager.

### class `multi_goal_selection::PolicyMABKFMANB`

Inherits from `multi_goal_selection::PolicyMAB`.

KF-MANB policy (McConachie and Berenson, T-ASE 2018).

Each arm reward is tracked by a scalar Kalman filter; the next arm is the one with the highest reward sampled from the estimated Gaussian distributions.

Parameters: `sigma_obs` (0.01), `sigma_tr` (0.1), `sigma_arms` (0.2), `sigma_obs_dyn` (false), `sigma_gain` (1.0).

#### Public functions

##### `PolicyMABKFMANB`

```cpp
PolicyMABKFMANB(const std::string &name, const int &n_goals)
```

##### `getVariance`

```cpp
virtual std::vector< double > getVariance()
```

Reward variance of each arm (not implemented in the base class).

##### `selectNextArm`

```cpp
virtual int selectNextArm()
```

Choose the arm to pull at the next iteration.

##### `reinitRewards`

```cpp
virtual bool reinitRewards(std::vector< double > rewards, std::vector< double > std_devs)
```

Reset the expected rewards to given values.

**Parameters:**

- `rewards`: Initial reward of each arm (size n_goals).
- `std_devs`: Standard deviation of each arm (unused here).

**Returns:** False if the vector size is wrong.

##### `updateState`

```cpp
virtual void updateState(const int &i_goal, const double &reward, const double &variance)
```

Update with a reward and its variance.

The base version ignores the variance.

**Parameters:**

- `i_goal`: Arm index.
- `reward`: Reward.
- `variance`: Reward variance.

##### `updateState`

```cpp
virtual void updateState(const int &i_goal, const double &reward)
```

Default update: increment the pull counter and accumulate the reward.

##### `toString`

```cpp
virtual std::string toString()
```

Short description of the policy.

### Free functions and types

#### `PolicyMABKFMANBPtr`

```cpp
typedef std::shared_ptr<PolicyMABKFMANB> multi_goal_selection::PolicyMABKFMANBPtr
```

Shared pointer to a `PolicyMABKFMANB`.

## include/mirrt_star/multi_goal_selection/policies/policy_mab_ts.h

Thompson sampling multi-armed bandit policy.

> **Warning:** Not yet ported from ROS1 (uses ros::NodeHandle): it does not compile and is not used by GoalSelectionManager.

### class `multi_goal_selection::PolicyMABTS`

Inherits from `multi_goal_selection::PolicyMAB`.

Thompson sampling with Beta priors and Bernoulli rewards (reward &gt; 0.5 counts as a success).

Parameters: `ts_alpha` and `ts_beta` (double, default 1.0), the prior of each arm.

#### Public functions

##### `PolicyMABTS`

```cpp
PolicyMABTS(const std::string &name, const int &n_goals)
```

##### `selectNextArm`

```cpp
virtual int selectNextArm()
```

Choose the arm to pull at the next iteration.

##### `updateState`

```cpp
virtual void updateState(const int &i_goal, const double &reward)
```

Default update: increment the pull counter and accumulate the reward.

##### `toString`

```cpp
virtual std::string toString()
```

Short description of the policy.

### Free functions and types

#### `PolicyMABTSPtr`

```cpp
typedef std::shared_ptr<PolicyMABTS> multi_goal_selection::PolicyMABTSPtr
```

Shared pointer to a `PolicyMABTS`.

## include/mirrt_star/multi_goal_selection/policies/policy_mab_ucb.h

UCB1 multi-armed bandit policy.

> **Warning:** Not yet ported from ROS1 (uses ros::NodeHandle): it does not compile and is not used by GoalSelectionManager.

### class `multi_goal_selection::PolicyMABUCB`

Inherits from `multi_goal_selection::PolicyMAB`.

UCB1 policy: pulls the arm maximizing mean reward + sqrt(alpha*ln(n)/n_i).

Parameter: `ucb_alpha` (double, default 1.0).

#### Public functions

##### `PolicyMABUCB`

```cpp
PolicyMABUCB(const std::string &name, const int &n_goals)
```

##### `selectNextArm`

```cpp
virtual int selectNextArm()
```

Choose the arm to pull at the next iteration.

##### `updateState`

```cpp
virtual void updateState(const int &i_goal, const double &reward)
```

Default update: increment the pull counter and accumulate the reward.

##### `toString`

```cpp
virtual std::string toString()
```

Short description of the policy.

### Free functions and types

#### `PolicyMABUCBPtr`

```cpp
typedef std::shared_ptr<PolicyMABUCB> multi_goal_selection::PolicyMABUCBPtr
```

Shared pointer to a `PolicyMABUCB`.

## include/mirrt_star/multi_goal_selection/policies/policy_uniform_on_goals.h

Uniform goal selection.

> **Warning:** Its constructor does not match PolicyMAB (missing logger): it does not compile and is not used.

### class `multi_goal_selection::PolicyUniformOnGoals`

Inherits from `multi_goal_selection::PolicyMAB`.

Selects one goal uniformly at random at every iteration.

#### Public functions

##### `PolicyUniformOnGoals`

```cpp
PolicyUniformOnGoals(const std::string &name, const int &n_goals)
```

##### `selectNextArm`

```cpp
virtual int selectNextArm()
```

Choose the arm to pull at the next iteration.

##### `updateState`

```cpp
virtual void updateState(const int &i_goal, const double &reward)
```

Default update: increment the pull counter and accumulate the reward.

##### `toString`

```cpp
virtual std::string toString()
```

Short description of the policy.

### Free functions and types

#### `PolicyUniformOnGoalsPtr`

```cpp
typedef std::shared_ptr<PolicyUniformOnGoals> multi_goal_selection::PolicyUniformOnGoalsPtr
```

Shared pointer to a `PolicyUniformOnGoals`.

## include/mirrt_star/multi_goal_selection/policies/policy_uniform_on_volume.h

Goal selection proportional to the volume of each informed set.

> **Warning:** Not yet ported from ROS1 (uses ROS_* macros): it does not compile and is not used.

### class `multi_goal_selection::PolicyUniformOnVolume`

Inherits from `multi_goal_selection::PolicyMAB`.

Selects a goal with probability proportional to the volume of its informed set.

Requires the BestCost reward and warm start (the warm-start rewards are the utopias). Before a first solution is found, goals are selected uniformly.

#### Public functions

##### `PolicyUniformOnVolume`

```cpp
PolicyUniformOnVolume(const std::string &name, const int &n_goals, const int &n_dof)
```

##### `selectNextArm`

```cpp
virtual int selectNextArm()
```

Choose the arm to pull at the next iteration.

##### `updateState`

```cpp
virtual void updateState(const int &i_goal, const double &reward)
```

Default update: increment the pull counter and accumulate the reward.

##### `toString`

```cpp
virtual std::string toString()
```

Short description of the policy.

### Free functions and types

#### `PolicyUniformOnVolumePtr`

```cpp
typedef std::shared_ptr<PolicyUniformOnVolume> multi_goal_selection::PolicyUniformOnVolumePtr
```

Shared pointer to a `PolicyUniformOnVolume`.

## include/mirrt_star/multi_goal_selection/rewards/reward_base.h

Base class of the rewards used to update the goal selection policies.

### class `multi_goal_selection::RewardBase`

Base class of a reward function.

The reward measures how useful the last iteration was, and it is given to the policy for every goal that was expanded.

#### Public functions

##### `RewardBase`

```cpp
RewardBase(const cnr_logger::TraceLoggerPtr &logger)
```

Constructor.

**Parameters:**

- `logger`: Logger.

##### `getReward`

```cpp
virtual double getReward(const std::vector< double > &costs, const std::vector< double > &utopias, const double &best_cost)=0
```

Compute the reward of the last iteration.

**Parameters:**

- `costs`: Current cost of each goal.
- `utopias`: Utopia (lower bound) of each goal.
- `best_cost`: Current best cost.

**Returns:** Reward.

#### Protected attributes

| Member | Type | Description |
|---|---|---|
| `logger_` | `cnr_logger::TraceLoggerPtr` | Logger. |
| `last_best_cost_` | `double` | Best cost at the previous call. |

### Free functions and types

#### `RewardBasePtr`

```cpp
typedef std::shared_ptr<RewardBase> multi_goal_selection::RewardBasePtr
```

Shared pointer to a `RewardBase`.

## include/mirrt_star/multi_goal_selection/rewards/reward_bernoulli.h

Bernoulli reward.

### class `multi_goal_selection::RewardBernoulli`

Inherits from `multi_goal_selection::RewardBase`.

Reward is 1 if the best cost improved (by more than 1e-4) since the last call, 0 otherwise.

#### Public functions

##### `RewardBernoulli`

```cpp
RewardBernoulli(const cnr_logger::TraceLoggerPtr &logger)
```

Constructor.

**Parameters:**

- `logger`: Logger.

##### `getReward`

```cpp
virtual double getReward(const std::vector< double > &costs, const std::vector< double > &utopias, const double &best_cost)
```

Compute the reward of the last iteration.

**Parameters:**

- `costs`: Current cost of each goal.
- `utopias`: Utopia (lower bound) of each goal.
- `best_cost`: Current best cost.

**Returns:** Reward.

### Free functions and types

#### `RewardBernoulliPtr`

```cpp
typedef std::shared_ptr<RewardBernoulli> multi_goal_selection::RewardBernoulliPtr
```

Shared pointer to a `RewardBernoulli`.

## include/mirrt_star/multi_goal_selection/rewards/reward_best_cost.h

Best-cost reward.

### class `multi_goal_selection::RewardBestCost`

Inherits from `multi_goal_selection::RewardBase`.

Reward is minus the current best cost.

Used by `PolicyUniformOnVolume`.

#### Public functions

##### `RewardBestCost`

```cpp
RewardBestCost(const cnr_logger::TraceLoggerPtr &logger)
```

Constructor.

**Parameters:**

- `logger`: Logger.

##### `getReward`

```cpp
virtual double getReward(const std::vector< double > &costs, const std::vector< double > &utopias, const double &best_cost)
```

Compute the reward of the last iteration.

**Parameters:**

- `costs`: Current cost of each goal.
- `utopias`: Utopia (lower bound) of each goal.
- `best_cost`: Current best cost.

**Returns:** Reward.

### Free functions and types

#### `RewardBestCostPtr`

```cpp
typedef std::shared_ptr<RewardBestCost> multi_goal_selection::RewardBestCostPtr
```

Shared pointer to a `RewardBestCost`.

## include/mirrt_star/multi_goal_selection/rewards/reward_relative_improvement.h

Relative improvement reward (default reward).

### class `multi_goal_selection::RewardRelativeImprovement`

Inherits from `multi_goal_selection::RewardBase`.

Reward is the relative improvement of the best cost since the last call, in percent: 100*(last_best - best)/last_best.

It is 0 until a first solution exists.

#### Public functions

##### `RewardRelativeImprovement`

```cpp
RewardRelativeImprovement(const cnr_logger::TraceLoggerPtr &logger)
```

Constructor.

**Parameters:**

- `logger`: Logger.

##### `getReward`

```cpp
virtual double getReward(const std::vector< double > &costs, const std::vector< double > &utopias, const double &best_cost)
```

Compute the reward of the last iteration.

**Parameters:**

- `costs`: Current cost of each goal.
- `utopias`: Utopia (lower bound) of each goal.
- `best_cost`: Current best cost.

**Returns:** Reward.

### Free functions and types

#### `RewardRelativeImprovementPtr`

```cpp
typedef std::shared_ptr<RewardRelativeImprovement> multi_goal_selection::RewardRelativeImprovementPtr
```

Shared pointer to a `RewardRelativeImprovement`.

## include/mirrt_star/plugins/solvers/mirrt_star_plugin.h

cnr_class_loader plugin that creates a `graph::mirrt_star::MultigoalSolver`.

### class `graph::mirrt_star::MIRRTStarPlugin`

Inherits from `TreeSolverPlugin`.

This class implements a wrapper to `graph::mirrt_star::MultigoalSolver` to allow its plugin to be defined.

The class can be loaded as a plugin and builds a `graph::mirrt_star::MultigoalSolver` object.

#### Public functions

##### `MIRRTStarPlugin`

```cpp
EIGEN_MAKE_ALIGNED_OPERATOR_NEW MIRRTStarPlugin()
```

Empty constructor for `MIRRTStarPlugin`.

The function `MIRRTStarPlugin::init()` must be called afterwards.

##### `init`

```cpp
virtual bool init(const std::string &param_ns, const graph::core::MetricsPtr &metrics, const graph::core::CollisionCheckerPtr &checker, const graph::core::SamplerPtr &sampler, const graph::core::GoalCostFunctionPtr &goal_cost_fcn, const cnr_logger::TraceLoggerPtr &logger)
```

init Initialise the object, defining its main attributes.

**Parameters:**

- `param_ns`: defines the namespace under which parameter are searched for using cnr_param library.
- `metrics`: The metrics used to evaluate paths.
- `checker`: The collision checker for checking collisions.
- `sampler`: The sampler for generating random configurations.
- `goal_cost_fcn`: The function used to assign the cost of the goal. If it is not defined, the default cost function does not assign any cost to the goal.
- `logger`: The logger for logging messages.

**Returns:** True if correctly initialised, False if already initialised.

## include/mirrt_star/solvers/mirrt_star.h

Multi-goal informed RRT* solver (MI-RRT*).

### class `graph::mirrt_star::MultigoalSolver`

Inherits from `TreeSolver`.

Multi-goal informed RRT* (MI-RRT*) path planner.

The solver plans from one start configuration towards a set of candidate goal configurations at the same time and returns the path to the goal with the lowest total cost (path cost + goal cost). It keeps a single start tree and, when bidirectional search is enabled, one backward tree per goal.

Every goal has a status:

- `search`: no path to the goal is known yet;
- `refine`: a path exists and it is being optimized (RRT* rewiring);
- `done`: the cost is within `utopia_tolerance` of the goal utopia (lower bound), so it cannot improve much more;
- `discard`: the goal utopia is worse than the best cost found so far, so the goal is ignored.

At each iteration a goal selection policy (see `multi_goal_selection::GoalSelectionManager`) decides which goals are expanded. Samples are drawn from an informed set around each goal (graph::core::InformedSampler), optionally mixed with a tube around the current solution (graph::core::TubeInformedSampler).

Typical use:

```cpp
auto solver = std::make_shared<graph::mirrt_star::MultigoalSolver>(metrics, checker, sampler, goal_cost_fcn, logger);
solver->config("/my_planner/mirrt_star");
solver->addStart(start_node);
for (const auto& goal : goals)
  solver->addGoal(goal, direct_connection_time, goal_cost);
solver->finalizeProblem();
graph::core::PathPtr solution;
while (time_left)
  solver->update(solution);  // true when the best solution improved
```

Parameters read by `config()` under `param_ns` (besides those of graph::core::TreeSolver):

- `rewire_radius` (double, default 2*max_distance): RRT* rewiring radius;
- `mixed_strategy` (bool, default true): sample also inside a tube around the current solution;
- `bidirectional` (bool, default true): grow one tree from each goal as well;
- `k_nearest` (bool, default false): use k-nearest rewiring instead of a fixed radius;
- `local_bias` (double in [0,1], default 0.3): initial probability of sampling inside the tube;
- `tube_radius` (double, default 0.3): tube radius, relative to the solution cost;
- `forgetting_factor` (double, default 0.01): forgetting factor of the adaptive local bias;
- `apple_weight` (double, default 10.0): weight of the goal cost (currently unused by the solver);
- goal selection parameters, see `multi_goal_selection::GoalSelectionManager`.

#### Public functions

##### `MultigoalSolver`

```cpp
MultigoalSolver(const MetricsPtr &metrics, const CollisionCheckerPtr &checker, const SamplerPtr &sampler, const GoalCostFunctionPtr &goal_cost_fcn, const cnr_logger::TraceLoggerPtr &logger)
```

Constructor.

`config()` must be called before adding start and goals.

**Parameters:**

- `metrics`: Metrics used to compute path costs.
- `checker`: Collision checker.
- `sampler`: Sampler; only its bounds are used to build the per-goal informed samplers.
- `goal_cost_fcn`: Function that assigns a cost to each goal.
- `logger`: Logger.

##### `config`

```cpp
virtual bool config(const std::string &param_ns) override
```

Read the parameters (see class description).

**Parameters:**

- `param_ns`: Parameter namespace.

**Returns:** False if the base TreeSolver configuration fails.

##### `initGoalSelector`

```cpp
virtual bool initGoalSelector()
```

Create the goal selection manager for the current set of goals.

Called by `finalizeProblem()`.

**Returns:** Always true.

##### `update`

```cpp
virtual bool update(PathPtr &solution) override
```

Run one iteration of the planner.

Selects the goals to expand, samples a configuration for each of them, grows the trees, and updates goal status and costs.

**Parameters:**

- `solution` [out]: Best solution found so far (nullptr if none).

**Returns:** True if the best solution improved during this iteration, false otherwise (also when the solver is not initialized or the best cost reached the utopia tolerance).

##### `addStart`

```cpp
virtual bool addStart(const NodePtr &start_node, const double &max_time=std::numeric_limits< double >::infinity()) override
```

Set the start node and create the start tree.

Adds a start node to the multigoal solver.

**Parameters:**

- `start_node`: Start node.
- `max_time`: Unused.

**Parameters:**

- `start_node`: The start node to be added.
- `max_time`: The maximum time allowed for solving.

**Returns:** False if the solver is not configured.

**Returns:** True if the start node was successfully added, false otherwise.

##### `addGoal`

```cpp
virtual bool addGoal(const NodePtr &goal_node, const double &max_time=std::numeric_limits< double >::infinity(), const double &apple_cost=0.0)
```

Register a candidate goal.

The goal is skipped if it is in collision or if its utopia is worse than the current best cost. The solver first tries a direct connection from the start tree; if it succeeds the goal starts in `refine` (or `done`) status, otherwise a backward tree is created and the goal starts in `search` status.

**Parameters:**

- `goal_node`: Goal node.
- `max_time`: Maximum time for the direct connection attempt.
- `apple_cost`: Extra cost added to the goal cost (e.g. a task-dependent cost).

**Returns:** True if the goal was registered.

##### `addGoal`

```cpp
virtual bool addGoal(const NodePtr &goal_node, const double &max_time=std::numeric_limits< double >::infinity()) override
```

Register a candidate goal with no extra cost.

> **See also:** `addGoal(const NodePtr&, const double&, const double&)`

##### `addStartTree`

```cpp
virtual bool addStartTree(const TreePtr &start_tree, const double &max_time=std::numeric_limits< double >::infinity()) override
```

Use an existing tree as start tree.

**Parameters:**

- `start_tree`: Tree whose root is the start configuration.
- `max_time`: Unused.

**Returns:** Always true.

##### `resetProblem`

```cpp
virtual void resetProblem() override
```

Remove start, goals and solutions.

Parameters are kept.

##### `finalizeProblem`

```cpp
virtual bool finalizeProblem()
```

Finish the problem setup.

Call it after the last `addGoal()` and before `update()`.

Creates the goal selection manager and, if requested, warm-starts it.

**Returns:** True on success.

##### `getGoalTrees`

```cpp
std::vector< TreePtr > getGoalTrees()
```

Get the backward trees (one per goal, nullptr if not created).

##### `getGoals`

```cpp
std::vector< NodePtr > getGoals()
```

Get the registered goal nodes.

##### `cleanTree`

```cpp
void cleanTree()
```

Prune the start tree, removing nodes outside the informed sets of the active goals.

#### Protected types

##### `GoalStatus`

```cpp
enum GoalStatus { search, refine, done, discard }
```

Search status of a single goal.

- `search`: no path to this goal has been found yet
- `refine`: a path exists and is being optimized
- `done`: cost is within utopia tolerance, the goal is not expanded anymore
- `discard`: utopia is worse than the best cost, the goal is ignored

#### Protected functions

##### `setProblem`

```cpp
virtual bool setProblem(const double &max_time=std::numeric_limits< double >::infinity()) override
```

Check that the problem has a start tree and at least one goal.

**Parameters:**

- `max_time`: Unused.

**Returns:** True if start and goals are defined.

##### `isBestSolution`

```cpp
bool isBestSolution(const int &index)
```

Check if the solution of a goal is the new global best and, if so, update the solver state.

When a new best is found, goals whose utopia exceeds the new cost are discarded, the informed samplers are shrunk, and the start tree is pruned if needed.

**Parameters:**

- `index`: Goal index.

**Returns:** True if the goal solution is the new best one.

##### `printMyself`

```cpp
virtual void printMyself(std::ostream &os) const override
```

Print best cost, tree size and status of each goal.

#### Protected attributes

| Member | Type | Description |
|---|---|---|
| `goal_nodes_` | `std::vector< NodePtr >` | Candidate goal nodes. |
| `costs_` | `std::vector< double >` | Total cost (path + goal cost) for each goal; infinity if not solved. |
| `utopias_` | `std::vector< double >` | Lower bound of the total cost for each goal. |
| `goal_probabilities_` | `std::vector< double >` | Probability of expanding each goal at the next iteration. |
| `were_goals_sampled_` | `std::vector< bool >` | True for goals expanded during the last iteration. |
| `solutions_` | `std::vector< PathPtr >` | Best path found for each goal (nullptr if none). |
| `goal_costs_` | `std::vector< double >` | Cost assigned to each goal by the goal cost function (plus the extra cost given to `addGoal()`). |
| `path_costs_` | `std::vector< double >` | Path cost for each goal. |
| `goal_trees_` | `std::vector< TreePtr >` | Backward tree rooted at each goal (used when bidirectional). |
| `tube_samplers_` | `std::vector< TubeInformedSamplerPtr >` | Tube samplers around the current solution of each goal. |
| `samplers_` | `std::vector< SamplerPtr >` | Informed samplers for each goal. |
| `status_` | `std::vector< GoalStatus >` | Search status of each goal. |
| `cost_at_last_clean` | `double` | Best cost when the start tree was last pruned. |
| `iter_` | `unsigned int` | Number of calls to `update()`. |
| `best_goal_index` | `int` | Index of the goal with the best solution (-1 if none). |
| `r_rewire_` | `double` | RRT* rewiring radius. |
| `local_bias_` | `double` | Probability of sampling inside the tube (adapted online). |
| `reward_` | `double` | Gain of the local bias adaptation. |
| `forgetting_factor_` | `double` | Forgetting factor of the local bias adaptation. |
| `tube_radius_` | `double` | Tube radius, relative to the solution cost. |
| `goal_bias_` | `double` | Probability of trying a direct connection to the goal. |
| `bidirectional_` | `bool` | Grow a backward tree from each goal. |
| `mixed_strategy_` | `bool` | Sample from both the informed set and the tube. |
| `knearest_` | `bool` | Use k-nearest rewiring. |
| `apple_weight_` | `double` | Weight of the goal cost (read from parameters, currently unused). |
| `goal_manager_` | `multi_goal_selection::GoalSelectionManagerPtr` | Policy that decides which goals to expand. Created by `finalizeProblem()`. |

### Free functions and types

#### `MultigoalSolverPtr`

```cpp
typedef std::shared_ptr<MultigoalSolver> graph::mirrt_star::MultigoalSolverPtr
```

Shared pointer to a `MultigoalSolver`.
