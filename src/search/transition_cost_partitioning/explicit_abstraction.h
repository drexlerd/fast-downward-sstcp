#ifndef TRANSITION_COST_PARTITIONING_EXPLICIT_ABSTRACTION_H
#define TRANSITION_COST_PARTITIONING_EXPLICIT_ABSTRACTION_H

#include "abstraction.h"
#include "types.h"

#include "abstraction_function.h"

#include "../algorithms/priority_queues.h"
#include "../cegar/cartesian_set.h"
#include "../cegar/split_tree.h"

#include <memory>
#include <vector>
#include <functional>

using namespace std;

namespace transition_cost_partitioning {

/*
  An ExplicitAbstraction represents a similar structure for all type
  of cartesian abstractions (CEGAR, projections).
  It reverts the domain abstraction performed in CEGAR landmark tasks.
*/
class ExplicitAbstraction : public Abstraction {
  protected:
    /**
     * Store state-changing transitions explicitely.
     */
    const vector<vector<Successor>> backward_graph;
    mutable vector<vector<Successor>> forward_graph;

    /** 
     * Priority queue for distance analysis.
     */
    mutable priority_queues::AdaptiveQueue<int> queue;

    /**
     * num_transitions_by_operator[op] is the number of state-changing transitions that operator op induces.
     */
    const vector<int> num_transitions_by_operator;

    /**
     * has_loop[op]=true iff operator op induces an abstract self-loop.
     */
    const vector<bool> has_loop;
    /**
     * has_outgoing[op]=true iff operator op induces an abstract state-chancging transition. 
     */
    const vector<bool> has_outgoing;

  protected:
    virtual vector<int> compute_goal_distances_for_non_negative_costs_ocf(const vector<int> &ocf) const override;
    virtual vector<int> compute_goal_distances_for_non_negative_costs_tcf(
      const CostFunctionStateDependent &sdac,
      AbstractTransitionCostFunction &tcf) const override;
    virtual vector<int> compute_goal_distances_for_non_negative_costs_tcf(
      AbstractTransitionCostFunction &tcf) const override;

  public:
    /**
     * R6: Moveable and not copyable.
     */
    ExplicitAbstraction() = delete;
    ExplicitAbstraction(
      const TaskInfo &task_info,
      const BddBuilder &bdd_builder,
      unique_ptr<AbstractionFunction> abstraction_function,
      int num_transitions,
      int num_states,
      int init_state_id,
      unordered_set<int> &&goal_states,
      vector<vector<Successor>> &&backward_graph,
      vector<int> &&num_transitions_by_operator,
      vector<bool> &&has_outgoing,
      vector<bool> &&has_loop);
    ExplicitAbstraction(const ExplicitAbstraction& other) = delete;
    ExplicitAbstraction& operator=(const ExplicitAbstraction &other) = delete;
    ExplicitAbstraction(ExplicitAbstraction &&other) = default;
    ExplicitAbstraction& operator=(ExplicitAbstraction &&other) = default;
    virtual ~ExplicitAbstraction();

    virtual void for_each_transition(const TransitionCallback &callback) const override;

    virtual vector<bool> compute_reachability_from_state_ocf(const vector<int> &ocf, int state_id) const override;
    virtual vector<bool> compute_reachability_from_state_tcf(AbstractTransitionCostFunction &tcf, int state_id) const override;

    virtual vector<bool> compute_reachability_to_state_ocf(const vector<int> &ocf, int state_id) const override;
    virtual vector<bool> compute_reachability_to_state_tcf(AbstractTransitionCostFunction &tcf, int state_id) const override;

    virtual vector<int> compute_saturated_costs_ocf(const vector<int> &h_values) const override;
    virtual void compute_saturated_costs_tcf(const vector<int> &h_values, AbstractTransitionCostFunction &stcf) const override;

    virtual int get_num_transitions(int op_id) const override;
    virtual bool operator_induces_self_loop(int op_id) const override;
    virtual bool operator_is_active(int op_id) const override;
};

}

#endif
