#ifndef TRANSITION_COST_PARTITIONING_PROJECTION_H
#define TRANSITION_COST_PARTITIONING_PROJECTION_H

#include "abstraction.h"

#include "types.h"

using namespace std;

class OperatorProxy;
class TaskProxy;
class VariablesProxy;

namespace pdbs {
class MatchTree;
}

namespace transition_cost_partitioning {
using OperatorCallback = function<void (Facts &, Facts &, Facts &, int, const vector<size_t> &, int)>;

struct AbstractForwardOperator {
    int precondition_hash;
    int hash_effect;

    AbstractForwardOperator(
        int precondition_hash,
        int hash_effect)
        : precondition_hash(precondition_hash),
          hash_effect(hash_effect) {
    }
};

struct AbstractBackwardOperator {
    int concrete_operator_id;
    int hash_effect;

    AbstractBackwardOperator(
        int concrete_operator_id,
        int hash_effect)
        : concrete_operator_id(concrete_operator_id),
          hash_effect(hash_effect) {
    }
};

class Projection : public Abstraction {
  private:
    const pdbs::Pattern pattern;

    // Multipliers for each variable for perfect hash function.
    const vector<size_t> hash_multipliers;

    // Domain size of each variable in the pattern.
    const vector<int> pattern_domain_sizes;

    const vector<AbstractForwardOperator> abstract_forward_operators;
    const unique_ptr<pdbs::MatchTree> match_tree_forward;

    const vector<AbstractBackwardOperator> abstract_backward_operators;
    const unique_ptr<pdbs::MatchTree> match_tree_backward;

    // To compute transition id:
    const vector<int> transition_id_offset;

    /**
     * num_transitions_by_operator[op] is the number of state-changing transitions that operator op induces.
     */
    const vector<int> num_transitions_by_operator;
    /**
     * abstract_operator_id_offset[op] is the index of the first abstract operator corresponding to op
     */
    const vector<int> abstract_operator_id_offset;

    // Reuse vector to save allocations.
    mutable vector<FactPair> abstract_facts;
    mutable vector<FactPair> state_facts;

  private:
    /*
      Given an abstract state (represented as a vector of facts), compute the
      "next" fact. Return true iff there is a next fact.
    */
    bool increment_to_next_state() const;

    /*
      build the pattern state from the given state id
    */
    const vector<FactPair> &compute_state(int state_id) const;

    /**
     * Perfect hash function that computes the transition_id
     * for a given pair of source state and abstract operator id.
     * Note: the operator must be applicable in the state.
     */
    int get_transition_id(int source_id, int abs_op_id) const;

  private:
    virtual vector<int> compute_goal_distances_for_non_negative_costs_ocf(const vector<int> &ocf) const override;
    virtual vector<int> compute_goal_distances_for_non_negative_costs_tcf(
      const CostFunctionStateDependent &sdac,
      AbstractTransitionCostFunction &tcf) const override;
    virtual vector<int> compute_goal_distances_for_non_negative_costs_tcf(
      AbstractTransitionCostFunction &tcf) const override;

    void print_statistics() const;

  public:
    /**
     * R6: Moveable and not copyable.
     */
    Projection() = delete;
    Projection(
        const TaskInfo &task_info,
        const BddBuilder &bdd_builder,
        unique_ptr<AbstractionFunction> abstraction_function,
        int num_transitions,
        int num_states,
        int init_state_id,
        unordered_set<int> &&goal_states,
        const pdbs::Pattern &pattern,
        vector<size_t> &&hash_multipliers,
        vector<int> &&pattern_domain_sizes,
        vector<AbstractForwardOperator> &&abstract_forward_operators,
        unique_ptr<pdbs::MatchTree> &&match_tree_forward,
        vector<AbstractBackwardOperator> &&abstract_backward_operators,
        unique_ptr<pdbs::MatchTree> &&match_tree_backward,
        vector<int> &&transition_id_offset);
    Projection(const Projection &other) = delete;
    Projection& operator=(const Projection &other) = delete;
    Projection(Projection &&other) = default;
    Projection& operator=(Projection &&other) = default;
    virtual ~Projection();

    virtual void for_each_transition(const TransitionCallback &callback) const override;
    virtual void for_each_transition(const vector<bool> &si, const TransitionCallback &callback) const override;

    virtual vector<bool> compute_reachability_from_state_ocf(const vector<int> &ocf, int state_id) const override;
    virtual vector<bool> compute_reachability_from_state_tcf(AbstractTransitionCostFunction &tcf, int state_id) const override;

    virtual vector<bool> compute_reachability_to_state_ocf(const vector<int> &ocf, int state_id) const override;
    virtual vector<bool> compute_reachability_to_state_tcf(AbstractTransitionCostFunction &tcf, int state_id) const override;

    virtual vector<int> compute_saturated_costs_ocf(const vector<int> &h_values) const override;
    virtual void compute_saturated_costs_tcf(const vector<int> &h_values, AbstractTransitionCostFunction &stcf) const override;

    virtual vector<int> get_split_variables() const override;

    virtual int get_num_transitions(int op_id) const override;
    virtual bool operator_induces_self_loop(int op_id) const override;
    virtual bool operator_is_active(int op_id) const override;

    virtual BDD make_state_bdd(int state_id) const override;
    virtual BDD make_transition_bdd_and_cache(const Transition &transition) const override;
    virtual BDD make_transition_bdd(const Transition &transition) const override;
};

}

#endif
