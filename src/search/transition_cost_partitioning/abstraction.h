#ifndef TRANSITION_COST_PARTITIONING_ABSTRACTION_H
#define TRANSITION_COST_PARTITIONING_ABSTRACTION_H

#include "types.h"

#include "abstraction_function.h"
#include "dd_cache.h"

using namespace std;

namespace transition_cost_partitioning {
class AbstractTransitionCostFunction;
class BddBuilder;
class CostFunctionStateDependent;
class TaskInfo;
class TransitionWeights;
struct Transition;
struct AbstractState;

using TransitionCallback = function<void (const Transition &)>;

/**
 * A Successor is a directed arc in forward and backward graph.
 * They contain only the smallest amount of information.
 * That is the target and the operator label.
 */
struct Successor {
    int transition_id;
    int op_id;
    int target_id;

    Successor(int transition_id, int op_id, int target_id) :
      transition_id(transition_id),
      op_id(op_id),
      target_id(target_id) {
    }

    bool operator<(const Successor &other) const {
        return std::make_pair(op_id, target_id) < std::make_pair(other.op_id, other.target_id);
    }

    bool operator>=(const Successor &other) const {
        return !(*this < other);
    }
};

/**
 * A Transition is a labelled state-changing transition with a unique identifier.
 * The numbering of identifier start at 0 such that they can be used for indexing arrays.
 * The indexing is required in Transition Cost Partitioning
 * where costs are assigned to Transitions.
 */
struct Transition {
    int transition_id;
    int op_id;
    int source_id;
    int target_id;

    Transition(int transition_id, int op_id, int source_id, int target_id) :
      transition_id(transition_id), op_id(op_id), source_id(source_id), target_id(target_id) {
    }
};

/**
 * Implements an abstract transition system with abstraction mapping alpha.
 *
 * This implementation focusses on never looping over abstract self-loops
 * due to the fact that this is highly inefficient if the number of self-loops is large.
 * This may look like a problem to compute transition cost functions
 * in states with infinite heuristic values. The problem is solved
 * with iteration over abstract states and performing
 * restrict operation on decision diagrams
 * to find compact representation of the transition cost functions.
 *
 * We can similarly recompute the has_loop and has_outgoing function
 * using bdds efficiently.
 */
class Abstraction {
  protected:
    /**
     * Specialized access to task related information.
     */
    const TaskInfo &task_info;

    const BddBuilder &bdd_builder;

    /**
     * The abstraction functions alpha.
     */
    unique_ptr<AbstractionFunction> abstraction_function;

    /**
     * The number of abstract state-changing transitions.
     */
    const int num_transitions;

    /**
     * The number of abstract states.
     */
    const int num_states;

    /**
     * The abstract initial state.
     */
    const int init_state_id;

    /**
     * The abstract goal states.
     */
    const unordered_set<int> goal_states;

    /**
     * The reachability function from the initial state.
     * Note: It is constructed on first request.
     */
    mutable vector<bool> reachability_from_init;

    /**
     * Cache transition bdds for reuse.
     */
    mutable DDCache<BDD> transition_bdd_cache;

  private:
    /**
     * Compute goal distances with non negative costs using dijkstra.
     */
    virtual vector<int> compute_goal_distances_for_non_negative_costs_ocf(
      const vector<int> &ocf) const = 0;
    virtual vector<int> compute_goal_distances_for_non_negative_costs_tcf(
      const CostFunctionStateDependent &sdac,
      AbstractTransitionCostFunction &tcf) const = 0;
    virtual vector<int> compute_goal_distances_for_non_negative_costs_tcf(
      AbstractTransitionCostFunction &tcf) const = 0;

  protected:
    /**
     * Compute goal distances with negative costs using Bellman-Ford.
     */
    vector<int> compute_goal_distances_for_negative_costs_ocf(
      const vector<int> &ocf) const;
    vector<int> compute_goal_distances_for_negative_costs_tcf(
      AbstractTransitionCostFunction &tcf) const;

  public:
    /**
     * R6: Moveable and not copyable.
     */
    Abstraction() = delete;
    explicit Abstraction(
      const TaskInfo &task_info,
      const BddBuilder &bdd_builder,
      unique_ptr<AbstractionFunction> abstraction_function,
      int num_transitions,
      int num_states,
      int init_state_id,
      unordered_set<int> goal_states);
    Abstraction(const Abstraction &other) = delete;
    Abstraction& operator=(const Abstraction &other) = delete;
    Abstraction(Abstraction &&other) = default;
    Abstraction& operator=(Abstraction &&other) = default;
    virtual ~Abstraction() = default;

    /**
     * Clears all caches. Uses swap trick to ensure that memory is freed correctly.
     */
    void clear_caches();

    /**
     * Apply a function to all state-changing transitions.
     */
    virtual void for_each_transition(const TransitionCallback &callback) const = 0;
    /**
     * Apply a function to all state-changing transitions
     * that are not marked as state-indendepent.
     * Note: This function is useful in the subtraction of state-dependent costs.
     */
    virtual void for_each_transition(
      const vector<bool> &si,
      const TransitionCallback &callback) const;

    /**
     * Compute reachable states from state for operator cost function.
     */
    virtual vector<bool> compute_reachability_from_state_ocf(const vector<int> &ocf, int state_id) const = 0;
    /**
     * Compute reachable states from state for transition cost function.
     */
    virtual vector<bool> compute_reachability_from_state_tcf(AbstractTransitionCostFunction &tcf, int state_id) const = 0;

    /**
     * Compute reachable states to state for operator cost function.
     */
    virtual vector<bool> compute_reachability_to_state_ocf(const vector<int> &ocf, int state_id) const = 0;
    /**
     * Compute reachable states to state for transition cost function.
     */
    virtual vector<bool> compute_reachability_to_state_tcf(AbstractTransitionCostFunction &tcf, int state_id) const = 0;

    /**
     * Compute goal distances for operator cost function.
     */
    vector<int> compute_goal_distances_ocf(const vector<int> &ocf) const;
    /**
     * Compute goal distances from sdac data structure
     * and store the transition weights in the abstract transition cost function.
     */
    vector<int> compute_goal_distances_tcf(
      const CostFunctionStateDependent &sdac,
      AbstractTransitionCostFunction &tcf) const;
    /**
     * Compute goal distances from the abstract transition cost function.
     */
    vector<int> compute_goal_distances_tcf(
      AbstractTransitionCostFunction &tcf) const;

    /**
     * Compute saturated operator cost function.
     */
    virtual vector<int> compute_saturated_costs_ocf(const vector<int> &h_values) const = 0;
    /**
     * Compute saturated transition cost function with byproducts, i.e.,
     * saturated operator cost function and state-dependent operators.
     */
    virtual void compute_saturated_costs_tcf(const vector<int> &h_values, AbstractTransitionCostFunction &stcf) const = 0;

    /**
     * Computes the split rate for each variable.
     */
    virtual vector<int> get_split_variables() const = 0;


    /**
     * Moves out the abstraction function.
     */
    unique_ptr<AbstractionFunction> extract_abstraction_function();

    /**
     * Getters.
     */
    const vector<bool> &get_reachability_from_init() const;
    const unordered_set<int> &get_goal_states() const;
    bool is_goal_state(int state_id) const;
    int get_num_operators() const;
    int get_num_states() const;
    int get_num_transitions() const;
    virtual int get_num_transitions(int op_id) const = 0;
    int get_initial_state_id() const;
    int get_abstract_state_id(const State &concrete_state) const;

    /**
     * These two functions are considered independent of the reachability mapping that is used.
     * For CEGAR abstractions, we exclude deadend states that are present initially.
     * For Projections, we consider all states.
     * The currently best known way to update this is using bdds.
     * It will be considered in future version.
     */
    virtual bool operator_induces_self_loop(int op_id) const = 0;
    virtual bool operator_is_active(int op_id) const = 0;

    /**
     * Compute a representation of a state.
     */
    virtual BDD make_state_bdd(int state_id) const = 0;
    /**
     * Compute a BDD that represents the operator regression.
     * This function caches the result for reuse.
     */
    virtual BDD make_transition_bdd_and_cache(const Transition &transition) const = 0;
    /**
     * Compute a BDD that represents the operator regression.
     */
    virtual BDD make_transition_bdd(const Transition &transition) const = 0;
};

}

#endif
