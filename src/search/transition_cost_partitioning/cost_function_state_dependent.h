#ifndef TRANSITION_COST_PARTITIONING_COST_FUNCTION_TRANSITION_H
#define TRANSITION_COST_PARTITIONING_COST_FUNCTION_TRANSITION_H

#include "types.h"

using namespace std;

namespace transition_cost_partitioning {
class AbstractTransitionCostFunction;
class TaskInfo;
class BddBuilder;
struct Transition;

/**
 * CostFunctionStateDependent to represent and manipulate functions of the form c : S x O -> R
 */
class CostFunctionStateDependent {
    /**
     * Reference to get task related information.
     */
    const TaskInfo &task_info;
    /**
     * Reference to build bdds.
     */
    const BddBuilder &bdd_builder;

    /**
     * Maximum number of buckets for each operator.
     * If the number of buckets exceeds the threshold
     * then merge buckets together such that the resulting
     * cost function is an underapproximation of the remaining costs.
     * This is reasonable because otherwise the computation
     * might not terminate due to combinatorial explosion in buckets.
     * This happens in instances where operators have different costs.
     */
    const int max_buckets;

    /**
     * Whether multiple orders are considered.
     * If diversify is true then we cache transition bdds for reuse.
     * This is reasonable because we do not want to do the same work twice.
     */
    const bool diversify;

    /**
     * Store state-dependent costs.
     */
    vector<map<int, BDD>> remaining_sd_costs;

    /**
     * useless_operators[o] = true iff operator o can be removed from the task.
     */
    vector<bool> useless_operators;

    /**
     * Collect statistics.
     */
    mutable int count_evaluations;
    mutable int count_subtractions;

  private:
    /**
     * Returns true if the cost function is in a legal state.
     * More info in function definition.
     */
    bool verify_cost_function_state_space() const;

    /**
     * Returns true if the cost function is in a legal state.
     * More info in function definition.
     */
    bool verify_cost_function_state_space(int op_id) const;

    /**
     * Subtract saturated cost of a single operator.
     */
    void reduce_operator_costs(
        int op_id, int saturated);

  public:
    /**
     * R6: Moveable and not copyable.
     */
    CostFunctionStateDependent() = delete;
    CostFunctionStateDependent(const TaskInfo &task_info, const BddBuilder &bdd_builder, int max_buckets, bool diversify);
    CostFunctionStateDependent(const CostFunctionStateDependent& other) = delete;
    CostFunctionStateDependent& operator=(const CostFunctionStateDependent& other) = delete;
    CostFunctionStateDependent(CostFunctionStateDependent &&other) = default;
    CostFunctionStateDependent& operator=(CostFunctionStateDependent&& other) = default;
    ~CostFunctionStateDependent() = default;

    /**
     * Reinitialize the cost function with the initially given operator costs.
     */
    void reinitialize();

    /**
     * Determine the remaining operator cost function.
     */
    vector<int> determine_remaining_costs_operator() const;

    /**
     * Determine the remaining operator cost of a single operator.
     */
    int determine_remaining_costs_operator(int op_id) const;

    /**
     * Determine the remaining transition cost function.
     */
    vector<int> determine_remaining_costs_transition(
        const Abstraction &abstraction) const;

    /**
     * Determine the remaining abstract transition cost function.
     */
    void determine_remaining_abstract_transition_cost_function(
        const Abstraction &abstraction,
        AbstractTransitionCostFunction &tcf) const;

    /**
     * Determine the remaining transition cost of a single transition.
     */
    int determine_remaining_costs_transition(
        const Abstraction &abstraction,
        const Transition &transition) const;

    /**
     * Determine the remaining transition cost of a single transition.
     */
    int determine_remaining_costs_transition(
        const Abstraction &abstraction,
        const Transition &transition,
        int required) const;

    /**
     * Reduce the state-independent costs for this operator
     */
    void reduce_operator_costs(
        const vector<int> &socf);

    /**
     * Reduce the state-independent costs for this operator
     */
    void reduce_operator_costs(
        AbstractTransitionCostFunction &tcf);

    /**
     * Reduce the state-dependent costs for this transition.
     */
    void reduce_transition_costs_finite(
        const Abstraction &abstraction,
        AbstractTransitionCostFunction &tcf);

    /**
     * Handle the costs state-dependent for unreachable transitions/loops
     */
    void reduce_transition_costs_negative_infinity(
        const Abstraction &abstraction,
        const vector<int> &h_values);

    /**
     * Print information about the cost function to the screen.
     */
    void print_statistics() const;
};

}

#endif
