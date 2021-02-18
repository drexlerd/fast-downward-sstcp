#ifndef TRANSITION_COST_PARTITIONING_ABSTRACTION_TRANSITION_COST_FUNCTION_H
#define TRANSITION_COST_PARTITIONING_ABSTRACTION_TRANSITION_COST_FUNCTION_H

#include "abstraction.h"

using namespace std;

namespace transition_cost_partitioning {
class Abstraction;

/**
 * The AbstractTransitionCostFunction compactly encodes the mixture
 * of state-dependent and state-independent assignments to
 * an abstract transition system.
 * 
 * The saturated cost partitioning pipeline allows classification
 * of operators into either state-dependent or state-independent at two steps:
 * (1) Determine abstract transition weights, and
 * (2) Subtract abstract transition weights.
 * We currently use the operator mask sd for both steps
 * but future work might want to distinguish both steps.
 * In this extension, it is natural to use two separate
 * abstract transition cost functions 
 * and initialize each with an operator mask.
 * In step (2) we additionally allow setting an operator to state-independent
 * if a state-independent cost assignment corresponds to a state-dependent cost assignment
 * because this allows subtraction in constant time.
 */
class AbstractTransitionCostFunction {
private:
    /**
     * sd_costs[i] is the cost of the i-th transition with label op.
     */
    vector<int> sd_costs;
    /**
     * si[op]=true if state-dependent costs never deviate from state-independent costs.
     */
    vector<bool> si;
    /**
     * si_costs[op] is the cost of the operator op.
     */
    vector<int> si_costs;
public:
    /**
     * R6: Moveable but not copyable.
     */
    AbstractTransitionCostFunction();
    AbstractTransitionCostFunction(const Abstraction &abstraction);
    AbstractTransitionCostFunction(const AbstractTransitionCostFunction &other) = delete;
    AbstractTransitionCostFunction& operator=(const AbstractTransitionCostFunction &other) = delete;
    AbstractTransitionCostFunction(AbstractTransitionCostFunction &&other) = default;
    AbstractTransitionCostFunction& operator=(AbstractTransitionCostFunction &&other) = default;
    ~AbstractTransitionCostFunction() = default;

    /**
     * Returns true if the object is default constructed.
     */
    bool is_uninitialized() const;

    /**
     * Returns true if all cost values are nonnegative.
     */
    bool is_nonnegative() const;

    /**
     * In saturated cost partitioning it is convenient to iterate
     * over either operators or transitions
     * and process information into the abstract transition cost function.
     * Therefore, we provide getters that return a reference to each member.
     */
    vector<int> &get_sd_costs();
    vector<bool> &get_si();
    vector<int> &get_si_costs();
};

}

#endif
