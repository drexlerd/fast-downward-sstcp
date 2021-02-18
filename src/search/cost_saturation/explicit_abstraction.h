#ifndef COST_SATURATION_EXPLICIT_ABSTRACTION_H
#define COST_SATURATION_EXPLICIT_ABSTRACTION_H

#include "abstraction.h"

#include "../algorithms/priority_queues.h"

#include <memory>
#include <utility>
#include <vector>

namespace cost_saturation {
std::ostream &operator<<(std::ostream &os, const Successor &successor);


class ExplicitAbstraction : public Abstraction {
    // State-changing transitions.
    std::vector<std::vector<Successor>> backward_graph;

    // Operators inducing state-changing transitions.
    std::vector<bool> active_operators;

    // Operators inducing self-loops.
    std::vector<bool> looping_operators;

    std::vector<int> goal_states;

    mutable priority_queues::AdaptiveQueue<int> queue;

protected:
    virtual std::vector<int> compute_goal_distances_for_non_negative_costs(
        const std::vector<int> &costs) const override;

public:
    ExplicitAbstraction(
        std::unique_ptr<AbstractionFunction> abstraction_function,
        std::vector<std::vector<Successor>> &&backward_graph,
        std::vector<bool> &&looping_operators,
        std::vector<int> &&goal_states);

    virtual std::vector<bool> compute_reachability_from_state(
        const std::vector<int> &costs, int state_id) const override;
    virtual std::vector<bool> compute_reachability_to_state(
        const std::vector<int> &costs, int state_id) const override;
    virtual std::vector<int> compute_saturated_costs(
        const std::vector<int> &h_values) const override;
    virtual int get_num_operators() const override;
    virtual bool operator_is_active(int op_id) const override;
    virtual bool operator_induces_self_loop(int op_id) const override;
    virtual void for_each_transition(const TransitionCallback &callback) const override;
    virtual int get_num_states() const override;
    virtual const std::vector<int> &get_goal_states() const override;
    virtual void dump() const override;
};

extern void dijkstra_search(
    const std::vector<std::vector<Successor>> &graph,
    const std::vector<int> &costs,
    priority_queues::AdaptiveQueue<int> &queue,
    std::vector<int> &distances);
}

#endif
