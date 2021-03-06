#ifndef COST_SATURATION_ABSTRACTION_H
#define COST_SATURATION_ABSTRACTION_H

#include "types.h"

#include <cassert>
#include <functional>
#include <memory>
#include <ostream>
#include <vector>

class State;

namespace cost_saturation {
struct Transition;
using TransitionCallback = std::function<void (const Transition &)>;

extern Graph get_forward_graph(const Abstraction &abstraction);
extern std::vector<int> compute_forward_distances(
    const Graph &forward_graph, const std::vector<int> &costs, int state_id);

struct Transition {
    int src;
    int op;
    int target;

    Transition(int src, int op, int target)
        : src(src),
          op(op),
          target(target) {
        assert(src != target);
    }

    bool operator==(const Transition &other) const {
        return src == other.src && op == other.op && target == other.target;
    }

    bool operator<(const Transition &other) const {
        return src < other.src
               || (src == other.src && op < other.op)
               || (src == other.src && op == other.op && target < other.target);
    }

    friend std::ostream &operator<<(std::ostream &os, const Transition &t) {
        return os << "<" << t.src << "," << t.op << "," << t.target << ">";
    }
};


class AbstractionFunction {
public:
    virtual ~AbstractionFunction() = default;
    virtual int get_abstract_state_id(const State &concrete_state) const = 0;
};


class Abstraction {
    std::vector<int> compute_goal_distances_for_general_costs(
        const std::vector<int> &costs) const;

protected:
    std::unique_ptr<AbstractionFunction> abstraction_function;
    virtual std::vector<int> compute_goal_distances_for_non_negative_costs(
        const std::vector<int> &costs) const = 0;

public:
    explicit Abstraction(std::unique_ptr<AbstractionFunction> abstraction_function);
    virtual ~Abstraction() = default;

    Abstraction(const Abstraction &) = delete;

    std::vector<int> compute_goal_distances(
        const std::vector<int> &costs) const;

    virtual std::vector<bool> compute_reachability_from_state(
        const std::vector<int> &costs, int state_id) const = 0;
    virtual std::vector<bool> compute_reachability_to_state(
        const std::vector<int> &costs, int state_id) const = 0;

    virtual std::vector<int> compute_saturated_costs(
        const std::vector<int> &h_values) const = 0;

    virtual int get_num_operators() const = 0;

    // Return true iff operator induced a state-changing transition.
    virtual bool operator_is_active(int op_id) const = 0;

    // Return true iff operator induced a self-loop. Note that an operator may
    // induce both state-changing transitions and self-loops.
    virtual bool operator_induces_self_loop(int op_id) const = 0;

    // Call a function for each state-changing transition.
    virtual void for_each_transition(const TransitionCallback &callback) const = 0;

    virtual int get_num_states() const = 0;
    virtual const std::vector<int> &get_goal_states() const = 0;

    int get_abstract_state_id(const State &concrete_state) const;
    std::unique_ptr<AbstractionFunction> extract_abstraction_function();

    virtual void dump() const = 0;
};
}

#endif
