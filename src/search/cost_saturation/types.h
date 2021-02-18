#ifndef COST_SATURATION_TYPES_H
#define COST_SATURATION_TYPES_H

#include <functional>
#include <limits>
#include <memory>
#include <vector>

namespace cost_saturation {
class Abstraction;
class AbstractionFunction;
class CostPartitioningHeuristic;
class Saturator;

struct Successor {
    int op;
    int state;

    Successor(int op, int state)
        : op(op),
          state(state) {
    }

    bool operator<(const Successor &other) const {
        return std::make_pair(op, state) < std::make_pair(other.op, other.state);
    }

    bool operator>=(const Successor &other) const {
        return !(*this < other);
    }
};

// Positive infinity. The name "INFINITY" is taken by an ISO C99 macro.
const int INF = std::numeric_limits<int>::max();

using Abstractions = std::vector<std::unique_ptr<Abstraction>>;
using AbstractionFunctions = std::vector<std::unique_ptr<AbstractionFunction>>;
using CPFunction = std::function<CostPartitioningHeuristic(
                                     const Abstractions &, const std::vector<int> &, const std::vector<int> &)>;
using CPHeuristics = std::vector<CostPartitioningHeuristic>;
using Graph = std::vector<std::vector<Successor>>;
using Order = std::vector<int>;
using Saturators = std::vector<std::shared_ptr<Saturator>>;
}

#endif
