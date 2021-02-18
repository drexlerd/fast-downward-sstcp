#ifndef TRANSITION_COST_PARTITIONING_TYPES_H
#define TRANSITION_COST_PARTITIONING_TYPES_H

#include "../abstract_task.h"

#include "../cegar/abstract_state.h"
#include "../cegar/abstraction.h"
#include "../cegar/split_tree.h"
#include "../cegar/split_selector.h"
#include "../cegar/cartesian_set.h"
#include "../cegar/transition.h"
#include "../cegar/types.h"

#include "gmpxx.h"
#include "cudd.h"
#include "cuddObj.hh"

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <memory>
#include <functional>

namespace transition_cost_partitioning {
class Abstraction;
class AbstractionFunction;
class CostPartitioningHeuristic;
class Saturator;

using Facts = std::vector<FactPair>;

using AbstractionFunctions = std::vector<std::unique_ptr<AbstractionFunction>>;

using Abstractions = std::vector<std::unique_ptr<Abstraction>>;

using NodeID = int;

using CPFunction = std::function<CostPartitioningHeuristic(const Abstractions &, const std::vector<int> &, const std::vector<int> &)>;

using CPHeuristics = std::vector<CostPartitioningHeuristic>;
using Order = std::vector<int>;
using Saturators = std::vector<std::shared_ptr<Saturator>>;

// Positive infinity. The name "INFINITY" is taken by an ISO C99 macro.
const int INF = std::numeric_limits<int>::max();

// Undefined is used for variables that range over natural numbers, 
// i.e. indexing variables or planning task variable domains.
const int UNDEFINED = -1;

// Multiply all costs by this factor to avoid using real-valued costs.
const int COST_FACTOR = 1000;

}

#endif
