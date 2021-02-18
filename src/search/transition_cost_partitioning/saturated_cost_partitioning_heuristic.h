#ifndef TRANSITION_COST_PARTITIONING_SATURATED_COST_PARTITIONING_HEURISTIC_H
#define TRANSITION_COST_PARTITIONING_SATURATED_COST_PARTITIONING_HEURISTIC_H

#include "types.h"

#include <vector>

namespace transition_cost_partitioning {
class CostPartitioningHeuristic;

extern CostPartitioningHeuristic compute_saturated_cost_partitioning(
    const Abstractions &abstractions,
    const std::vector<int> &order,
    const std::vector<int> &costs);
}

#endif
