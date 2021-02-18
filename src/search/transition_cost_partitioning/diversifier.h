#ifndef TRANSITION_COST_PARTITIONING_DIVERSIFIER_H
#define TRANSITION_COST_PARTITIONING_DIVERSIFIER_H

#include "types.h"

namespace transition_cost_partitioning {
class CostPartitioningHeuristic;

class Diversifier {
    std::vector<std::vector<int>> abstract_state_ids_by_sample;
    std::vector<int> portfolio_h_values;

public:
    explicit Diversifier(std::vector<std::vector<int>> &&abstract_state_ids_by_sample);

    /* Return true iff the cost-partitioned heuristic has a higher heuristic
       value than all previously seen heuristics for at least one sample. */
    bool is_diverse(const CostPartitioningHeuristic &cp_heuristic);

    int compute_sum_portfolio_h_value_for_samples() const;
};
}

#endif
