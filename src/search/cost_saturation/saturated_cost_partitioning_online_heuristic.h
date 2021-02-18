#ifndef COST_SATURATION_SATURATED_COST_PARTITIONING_ONLINE_HEURISTIC_H
#define COST_SATURATION_SATURATED_COST_PARTITIONING_ONLINE_HEURISTIC_H

#include "types.h"

#include "../heuristic.h"
#include "../utils/timer.h"

#include <memory>
#include <vector>

using namespace std;

namespace cost_saturation {
class OrderGenerator;

class Stats {
  public:
    std::string name;
    int evaluations;

    utils::Timer saturator_timer_saturate;
    utils::Timer saturator_timer_reduce;

    Stats(std::string name) : name(name), evaluations(0) {
        saturator_timer_saturate.stop();
        saturator_timer_reduce.stop();
    }

    void print_statistics() const {
        if (evaluations > 0) {
            cout << "Total time " << name << " step saturate: " << saturator_timer_saturate() << "\n";
            cout << "Total time " << name << " step reduce: " << saturator_timer_reduce() << "\n";
            cout << "Average time " << name << " step saturate: " << saturator_timer_saturate() / evaluations << "s" << "\n";
            cout << "Average time " << name << " step reduce: " << saturator_timer_reduce() / evaluations << "s" << "\n";
        }
    }
};


class SaturatedCostPartitioningOnlineHeuristic : public Heuristic {
    const std::shared_ptr<OrderGenerator> cp_generator;
    Abstractions abstractions;
    const std::vector<int> costs;
    const Saturators saturators;

    int num_scps_computed;

    // For statistics.
    mutable std::vector<int> num_best_order;

    void print_statistics() const;

protected:
    virtual int compute_heuristic(const GlobalState &state) override;

public:
    SaturatedCostPartitioningOnlineHeuristic(
        const options::Options &opts,
        Abstractions &&abstractions);
    virtual ~SaturatedCostPartitioningOnlineHeuristic() override;
};

CostPartitioningHeuristic compute_saturated_cost_partitioning_with_saturators(
    const Abstractions &abstractions,
    const std::vector<int> &order,
    const Saturators &saturators,
    std::vector<int> &remaining_costs,
    const std::vector<int> &abstract_state_ids,
    Stats &stats);
}

#endif
