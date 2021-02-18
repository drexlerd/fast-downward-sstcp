#ifndef TRANSITION_COST_PARTITIONING_SATURATED_COST_PARTITIONING_ONLINE_HEURISTIC_H
#define TRANSITION_COST_PARTITIONING_SATURATED_COST_PARTITIONING_ONLINE_HEURISTIC_H

#include "types.h"

#include "../heuristic.h"
#include "../utils/timer.h"

#include <memory>
#include <vector>
#include <utility>

namespace transition_cost_partitioning {
class OrderGenerator;
class CostFunctionStateDependent;
class OperatorMaskGenerator;
class AbstractionMaskGenerator;
class TaskInfo;


/**
 */
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
        Abstractions &&abstractions,
        vector<int> &&costs);
    virtual ~SaturatedCostPartitioningOnlineHeuristic() override;
};

// compute saturated cost partitioning with saturators
CostPartitioningHeuristic compute_saturated_cost_partitioning_with_saturators(
    const TaskInfo &task_info,
    const Abstractions &abstractions,
    const OperatorMaskGenerator &operator_mask_generator,
    const AbstractionMaskGenerator &abstraction_mask_generator,
    const std::vector<int> &order,
    const Saturators &saturators,
    const std::vector<int> &abstract_state_ids,
    CostFunctionStateDependent &sdac,
    Stats &stats);

}

#endif
