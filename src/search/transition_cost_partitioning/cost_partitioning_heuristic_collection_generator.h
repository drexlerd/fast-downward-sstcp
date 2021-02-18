#ifndef TRANSITION_COST_PARTITIONING_COST_PARTITIONING_HEURISTIC_COLLECTION_GENERATOR_H
#define TRANSITION_COST_PARTITIONING_COST_PARTITIONING_HEURISTIC_COLLECTION_GENERATOR_H

#include "types.h"

class TaskProxy;

namespace utils {
class RandomNumberGenerator;
}

namespace transition_cost_partitioning {
class CostPartitioningHeuristic;
class OrderGenerator;
class TaskInfo;
class CostFunctionStateDependent;
class OperatorMaskGenerator;
class AbstractionMaskGenerator;


class CostPartitioningHeuristicCollectionGenerator {
    const std::shared_ptr<OrderGenerator> order_generator;
    const int max_orders;
    const double max_time;
    const bool diversify;
    const int num_samples;
    const double max_optimization_time;
    const std::shared_ptr<utils::RandomNumberGenerator> rng;

public:
    CostPartitioningHeuristicCollectionGenerator(
        const std::shared_ptr<OrderGenerator> &order_generator,
        int max_orders,
        double max_time,
        bool diversify,
        int num_samples,
        double max_optimization_time,
        const std::shared_ptr<utils::RandomNumberGenerator> &rng);

    std::vector<CostPartitioningHeuristic> generate_cost_partitionings(
        const TaskProxy &task_proxy,
        const Abstractions &abstractions,
        const OperatorMaskGenerator &operator_mask_generator,
        const AbstractionMaskGenerator &abstraction_mask_generator,
        const TaskInfo &task_info,
        const Saturators &saturators,
        const std::shared_ptr<Saturator> &extra_saturator,
        const std::shared_ptr<Saturator> &diversified_saturator,
        CostFunctionStateDependent &cost_function_state_dependent) const;
};
}

#endif
