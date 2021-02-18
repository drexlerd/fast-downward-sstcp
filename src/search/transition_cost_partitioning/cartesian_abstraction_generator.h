#ifndef TRANSITION_COST_PARTITIONING_CARTESIAN_ABSTRACTION_GENERATOR_H
#define TRANSITION_COST_PARTITIONING_CARTESIAN_ABSTRACTION_GENERATOR_H

#include "abstraction_generator.h"

#include "types.h"

namespace options {
class Options;
}

namespace cegar {
class SubtaskGenerator;
}

namespace utils {
class RandomNumberGenerator;
}

namespace transition_cost_partitioning {


class CartesianAbstractionGenerator : public AbstractionGenerator {
    const std::vector<std::shared_ptr<cegar::SubtaskGenerator>> subtask_generators;
    const int max_states;
    const int max_transitions;
    const std::shared_ptr<utils::RandomNumberGenerator> rng;
    const cegar::PickSplit pick_split;
    const bool debug;

    int num_states;
    int num_transitions;

    void build_abstractions_for_subtasks(
        const std::vector<std::shared_ptr<AbstractTask>> &subtasks,
        const TaskInfo &task_info,
        const BddBuilder &bdd_builder,
        std::function<bool()> total_size_limit_reached,
        vector<unique_ptr<Abstraction>> &abstractions);

public:
    /**
     * R6: Moveable and not copyable.
     */
    CartesianAbstractionGenerator() = delete;
    explicit CartesianAbstractionGenerator(const options::Options &opts);
    CartesianAbstractionGenerator(const CartesianAbstractionGenerator &other) = delete;
    CartesianAbstractionGenerator& operator=(const CartesianAbstractionGenerator &other) = delete;
    CartesianAbstractionGenerator(CartesianAbstractionGenerator &&other) = default;
    CartesianAbstractionGenerator& operator=(CartesianAbstractionGenerator &&other) = default;
    ~CartesianAbstractionGenerator() = default;

    vector<unique_ptr<Abstraction>> generate_abstractions(
        const std::shared_ptr<AbstractTask> &task,
        const TaskInfo &task_info,
        const BddBuilder &bdd_builder);
};
}

#endif
