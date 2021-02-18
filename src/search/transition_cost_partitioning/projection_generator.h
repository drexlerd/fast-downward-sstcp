#ifndef TRANSITION_COST_PARTITIONING_PROJECTION_GENERATOR_H
#define TRANSITION_COST_PARTITIONING_PROJECTION_GENERATOR_H

#include "abstraction_generator.h"

namespace options {
class Options;
}

namespace pdbs {
class PatternCollectionGenerator;
}

namespace transition_cost_partitioning {

class ProjectionGenerator : public AbstractionGenerator {
    const std::shared_ptr<pdbs::PatternCollectionGenerator> pattern_generator;
    const bool dominance_pruning;
    const bool create_complete_transition_system;
    const bool use_add_after_delete_semantics;
    const bool debug;

public:
    explicit ProjectionGenerator(const options::Options &opts);

    vector<unique_ptr<Abstraction>> generate_abstractions(
        const std::shared_ptr<AbstractTask> &task,
        const TaskInfo &task_info,
        const BddBuilder &bdd_builder);
};
}

#endif
