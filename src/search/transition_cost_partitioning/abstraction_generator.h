#ifndef TRANSITION_COST_PARTITIONING_ABSTRACTION_GENERATOR_H
#define TRANSITION_COST_PARTITIONING_ABSTRACTION_GENERATOR_H

#include "types.h"

class AbstractTask;

namespace transition_cost_partitioning {
class Abstraction;
class TaskInfo;
class BddBuilder;

class AbstractionGenerator {
public:
    virtual vector<unique_ptr<Abstraction>> generate_abstractions(
        const std::shared_ptr<AbstractTask> &task,
        const TaskInfo &task_info,
        const BddBuilder &bdd_builder) = 0;

    virtual ~AbstractionGenerator() = default;
};
}

#endif
