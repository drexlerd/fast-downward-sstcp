#ifndef TRANSITION_COST_PARTITIONING_PROJECTION_FACTORY_H
#define TRANSITION_COST_PARTITIONING_PROJECTION_FACTORY_H

#include "types.h"

#include "../pdbs/types.h"

using namespace std;

namespace pdbs {
class MatchTree;
}

namespace transition_cost_partitioning {
class Abstraction;
class TaskInfo;
class BddBuilder;

/**
 * Factory for constructing Projections.
 * 
 * This allow using initializer list 
 * and get rid of errors 
 * and leads to most efficient code.
 * 
 * Note: Using initializer list efficiently was not possible before
 * because some code initialized multiple member attributes.
 */
class ProjectionFactory { 
public:
    static unique_ptr<Abstraction> convert_abstraction(
        const TaskProxy &task_proxy,        
        const pdbs::Pattern &pattern,
        const TaskInfo &task_info,
        const BddBuilder &bdd_builder);
};

}

#endif
