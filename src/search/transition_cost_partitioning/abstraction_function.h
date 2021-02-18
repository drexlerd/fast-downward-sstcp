#ifndef TRANSITION_COST_PARTITIONING_ABSTRACTION_FUNCTION_H
#define TRANSITION_COST_PARTITIONING_ABSTRACTION_FUNCTION_H

#include "types.h"

#include "../cegar/refinement_hierarchy.h"
#include "../pdbs/types.h"

using namespace std;

class State;

namespace transition_cost_partitioning {

/*
  An AbstractionFunction is used to retrieve the abstract state id of a given search state.
*/
class AbstractionFunction {
  public:
    virtual ~AbstractionFunction() = default;
    virtual int get_abstract_state_id(const State &concrete_state) const = 0;
};

class CartesianAbstractionFunction : public AbstractionFunction {
    unique_ptr<cegar::RefinementHierarchy> refinement_hierarchy;

public:
    explicit CartesianAbstractionFunction(
        unique_ptr<cegar::RefinementHierarchy> refinement_hierarchy);

    virtual int get_abstract_state_id(const State &concrete_state) const override;
};


class ProjectionFunction : public AbstractionFunction {
    pdbs::Pattern pattern;
    vector<size_t> hash_multipliers;
public:
    ProjectionFunction(const pdbs::Pattern &pattern, vector<size_t> &hash_multipliers_);

    virtual int get_abstract_state_id(const State &concrete_state) const;
};


}

#endif