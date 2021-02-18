#include "abstraction_function.h"

namespace transition_cost_partitioning {

// ____________________________________________________________________________
CartesianAbstractionFunction::CartesianAbstractionFunction(
  unique_ptr<cegar::RefinementHierarchy> refinement_hierarchy)
  : refinement_hierarchy(move(refinement_hierarchy)) {
}

// ____________________________________________________________________________
int CartesianAbstractionFunction::get_abstract_state_id(const State &concrete_state) const {
    return refinement_hierarchy->get_abstract_state_id(concrete_state);
}

// ____________________________________________________________________________
ProjectionFunction::ProjectionFunction(const pdbs::Pattern &pattern, vector<size_t> &hash_multipliers_)
    : pattern(pattern),
    hash_multipliers(hash_multipliers_) {
    assert(pattern.size() == hash_multipliers.size());
}

// ____________________________________________________________________________
int ProjectionFunction::get_abstract_state_id(const State &concrete_state) const {
    int index = 0;
    for (size_t i = 0; i < pattern.size(); ++i) {
        index += hash_multipliers[i] * concrete_state[pattern[i]].get_value();
    }
    return index;
}

}
