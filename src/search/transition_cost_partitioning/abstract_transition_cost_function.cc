#include "abstract_transition_cost_function.h"

using namespace std;

namespace transition_cost_partitioning {

// ____________________________________________________________________________
AbstractTransitionCostFunction::AbstractTransitionCostFunction() {
}

// ____________________________________________________________________________
AbstractTransitionCostFunction::AbstractTransitionCostFunction(
    const Abstraction &abstraction) : 
    sd_costs(vector<int>(abstraction.get_num_transitions())),
    si(vector<bool>(abstraction.get_num_operators())),
    si_costs(vector<int>(abstraction.get_num_operators())) {
}

// ____________________________________________________________________________
bool AbstractTransitionCostFunction::is_uninitialized() const {
    assert(sd_costs.empty() && si.empty() && si_costs.empty());
    return sd_costs.empty();
}

// ____________________________________________________________________________
bool AbstractTransitionCostFunction::is_nonnegative() const {
    return all_of(sd_costs.begin(), sd_costs.end(), [](int c) {return c >= 0;}) && 
           all_of(si_costs.begin(), si_costs.end(), [](int c) {return c >= 0;});
}

// ____________________________________________________________________________
vector<int> &AbstractTransitionCostFunction::get_sd_costs() {
    return sd_costs;
}

// ____________________________________________________________________________
vector<bool> &AbstractTransitionCostFunction::get_si() {
    return si;
}

// ____________________________________________________________________________
vector<int> &AbstractTransitionCostFunction::get_si_costs() {
    return si_costs;
}


}