#include "abstraction.h"

#include "utils.h"
#include "cost_function_state_dependent.h"
#include "task_info.h"
#include "saturator_cap.h"
#include "bdd_builder.h"


namespace transition_cost_partitioning {

// ____________________________________________________________________________
Abstraction::Abstraction(
    const TaskInfo &task_info,
    const BddBuilder &bdd_builder,
    unique_ptr<AbstractionFunction> abstraction_function,
    int num_transitions,
    int num_states,
    int init_state_id,
    unordered_set<int> goal_states) :
    task_info(task_info),
    bdd_builder(bdd_builder),
    abstraction_function(move(abstraction_function)),
    num_transitions(num_transitions),
    num_states(num_states),
    init_state_id(init_state_id),
    goal_states(move(goal_states)) {
}

// ____________________________________________________________________________
void Abstraction::clear_caches() {
    transition_bdd_cache.uninitialize();
}

// ____________________________________________________________________________
void Abstraction::for_each_transition(
    const vector<bool> &si,
    const TransitionCallback &callback) const {
    for_each_transition(
        [&](const Transition &transition){
            if (si[transition.op_id]) {
                return;
            }
            callback(transition);
        }
    );
}

// ____________________________________________________________________________
vector<int> 
Abstraction::compute_goal_distances_for_negative_costs_ocf(
    const vector<int> &ocf) const {
    int num_states = get_num_states();
    vector<int> distances(num_states, INF);

    bool negative_weight_cycle_found = false;
    do {
        negative_weight_cycle_found = false;
        for (int &d : distances) {
            // Reset distances but keep distances of -\infty.
            if (d != -INF) {
                d = INF;
            }
        }
        for (int goal : get_goal_states()) {
            distances[goal] = 0;
        }
        for (int i = 0; i < num_states; ++i) {
            bool last_round = (i == num_states - 1);
            // Optimization: break if no distances change.
            bool distances_changed = false;
            for_each_transition(
                [this, last_round, &negative_weight_cycle_found, &distances_changed,
                 &ocf, &distances](const Transition &transition) {
                    // Convert forward to backward transition.
                    int src = transition.target_id;
                    int target = transition.source_id;
                    assert(utils::in_bounds(transition.op_id, ocf));
                    int cost = ocf[transition.op_id];
                    int new_distance = path_addition(distances[src], cost);
                    if (new_distance < distances[target]) {
                        if (last_round) {
                            negative_weight_cycle_found = true;
                            // For all states s that can reach target, set h(s) = -\infty.
                            vector<bool> can_reach_target =
                                compute_reachability_to_state_ocf(
                                    compute_reachability_cost_function(ocf), target);
                            for (size_t i = 0; i < can_reach_target.size(); ++i) {
                                if (can_reach_target[i]) {
                                    distances[target] = -INF;
                                }
                            }
                            assert(distances[target] == -INF);
                            return;
                        } else {
                            distances[target] = new_distance;
                            distances_changed = true;
                        }
                    }
                });
            if (!distances_changed) {
                break;
            }
        }
    } while (negative_weight_cycle_found);

    return distances;
}

// ____________________________________________________________________________
vector<int> 
Abstraction::compute_goal_distances_for_negative_costs_tcf(
    AbstractTransitionCostFunction &tcf) const {
    const vector<int> &sd_costs = tcf.get_sd_costs();
    int num_states = get_num_states();
    vector<int> distances(num_states, INF);

    bool negative_weight_cycle_found = false;
    do {
        negative_weight_cycle_found = false;
        for (int &d : distances) {
            // Reset distances but keep distances of -\infty.
            if (d != -INF) {
                d = INF;
            }
        }
        for (int goal : get_goal_states()) {
            distances[goal] = 0;
        }
        for (int i = 0; i < num_states; ++i) {
            bool last_round = (i == num_states - 1);
            // Optimization: break if no distances change.
            bool distances_changed = false;
            for_each_transition(
                [&](const Transition &transition) {
                    // Convert forward to backward transition.
                    int src = transition.target_id;
                    assert(utils::in_bounds(transition.transition_id, sd_costs));
                    int target = transition.source_id;
                    int cost = sd_costs[transition.transition_id];
                    int new_distance = path_addition(distances[src], cost);
                    if (new_distance < distances[target]) {
                        if (last_round) {
                            negative_weight_cycle_found = true;
                            // For all states s that can reach target, set h(s) = -\infty.
                            vector<bool> can_reach_target =
                                compute_reachability_to_state_tcf(tcf, target);
                            for (size_t i = 0; i < can_reach_target.size(); ++i) {
                                if (can_reach_target[i]) {
                                    distances[target] = -INF;
                                }
                            }
                            assert(distances[target] == -INF);
                            return;
                        } else {
                            distances[target] = new_distance;
                            distances_changed = true;
                        }
                    }
                });
            if (!distances_changed) {
                break;
            }
        }
    } while (negative_weight_cycle_found);

    return distances;
}

// ____________________________________________________________________________
vector<int> 
Abstraction::compute_goal_distances_ocf(
  const vector<int> &ocf) const {
    if (all_of(ocf.begin(), ocf.end(), [](int c) {return c >= 0;})) {
        return compute_goal_distances_for_non_negative_costs_ocf(ocf);
    } else {
        return compute_goal_distances_for_negative_costs_ocf(ocf);
    }
}

// ____________________________________________________________________________
vector<int> 
Abstraction::compute_goal_distances_tcf(
    const CostFunctionStateDependent &sdac,
    AbstractTransitionCostFunction &tcf) const {
    return compute_goal_distances_for_non_negative_costs_tcf(sdac, tcf);
}

// ____________________________________________________________________________
vector<int> 
Abstraction::compute_goal_distances_tcf(
    AbstractTransitionCostFunction &tcf) const {
    assert(tcf.is_nonnegative());
    return compute_goal_distances_for_non_negative_costs_tcf(tcf);
}

// ____________________________________________________________________________
unique_ptr<AbstractionFunction> Abstraction::extract_abstraction_function() {
    return move(abstraction_function);
}

// ____________________________________________________________________________
const vector<bool> &Abstraction::get_reachability_from_init() const {
    if (reachability_from_init.empty()) {
        reachability_from_init = compute_reachability_from_state_ocf(task_info.get_operator_costs(), get_initial_state_id());
    }
    return reachability_from_init;
}

// ____________________________________________________________________________
const unordered_set<int> &Abstraction::get_goal_states() const {
    assert(goal_states.size() > 0);
    return goal_states;
}

// ____________________________________________________________________________
bool Abstraction::is_goal_state(int state_id) const {
    if (goal_states.find(state_id) != goal_states.end()) {
        return true;
    }
    return false;
}

// ____________________________________________________________________________
int Abstraction::get_num_operators() const {
    return task_info.get_num_operators();
}

// ____________________________________________________________________________
int Abstraction::get_num_states() const {
    assert(num_states > 0);
    return num_states;
}

// ____________________________________________________________________________
int Abstraction::get_num_transitions() const {
    assert(num_transitions >= 0);
    return num_transitions;
}

// ____________________________________________________________________________
int Abstraction::get_initial_state_id() const {
    return init_state_id;
}

// ____________________________________________________________________________
int Abstraction::get_abstract_state_id(const State &concrete_state) const {
    return abstraction_function->get_abstract_state_id(concrete_state);
}

}