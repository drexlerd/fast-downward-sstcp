#include "projection.h"

#include "abstract_transition_cost_function.h"
#include "bdd_builder.h"
#include "cost_function_state_dependent.h"
#include "utils.h"
#include "task_info.h"

#include "../task_proxy.h"
#include "../algorithms/priority_queues.h"
#include "../pdbs/match_tree.h"
#include "../task_utils/task_properties.h"
#include "../utils/collections.h"
#include "../utils/logging.h"
#include "../utils/math.h"
#include "../utils/memory.h"

namespace transition_cost_partitioning {

// ____________________________________________________________________________
Projection::Projection(
    const TaskInfo &task_info,
    const BddBuilder &bdd_builder,
    unique_ptr<AbstractionFunction> abstraction_function,
    int num_transitions,
    int num_states,
    int init_state_id,
    unordered_set<int> &&goal_states,
    const pdbs::Pattern &pattern,
    vector<size_t> &&hash_multipliers,
    vector<int> &&pattern_domain_sizes,
    vector<AbstractForwardOperator> &&abstract_forward_operators,
    unique_ptr<pdbs::MatchTree> &&match_tree_forward,
    vector<AbstractBackwardOperator> &&abstract_backward_operators,
    unique_ptr<pdbs::MatchTree> &&match_tree_backward,
    vector<int> &&transition_id_offset) :
    Abstraction(task_info, bdd_builder, move(abstraction_function), num_transitions, num_states, init_state_id, move(goal_states)),
    pattern(pattern),
    hash_multipliers(move(hash_multipliers)),
    pattern_domain_sizes(move(pattern_domain_sizes)),
    abstract_forward_operators(move(abstract_forward_operators)),
    match_tree_forward(move(match_tree_forward)),
    abstract_backward_operators(move(abstract_backward_operators)),
    match_tree_backward(move(match_tree_backward)),
    transition_id_offset(move(transition_id_offset)) {
    assert(utils::is_sorted_unique(this->pattern));
    assert(this->pattern.size() > 0);
    // print_statistics();
}

// ____________________________________________________________________________
Projection::~Projection() {
}

// ____________________________________________________________________________
bool Projection::increment_to_next_state() const {
    for (FactPair &fact : abstract_facts) {
        ++fact.value;
        if (fact.value > pattern_domain_sizes[fact.var] - 1) {
            fact.value = 0;
        } else {
            return true;
        }
    }
    return false;
}

// ____________________________________________________________________________
const vector<FactPair> &Projection::compute_state(int state_id) const {
    state_facts.clear();
    // state facts ordered descending for variable id (= consistent with bdd topdown variable order)
    for (int i = pattern.size() - 1; i >= 0; --i) {
        int temp = state_id / hash_multipliers[i];
        int val = temp % pattern_domain_sizes[i];
        int var = pattern[i];
        state_facts.emplace_back(var, val);
    }
    return state_facts;
}

// ____________________________________________________________________________
vector<int> Projection::compute_goal_distances_for_non_negative_costs_ocf(const vector<int> &ocf) const {
    assert(all_of(ocf.begin(), ocf.end(), [](int c) {return c >= 0;}));
    vector<int> distances(get_num_states(), INF);

    // Initialize queue.
    priority_queues::AdaptiveQueue<size_t> pq;
    for (int goal_state_id : goal_states) {
        pq.push(0, goal_state_id);
        distances[goal_state_id] = 0;
    }

    // Reuse vector to save allocations.
    vector<int> applicable_operator_ids;

    // Run Dijkstra loop.
    while (!pq.empty()) {
        pair<int, size_t> node = pq.pop();
        int distance = node.first;
        size_t state_index = node.second;
        assert(utils::in_bounds(state_index, distances));
        if (distance > distances[state_index]) {
            continue;
        }

        // Regress abstract state.
        applicable_operator_ids.clear();
        match_tree_backward->get_applicable_operator_ids(
            state_index, applicable_operator_ids);
        for (int abs_op_id : applicable_operator_ids) {
            const AbstractBackwardOperator &op = abstract_backward_operators[abs_op_id];
            size_t predecessor = state_index + op.hash_effect;
            int conc_op_id = op.concrete_operator_id;
            assert(utils::in_bounds(conc_op_id, ocf));
            int alternative_cost = (ocf[conc_op_id] == INF) ?
                INF : distances[state_index] + ocf[conc_op_id];
            assert(utils::in_bounds(predecessor, distances));
            if (alternative_cost < distances[predecessor]) {
                distances[predecessor] = alternative_cost;
                pq.push(alternative_cost, predecessor);
            }
        }
    }
    return distances;
}

// ____________________________________________________________________________
vector<int>
Projection::compute_goal_distances_for_non_negative_costs_tcf(
    const CostFunctionStateDependent &sdac,
    AbstractTransitionCostFunction &tcf) const {
    vector<int> distances(get_num_states(), INF);
    vector<int> &sd_costs = tcf.get_sd_costs();
    fill(sd_costs.begin(), sd_costs.end(), 0);
    // Initialize queue.
    priority_queues::AdaptiveQueue<size_t> pq;
    for (int goal_state_id : goal_states) {
        pq.push(0, goal_state_id);
        distances[goal_state_id] = 0;
    }

    // Reuse vector to save allocations.
    vector<int> applicable_operator_ids;

    // Run Dijkstra loop.
    while (!pq.empty()) {
        pair<int, size_t> node = pq.pop();
        int distance = node.first;
        size_t state = node.second;
        int state_distance = distances[state];
        assert(state_distance <= distance);
        if (distance > state_distance) {
            continue;
        }
        // Regress abstract state.
        applicable_operator_ids.clear();
        match_tree_backward->get_applicable_operator_ids(
            state, applicable_operator_ids);
        for (int abs_op_id : applicable_operator_ids) {
            const AbstractBackwardOperator &op = abstract_backward_operators[abs_op_id];
            size_t successor = state + op.hash_effect;
            int conc_op_id = op.concrete_operator_id;
            int required = distances[successor] - distances[state];
            int cost = 0;
            int transition_id = get_transition_id(successor, abs_op_id);
            if (required > 0) {
                cost = sdac.determine_remaining_costs_operator(conc_op_id);
                if (cost < required) {
                    cost = sdac.determine_remaining_costs_transition(*this, Transition(transition_id, conc_op_id, successor, state), required);
                }
            }
            assert(cost >= 0);
            sd_costs[transition_id] = cost;
            int successor_distance = (cost == INF) ? INF : state_distance + cost;
            assert(successor_distance >= 0);
            if (distances[successor] > successor_distance) {
                distances[successor] = successor_distance;
                pq.push(successor_distance, successor);
            }
        }
    }
    return distances;
}

// ____________________________________________________________________________
vector<int>
Projection::compute_goal_distances_for_non_negative_costs_tcf(
    AbstractTransitionCostFunction &tcf) const {
    vector<int> distances(get_num_states(), INF);
    const vector<int> &sd_costs = tcf.get_sd_costs();
    // Initialize queue.
    priority_queues::AdaptiveQueue<size_t> pq;
    for (int goal_state_id : goal_states) {
        pq.push(0, goal_state_id);
        distances[goal_state_id] = 0;
    }

    // Reuse vector to save allocations.
    vector<int> applicable_operator_ids;

    // Run Dijkstra loop.
    while (!pq.empty()) {
        pair<int, size_t> node = pq.pop();
        int distance = node.first;
        size_t state = node.second;
        int state_distance = distances[state];
        assert(state_distance <= distance);
        if (distance > state_distance) {
            continue;
        }
        // Regress abstract state.
        applicable_operator_ids.clear();
        match_tree_backward->get_applicable_operator_ids(
            state, applicable_operator_ids);
        for (int abs_op_id : applicable_operator_ids) {
            const AbstractBackwardOperator &op = abstract_backward_operators[abs_op_id];
            size_t successor = state + op.hash_effect;
            int cost = sd_costs[get_transition_id(successor, abs_op_id)];
            assert(cost >= 0);
            int successor_distance = (cost == INF) ? INF : state_distance + cost;
            assert(successor_distance >= 0);
            if (distances[successor] > successor_distance) {
                distances[successor] = successor_distance;
                pq.push(successor_distance, successor);
            }
        }
    }
    return distances;
}


// ____________________________________________________________________________
int Projection::get_transition_id(int source_id, int abs_op_id) const {
    int precondition_hash = abstract_forward_operators[abs_op_id].precondition_hash;
    int concrete_op_id = abstract_backward_operators[abs_op_id].concrete_operator_id;
    int state = source_id - precondition_hash;
    /* source_id = precondition_hash[o] + sum_{v not mentioned in operator and v in pattern} hash_multipliers[v] * s[v] */
    // Step 1: extract underlying state values s[v].
    abstract_facts.clear();
    // abstract facts are stored reversed (compared to for_each_transition).
    for (int i = (int)pattern.size() - 1; i >= 0; --i) {
        int var = pattern[i];
        if (!task_info.operator_mentions_variable(concrete_op_id, var)) {
            int val = state / hash_multipliers[i];
            state -= val * hash_multipliers[i];
            abstract_facts.emplace_back(i, val);
        }
    }
    assert(state == 0);
    // Step 2: find new representation in space of hash_multipliers without any gaps.
    // move to specific range of transition ids that belong to this abstract operator.
    int transition_id = transition_id_offset[abs_op_id];
    // the hash multiplier changes according to domain size.
    int hash_multiplier = 1;
    // reverse abstract facts again to fit for_each_transition again.
    for (int i = 0; i < (int)abstract_facts.size(); ++i) {
        FactPair &fact = abstract_facts[(int)abstract_facts.size() - 1 - i];
        transition_id += hash_multiplier * fact.value;
        hash_multiplier *= pattern_domain_sizes[fact.var];
    }
    return transition_id;
}


// ____________________________________________________________________________
void Projection::for_each_transition(const TransitionCallback &callback) const {
    int num_abstract_operators = abstract_forward_operators.size();
    int transition_id = 0;
    for (int op_id = 0; op_id < num_abstract_operators; ++op_id) {
        const AbstractForwardOperator &op = abstract_forward_operators[op_id];
        int concrete_op_id = abstract_backward_operators[op_id].concrete_operator_id;
        abstract_facts.clear();
        for (size_t i = 0; i < pattern.size(); ++i) {
            int var = pattern[i];
            /* for value of a variable that occurs only in the effect
               an abstract operator is build.
               for value of precondition this is already considered in precondition.hash
               for all other variables we need to increment_to_next_state.
            */
            if (!task_info.operator_mentions_variable(concrete_op_id, var)) {
                abstract_facts.emplace_back(i, 0);
            }
        }

        // Note: This shows how to compute S x O' -> N:
        // subtract precondition_hash from state and hash multipliers.
        // we should precompute the mask as given above.
        bool has_next_match = true;
        while (has_next_match) {
            int state = op.precondition_hash;
            for (const FactPair &fact : abstract_facts) {
                state += hash_multipliers[fact.var] * fact.value;
            }
            callback(Transition(transition_id,
                                concrete_op_id,
                                state,
                                state + op.hash_effect));
            ++transition_id;
            assert(state != state + op.hash_effect);
            has_next_match = increment_to_next_state();
        }
    }
}

// ____________________________________________________________________________
void Projection::for_each_transition(
    const vector<bool> &si,
    const TransitionCallback &callback) const {
    int num_abstract_operators = abstract_forward_operators.size();
    int transition_id = 0;
    for (int op_id = 0; op_id < num_abstract_operators; ++op_id) {
        const AbstractForwardOperator &op = abstract_forward_operators[op_id];
        int concrete_op_id = abstract_backward_operators[op_id].concrete_operator_id;
        if (si[concrete_op_id]) {
            transition_id += task_info.get_num_transitions_from_abstract_operator(pattern, concrete_op_id);
            continue;
        }
        abstract_facts.clear();
        for (size_t i = 0; i < pattern.size(); ++i) {
            int var = pattern[i];
            if (!task_info.operator_mentions_variable(concrete_op_id, var)) {
                abstract_facts.emplace_back(i, 0);
            }
        }
        bool has_next_match = true;
        while (has_next_match) {
            int state = op.precondition_hash;
            for (const FactPair &fact : abstract_facts) {
                state += hash_multipliers[fact.var] * fact.value;
            }
            callback(Transition(transition_id,
                                concrete_op_id,
                                state,
                                state + op.hash_effect));
            ++transition_id;
            assert(state != state + op.hash_effect);
            has_next_match = increment_to_next_state();
        }
    }
}

// ____________________________________________________________________________
vector<bool> Projection::compute_reachability_from_state_ocf(const vector<int> &ocf, int state_id) const {
    vector<bool> reachable(get_num_states(), false);
    vector<int> open = {state_id};
    reachable[state_id] = true;
    vector<int> applicable_op_ids;
    while (!open.empty()) {
        int current_state = open.back();
        open.pop_back();
        applicable_op_ids.clear();
        match_tree_forward->get_applicable_operator_ids(current_state, applicable_op_ids);
        for (int op_id : applicable_op_ids) {
            int successor = current_state + abstract_forward_operators[op_id].hash_effect;
            if (!reachable[successor] &&
                ocf[abstract_backward_operators[op_id].concrete_operator_id] != INF) {
                reachable[successor] = true;
                open.push_back(successor);
            }
        }
    }
    return reachable;
}

// ____________________________________________________________________________
vector<bool> Projection::compute_reachability_from_state_tcf(
    AbstractTransitionCostFunction &tcf,
    int state_id) const {
    const vector<int> &sd_costs = tcf.get_sd_costs();
    // Initialize queue
    vector<bool> reachable(get_num_states(), false);
    vector<int> open = {state_id};
    reachable[state_id] = true;
    vector<int> applicable_op_ids;
    while (!open.empty()) {
        int current_state = open.back();
        open.pop_back();
        applicable_op_ids.clear();
        match_tree_forward->get_applicable_operator_ids(current_state, applicable_op_ids);
        for (int op_id : applicable_op_ids) {
            int successor = current_state + abstract_forward_operators[op_id].hash_effect;
            int cost = sd_costs[get_transition_id(current_state, op_id)];
            if (!reachable[successor] &&
                cost != INF) {
                reachable[successor] = true;
                open.push_back(successor);
            }
        }
    }
    return reachable;
}

// ____________________________________________________________________________
vector<bool> Projection::compute_reachability_to_state_ocf(const vector<int> &ocf, int state_id) const {
    vector<bool> reachable(get_num_states(), false);
    vector<int> open = {state_id};
    reachable[state_id] = true;
    vector<int> applicable_op_ids;
    while (!open.empty()) {
        int current_state = open.back();
        open.pop_back();
        applicable_op_ids.clear();
        match_tree_backward->get_applicable_operator_ids(current_state, applicable_op_ids);
        for (int op_id : applicable_op_ids) {
            int predecessor = current_state + abstract_backward_operators[op_id].hash_effect;
            int conc_op_id = abstract_backward_operators[op_id].concrete_operator_id;
            if (!reachable[predecessor] &&
                ocf[conc_op_id] != INF) {
                reachable[predecessor] = true;
                open.push_back(predecessor);
            }
        }
    }
    return reachable;
}

// ____________________________________________________________________________
vector<bool> Projection::compute_reachability_to_state_tcf(AbstractTransitionCostFunction &tcf, int state_id) const {
    const vector<int> &sd_costs = tcf.get_sd_costs();
    // Initialize queue
    vector<bool> reachable(get_num_states(), false);
    vector<int> open = {state_id};
    reachable[state_id] = true;
    vector<int> applicable_op_ids;
    while (!open.empty()) {
        int current_state = open.back();
        open.pop_back();
        applicable_op_ids.clear();
        match_tree_backward->get_applicable_operator_ids(current_state, applicable_op_ids);
        for (int op_id : applicable_op_ids) {
            int predecessor = current_state + abstract_backward_operators[op_id].hash_effect;
            int cost = sd_costs[get_transition_id(current_state, op_id)];
            if (!reachable[predecessor] &&
                cost != INF) {
                reachable[predecessor] = true;
                open.push_back(predecessor);
            }
        }
    }
    return reachable;
}

// ____________________________________________________________________________
vector<int>
Projection::compute_saturated_costs_ocf(
    const vector<int> &h_values) const {
    assert(get_num_states() == (int)h_values.size());
    int num_operators = task_info.get_num_operators();
    vector<int> socf(num_operators, -INF);
    /* To prevent negative cost cycles, we ensure that all operators
       inducing self-loops have non-negative costs. */
    for (int op_id = 0; op_id < num_operators; ++op_id) {
        if (operator_induces_self_loop(op_id)) {
            socf[op_id] = 0;
        }
    }
    for_each_transition(
        [&](const Transition &transition) {
            int source_id = transition.source_id;
            int target_id = transition.target_id;
            assert(utils::in_bounds(source_id, h_values));
            assert(utils::in_bounds(target_id, h_values));
            int source_h = h_values[source_id];
            int target_h = h_values[target_id];
            if (source_h == INF || target_h == INF ||
                source_h == -INF || target_h == -INF) {
                return;
            }
            int op_id = transition.op_id;
            int &needed_costs = socf[op_id];
            needed_costs = max(needed_costs, source_h - target_h);
        }
    );
    return socf;
}

// ____________________________________________________________________________
void Projection::compute_saturated_costs_tcf(
    const vector<int> &h_values,
    AbstractTransitionCostFunction &stcf) const {
    int num_operators = task_info.get_num_operators();
    vector<int> &sd_costs = stcf.get_sd_costs();
    // track state-independent costs as well to allow fast subtraction
    vector<bool> &si = stcf.get_si();
    vector<int> &si_costs = stcf.get_si_costs();
    fill(sd_costs.begin(), sd_costs.end(), -INF);
    fill(si.begin(), si.end(), true);
    fill(si_costs.begin(), si_costs.end(), -INF);
    for_each_transition(
        [&](const Transition &transition) {
            int source_id = transition.source_id;
            int target_id = transition.target_id;
            assert(utils::in_bounds(source_id, h_values));
            assert(utils::in_bounds(target_id, h_values));
            int source_h = h_values[source_id];
            int target_h = h_values[target_id];
            if (source_h == INF || target_h == INF ||
                source_h == -INF || target_h == -INF) {
                // saturated cost of this transition remains -INF
                return;
            }
            const int op_id = transition.op_id;
            const int needed = source_h - target_h;
            // stcf deviates from socf:
            if (si[op_id] &&
                needed != si_costs[op_id] &&
                si_costs[op_id] != -INF) {
                si[op_id] = false;
            }
            sd_costs[transition.transition_id] = needed;
            si_costs[op_id] = max(needed, si_costs[op_id]);
        }
    );
    for (int op_id = 0; op_id < num_operators; ++op_id) {
        if (operator_induces_self_loop(op_id)) {
            // stcf deviates from socf:
            // We require si_costs[op_id] != -INF because only in this specific case
            // we have to iterate over all transitions with label op_id
            if (si_costs[op_id] != 0 && si_costs[op_id] != -INF) {
                si[op_id] = false;
            }
            /* To prevent negative cost cycles we ensure that all operators
               inducing self-loops have non-negative costs. */
            si_costs[op_id] = max(0, si_costs[op_id]);
        }
    }
}

// ____________________________________________________________________________
vector<int> Projection::get_split_variables() const {
    return pattern;
}

// ____________________________________________________________________________
BDD Projection::make_state_bdd(int state_id) const {
    return bdd_builder.make_bdd(compute_state(state_id));
}

// ____________________________________________________________________________
BDD Projection::make_transition_bdd_and_cache(const Transition &transition) const {
    if (transition_bdd_cache.is_uninitialized()) {
        transition_bdd_cache.initialize(Abstraction::get_num_transitions());
    }
    if (!transition_bdd_cache.exists(transition.transition_id)) {
        transition_bdd_cache.insert(
            transition.transition_id,
            bdd_builder.make_bdd(compute_state(transition.source_id), transition.op_id));
    }
    return transition_bdd_cache.get(transition.transition_id);
}

// ____________________________________________________________________________
BDD Projection::make_transition_bdd(const Transition &transition) const {
    return bdd_builder.make_bdd(compute_state(transition.source_id), transition.op_id);
}

// ____________________________________________________________________________
int Projection::get_num_transitions(int op_id) const {
    return task_info.get_num_transitions_from_concrete_operator(pattern, op_id);
}

// ____________________________________________________________________________
bool Projection::operator_induces_self_loop(int op_id) const {
    return task_info.operator_induces_self_loop(pattern, op_id);
}

// ____________________________________________________________________________
bool Projection::operator_is_active(int op_id) const {
    return task_info.operator_is_active(pattern, op_id);
}

// ____________________________________________________________________________
void Projection::print_statistics() const {
    cout << "pattern: " << pattern << "\n"
         << "num_states: " << num_states << "\n"
         << "num_transitions: " << num_transitions << "\n";
}

}