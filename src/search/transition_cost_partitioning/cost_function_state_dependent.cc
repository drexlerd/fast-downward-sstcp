#include "cost_function_state_dependent.h"

#include "abstraction.h"
#include "bdd_builder.h"
#include "utils.h"
#include "task_info.h"
#include "saturator_cap.h"

#include "../utils/logging.h"

#include <set>
#include <utility>
#include <algorithm>

namespace transition_cost_partitioning {


// ____________________________________________________________________________
static void insert_cost_value(
    const BddBuilder &bdd_builder,
    int cost_value,
    const BDD &cost_value_bdd,
    unordered_map<int, BDD> &new_sd_costs) {
    // do not insert empty bdd.
    if (cost_value_bdd == bdd_builder.make_zero()) {
        return;
    }
    auto exists = new_sd_costs.find(cost_value);
    if (exists != new_sd_costs.end()) {
        exists->second += cost_value_bdd;
    } else {
        new_sd_costs.emplace(cost_value, cost_value_bdd);
    }
}

// ____________________________________________________________________________
static void insert_cost_value(
    const BddBuilder &bdd_builder,
    int cost_value,
    const BDD &cost_value_bdd,
    map<int, BDD> &new_sd_costs) {
    // do not insert empty bdd.
    if (cost_value_bdd == bdd_builder.make_zero()) {
        return;
    }
    auto exists = new_sd_costs.find(cost_value);
    if (exists != new_sd_costs.end()) {
        exists->second += cost_value_bdd;
    } else {
        new_sd_costs.emplace(cost_value, cost_value_bdd);
    }
}

// ____________________________________________________________________________
static void remove_states(
    const BddBuilder &bdd_builder,
    const BDD &context,
    map<int, BDD> &remaining_costs_values) {
    for (auto it = remaining_costs_values.begin(); it != remaining_costs_values.end(); /*no increment*/) {
        int cost_value = it->first;
        // remaining cost value INF will remain INF forever.
        if (cost_value == INF) {
            ++it;
            continue;
        }
        BDD &cost_value_bdd = it->second;
        if (bdd_builder.intersect(context, cost_value_bdd)) {
            // remove the states from cost value bdd
            cost_value_bdd *= !context;
            if (cost_value_bdd != bdd_builder.make_zero()) {
                ++it;
            } else {
                it = remaining_costs_values.erase(it);
            }
        } else {
            ++it;
        }
    }
}

// ____________________________________________________________________________
static void limit_buckets(
    map<int, BDD> &remaining_costs_values,
    int MAX_BUCKETS) {
    if (static_cast<int>(remaining_costs_values.size()) > MAX_BUCKETS) {
        int index = 0;
        map<int, BDD>::iterator bucket;
        for (auto it = remaining_costs_values.begin(); it != remaining_costs_values.end();) {
            if (index == MAX_BUCKETS - 1) {
                bucket = it;
                ++it;
            } else if (index > MAX_BUCKETS - 1) {
                bucket->second += it->second;
                it = remaining_costs_values.erase(it);
            } else {
                ++it;
            }
            ++index;
        }
    }
}

// ____________________________________________________________________________
CostFunctionStateDependent::CostFunctionStateDependent(
    const TaskInfo &task_info,
    const BddBuilder &bdd_builder,
    int max_buckets,
    bool diversify) :
    task_info(task_info),
    bdd_builder(bdd_builder),
    max_buckets(max_buckets),
    diversify(diversify),
    useless_operators(task_info.get_num_operators(), false),
    count_evaluations(0),
    count_subtractions(0) {
    reinitialize();
}

// ____________________________________________________________________________
bool CostFunctionStateDependent::verify_cost_function_state_space() const {
    for (int op_id = 0; op_id < (int)remaining_sd_costs.size(); ++op_id) {
        if (!verify_cost_function_state_space(op_id)) {
            return false;
        }
    }
    return true;
}

// ____________________________________________________________________________
bool CostFunctionStateDependent::verify_cost_function_state_space(int op_id) const {
    BDD result = bdd_builder.make_zero();
    for (const pair<const int, BDD> &cost_value : remaining_sd_costs[op_id]) {
        if (bdd_builder.intersect(result, cost_value.second)) {
            cout << "cost value: " << cost_value.first << endl;
            ABORT("Bucket error: A state is detected in multiple buckets");
            return false;
        } else if (cost_value.second == bdd_builder.make_zero()) {
            cout << "cost value: " << cost_value.first << endl;
            ABORT("Bucket error: The bucket is empty.");
            return false;
        }
        result += cost_value.second;
    }
    if (result != bdd_builder.make_one()) {
        cout << "op_id: " << op_id << endl;
        ABORT("Totality of cost function violated");
        return false;
    }
    return true;
}

// ____________________________________________________________________________
void CostFunctionStateDependent::reduce_operator_costs(
    int op_id, int saturated) {
    assert(saturated != 0 && saturated != INF);
    if (saturated == -INF) {
        remaining_sd_costs[op_id].clear();
        remaining_sd_costs[op_id].emplace(INF, bdd_builder.make_one());
        useless_operators[op_id] = true;
    } else {
        map<int, BDD> new_sd_costs;
        for (auto &p : remaining_sd_costs[op_id]) {
            int old_cost_value = p.first;
            if (old_cost_value == INF) {
                insert_cost_value(bdd_builder, INF, p.second, new_sd_costs);
            } else {
                // we only allow non negative values after subtraction,
                // because in primed saturators, we might get negative values
                // in deadend/unreachable states.
                int new_cost_value = max(0, left_subtraction(old_cost_value, saturated));
                insert_cost_value(bdd_builder, new_cost_value, p.second, new_sd_costs);
            }
        }
        remaining_sd_costs[op_id] = move(new_sd_costs);
    }
    assert(verify_cost_function_state_space(op_id));
}

// ____________________________________________________________________________
void CostFunctionStateDependent::reinitialize() {
    int num_operators = task_info.get_num_operators();
    remaining_sd_costs = vector<map<int, BDD>>(num_operators);
    for (int op_id = 0; op_id < num_operators; ++op_id) {
        if (useless_operators[op_id]) {
            remaining_sd_costs[op_id].emplace(INF, bdd_builder.make_one());
        } else {
            remaining_sd_costs[op_id].emplace(task_info.get_operator_cost(op_id), bdd_builder.make_one());
        }
    }
    assert(verify_cost_function_state_space());
}


// ____________________________________________________________________________
vector<int>
CostFunctionStateDependent::determine_remaining_costs_operator() const {
    vector<int> remaining_costs(task_info.get_num_operators(), 0);
    for (int op_id = 0; op_id < task_info.get_num_operators(); ++op_id) {
        remaining_costs[op_id] = determine_remaining_costs_operator(op_id);
    }
    return remaining_costs;
}

// ____________________________________________________________________________
int CostFunctionStateDependent::determine_remaining_costs_operator(int op_id) const {
    auto p = remaining_sd_costs[op_id].begin();
    assert(bdd_builder.is_applicable(p->second, op_id));
    assert(p->first >= 0);
    return p->first;
}

// ____________________________________________________________________________
vector<int> CostFunctionStateDependent::determine_remaining_costs_transition(
    const Abstraction &abstraction) const {
    vector<int> remaining_costs_transition(abstraction.get_num_transitions(), -1);
    abstraction.for_each_transition(
        [&](const Transition &transition) {
            remaining_costs_transition[transition.transition_id] = determine_remaining_costs_transition(
                abstraction, transition);
        }
    );
    return remaining_costs_transition;
}

// ____________________________________________________________________________
void CostFunctionStateDependent::determine_remaining_abstract_transition_cost_function(
    const Abstraction &abstraction,
    AbstractTransitionCostFunction &tcf) const {
    vector<int> &sd_costs = tcf.get_sd_costs();
    abstraction.for_each_transition(
        [&](const Transition &transition) {
            sd_costs[transition.transition_id] = determine_remaining_costs_transition(
                abstraction, transition);
        }
    );
}

// ____________________________________________________________________________
int CostFunctionStateDependent::determine_remaining_costs_transition(
    const Abstraction &abstraction, const Transition &transition) const {
    /* Exploit structural information to speed up computation. */
    if (abstraction.is_goal_state(transition.source_id)) {
        return 0;
    }

    ++count_evaluations;

    const BDD transition_bdd = (diversify)
        ? abstraction.make_transition_bdd_and_cache(transition)
        : abstraction.make_transition_bdd(transition);

    // find minimal cost value, where the intersection is non empty
    int op_id = transition.op_id;
    int cost = -1;
    for (const auto &p : remaining_sd_costs[op_id]) {
        const BDD &active_state_set = p.second;
        cost = p.first;
        if (bdd_builder.intersect(active_state_set, transition_bdd)) {
            break;
        }
    }

    assert(remaining_sd_costs[op_id].size() > 0);
    assert(cost >= 0);
    return cost;
}

// ____________________________________________________________________________
int CostFunctionStateDependent::determine_remaining_costs_transition(
    const Abstraction &abstraction, const Transition &transition, int required) const {
    /* Exploit structural information to speed up computation. */
    if (abstraction.is_goal_state(transition.source_id)) {
        return 0;
    }

    ++count_evaluations;

    const BDD transition_bdd = (diversify)
        ? abstraction.make_transition_bdd_and_cache(transition)
        : abstraction.make_transition_bdd(transition);

    // find minimal cost value, where the intersection is non empty
    int op_id = transition.op_id;
    int cost = -1;
    for (const auto &p : remaining_sd_costs[op_id]) {
        const BDD &active_state_set = p.second;
        cost = p.first;
        if (cost >= required) {
            break;
        }
        if (bdd_builder.intersect(active_state_set, transition_bdd)) {
            break;
        }
    }

    assert(remaining_sd_costs[op_id].size() > 0);
    assert(cost >= 0);
    return cost;
}

// ____________________________________________________________________________
void CostFunctionStateDependent::reduce_operator_costs(
    const vector<int> &socf) {
    for (int op_id = 0; op_id < task_info.get_num_operators(); ++op_id) {
        int saturated = socf[op_id];
        if (saturated == 0 || saturated == INF)
            continue;
        reduce_operator_costs(op_id, saturated);
    }
}

// ____________________________________________________________________________
void CostFunctionStateDependent::reduce_operator_costs(
    AbstractTransitionCostFunction &tcf) {
    const vector<bool> &si = tcf.get_si();
    const vector<int> &si_costs = tcf.get_si_costs();
    for (int op_id = 0; op_id < task_info.get_num_operators(); ++op_id) {
        if (!si[op_id])
            continue;
        int saturated = si_costs[op_id];
        if (saturated == 0 || saturated == INF)
            continue;
        reduce_operator_costs(op_id, saturated);
    }
}

// ____________________________________________________________________________
void CostFunctionStateDependent::reduce_transition_costs_finite(
    const Abstraction &abstraction,
    AbstractTransitionCostFunction &tcf) {
    int num_operators = task_info.get_num_operators();
    const vector<int> &sd_costs = tcf.get_sd_costs();
    const vector<bool> &si = tcf.get_si();
    /* 1. Compute saturated transition cost function. */
    vector<unordered_map<int, BDD>> stcf_dds(num_operators);
    abstraction.for_each_transition(si,
        [&](const Transition &transition) {
            int saturated = sd_costs[transition.transition_id];
            // we handle negative infinities separately.
            if (saturated == -INF || saturated == 0 || saturated == INF)
                return;

            ++count_subtractions;

            const BDD transition_bdd = (diversify)
                ? abstraction.make_transition_bdd_and_cache(transition)
                : abstraction.make_transition_bdd(transition);

            insert_cost_value(bdd_builder, saturated, transition_bdd, stcf_dds[transition.op_id]);
        }
    );

    /* 2. Subtract saturated transition cost function.
       TODO: A lot of strategies can be used to subtract the saturated cost function.
    */
    vector<map<int, BDD>> new_sd_costs(num_operators);
    for (int op_id = 0; op_id < num_operators; ++op_id) {
        if (si[op_id]) {
            // steal buckets from old remaining cost function.
            new_sd_costs[op_id] = move(remaining_sd_costs[op_id]);
        } else {
            // move states of changed cost values
            for (auto &remaining : remaining_sd_costs[op_id]) {
                for (auto &saturated : stcf_dds[op_id]) { // swap iterations?
                    if (bdd_builder.intersect(remaining.second, saturated.second)) {
                        insert_cost_value(
                            bdd_builder,
                            left_subtraction(remaining.first, saturated.first),
                            remaining.second * saturated.second,
                            new_sd_costs[op_id]);
                        remaining.second *= !saturated.second;
                    }
                }
            }
            // move states of unchanged cost values
            for (auto &remaining : remaining_sd_costs[op_id]) {
                insert_cost_value(
                    bdd_builder,
                    remaining.first,
                    remaining.second,
                    new_sd_costs[op_id]);
            }
            // Limit number of buckets
            limit_buckets(new_sd_costs[op_id], max_buckets);
        }
    }
    remaining_sd_costs = move(new_sd_costs);
    assert(verify_cost_function_state_space());
}

// ____________________________________________________________________________
void CostFunctionStateDependent::reduce_transition_costs_negative_infinity(
    const Abstraction &abstraction,
    const vector<int> &h_values) {
    vector<BDD> reachability_bdds = bdd_builder.make_negative_infinity_bdds(abstraction, h_values, useless_operators);
    for (int op_id = 0; op_id < task_info.get_num_operators(); ++op_id) {
        if (useless_operators[op_id])
            continue;

        const BDD &reachability_bdd = reachability_bdds[op_id];

        remove_states(bdd_builder, reachability_bdd, remaining_sd_costs[op_id]);
        insert_cost_value(bdd_builder, INF, reachability_bdd, remaining_sd_costs[op_id]);

        assert(verify_cost_function_state_space(op_id));
    }
}

// ____________________________________________________________________________
void CostFunctionStateDependent::print_statistics() const {
    bdd_builder.print_statistics();
    cout << "Num evaluations: " << count_evaluations << "\n";
    cout << "Num subtractions: " << count_subtractions << "\n";
    cout << "Num useless operators: " << count(useless_operators.begin(), useless_operators.end(), true) << "\n";
}


}
