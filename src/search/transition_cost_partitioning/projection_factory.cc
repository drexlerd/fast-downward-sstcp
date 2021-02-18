#include "projection_factory.h"

#include "projection.h"
#include "utils.h"
#include "task_info.h"
#include "bdd_builder.h"

#include "../task_proxy.h"
#include "../pdbs/match_tree.h"
#include "../task_utils/task_properties.h"
#include "../utils/collections.h"
#include "../utils/logging.h"
#include "../utils/math.h"
#include "../utils/memory.h"

#include <cassert>
#include <unordered_map>
#include <map>

using namespace std;

namespace transition_cost_partitioning {

// ____________________________________________________________________________
static vector<int> get_abstract_preconditions(
    const vector<FactPair> &prev_pairs,
    const vector<FactPair> &pre_pairs,
    const vector<size_t> &hash_multipliers) {
    vector<int> abstract_preconditions(hash_multipliers.size(), -1);
    for (const FactPair &fact : prev_pairs) {
        int pattern_index = fact.var;
        abstract_preconditions[pattern_index] = fact.value;
    }
    for (const FactPair &fact : pre_pairs) {
        int pattern_index = fact.var;
        abstract_preconditions[pattern_index] = fact.value;
    }
    return abstract_preconditions;
}

// ____________________________________________________________________________
static int compute_hash_effect(
    const vector<FactPair> &preconditions,
    const vector<FactPair> &effects,
    const vector<size_t> &hash_multipliers,
    bool forward) {
    int hash_effect = 0;
    assert(preconditions.size() == effects.size());
    for (size_t i = 0; i < preconditions.size(); ++i) {
        int var = preconditions[i].var;
        assert(var == effects[i].var);
        int old_val = preconditions[i].value;
        int new_val = effects[i].value;
        assert(old_val != -1);
        if (!forward) {
            swap(old_val, new_val);
        }
        int effect = (new_val - old_val) * hash_multipliers[var];
        hash_effect += effect;
    }
    return hash_effect;
}

// ____________________________________________________________________________
static void multiply_out(
    int pos, int cost, int op_id,
    const vector<size_t> &hash_multipliers,
    const vector<int> &pattern,
    vector<FactPair> &prev_pairs,
    vector<FactPair> &pre_pairs,
    vector<FactPair> &eff_pairs,
    const vector<FactPair> &effects_without_pre,
    const VariablesProxy &variables,
    const OperatorCallback &callback) {
    if (pos == static_cast<int>(effects_without_pre.size())) {
        // All effects without precondition have been checked.
        // Only abstract operators that induce state changes are constructed via callback.
        if (!eff_pairs.empty()) {
            callback(prev_pairs, pre_pairs, eff_pairs, cost, hash_multipliers, op_id);
        }
    } else {
        // For each possible value for the current variable, build an
        // abstract operator.
        int var_id = effects_without_pre[pos].var;
        int eff = effects_without_pre[pos].value;
        VariableProxy var = variables[pattern[var_id]];
        for (int i = 0; i < var.get_domain_size(); ++i) {
            if (i != eff) {
                pre_pairs.emplace_back(var_id, i);
                eff_pairs.emplace_back(var_id, eff);
            } else {
                prev_pairs.emplace_back(var_id, i);
            }
            multiply_out(pos + 1, cost, op_id, hash_multipliers, pattern, prev_pairs, pre_pairs, eff_pairs,
                         effects_without_pre, variables, callback);
            if (i != eff) {
                pre_pairs.pop_back();
                eff_pairs.pop_back();
            } else {
                prev_pairs.pop_back();
            }
        }
    }
}

// ____________________________________________________________________________
static void build_abstract_operators(
    const OperatorProxy &op,
    int cost,
    const vector<size_t> &hash_multipliers,
    const vector<int> &pattern,
    const vector<int> &variable_to_pattern_index,
    const VariablesProxy &variables,
    const OperatorCallback &callback) {
    // All variable value pairs that are a prevail condition
    vector<FactPair> prev_pairs;
    // All variable value pairs that are a precondition (value != -1)
    vector<FactPair> pre_pairs;
    // All variable value pairs that are an effect
    vector<FactPair> eff_pairs;
    // All variable value pairs that are a precondition (value = -1)
    vector<FactPair> effects_without_pre;

    size_t num_vars = variables.size();
    vector<bool> has_precond_and_effect_on_var(num_vars, false);
    vector<bool> has_precondition_on_var(num_vars, false);

    for (FactProxy pre : op.get_preconditions())
        has_precondition_on_var[pre.get_variable().get_id()] = true;

    for (EffectProxy eff : op.get_effects()) {
        int var_id = eff.get_fact().get_variable().get_id();
        int pattern_var_id = variable_to_pattern_index[var_id];
        int val = eff.get_fact().get_value();
        if (pattern_var_id != -1) {
            if (has_precondition_on_var[var_id]) {
                has_precond_and_effect_on_var[var_id] = true;
                eff_pairs.emplace_back(pattern_var_id, val);
            } else {
                effects_without_pre.emplace_back(pattern_var_id, val);
            }
        }
    }

    for (FactProxy pre : op.get_preconditions()) {
        int var_id = pre.get_variable().get_id();
        int pattern_var_id = variable_to_pattern_index[var_id];
        int val = pre.get_value();
        if (pattern_var_id != -1) { // variable occurs in pattern
            if (has_precond_and_effect_on_var[var_id]) {
                pre_pairs.emplace_back(pattern_var_id, val);
            } else {
                prev_pairs.emplace_back(pattern_var_id, val);
            }
        }
    }

    multiply_out(0, cost, op.get_id(), hash_multipliers, pattern, prev_pairs, pre_pairs, eff_pairs,
                 effects_without_pre, variables, callback);
}

// ____________________________________________________________________________
static bool is_consistent(
    size_t state_index,
    const vector<size_t> &hash_multipliers,
    const vector<int> &pattern_domain_sizes,
    const vector<FactPair> &abstract_facts) {
    for (const FactPair &abstract_goal : abstract_facts) {
        int pattern_var_id = abstract_goal.var;
        int temp = state_index / hash_multipliers[pattern_var_id];
        int val = temp % pattern_domain_sizes[pattern_var_id];
        if (val != abstract_goal.value) {
            return false;
        }
    }
    return true;
}

// ____________________________________________________________________________
static unordered_set<int> compute_goal_states(
    int num_states,
    const TaskInfo &task_info,
    const vector<size_t> &hash_multipliers,
    const vector<int> &pattern_domain_sizes,
    const vector<int> &variable_to_pattern_index) {
    vector<FactPair> abstract_goals;
    for (FactPair goal : task_info.get_goals()) {
        if (variable_to_pattern_index[goal.var] != -1) {
            abstract_goals.emplace_back(
                variable_to_pattern_index[goal.var], goal.value);
        }
    }
    unordered_set<int> goal_states;
    for (int state_index = 0; state_index < num_states; ++state_index) {
        if (is_consistent(state_index, hash_multipliers, pattern_domain_sizes, abstract_goals)) {
            goal_states.insert(state_index);
        }
    }
    return goal_states;
}


// ____________________________________________________________________________
unique_ptr<Abstraction> ProjectionFactory::convert_abstraction(
    const TaskProxy &task_proxy,        
    const pdbs::Pattern &pattern,
    const TaskInfo &task_info,
    const BddBuilder &bdd_builder) {
    
    assert(utils::is_sorted_unique(pattern));
    assert(pattern.size() > 0);
    
    vector<size_t> hash_multipliers;
    hash_multipliers.reserve(pattern.size());
    int num_states = 1;
    for (int pattern_var_id : pattern) {
        hash_multipliers.push_back(num_states);
        VariableProxy var = task_proxy.get_variables()[pattern_var_id];
        if (utils::is_product_within_limit(num_states, var.get_domain_size(),
                                           numeric_limits<int>::max())) {
            num_states *= var.get_domain_size();
        } else {
            cerr << "Given pattern is too large! (Overflow occured): " << endl;
            cerr << pattern << endl;
            utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
        }
    }
    assert(hash_multipliers.size() > 0);

    unique_ptr<AbstractionFunction> abstraction_function = unique_ptr<AbstractionFunction>(new ProjectionFunction(pattern, hash_multipliers));

    VariablesProxy variables = task_proxy.get_variables();
    vector<int> variable_to_pattern_index(variables.size(), -1);
    for (size_t i = 0; i < pattern.size(); ++i) {
        variable_to_pattern_index[pattern[i]] = i;
    }
    vector<int> pattern_domain_sizes;
    pattern_domain_sizes.reserve(pattern.size());
    for (int pattern_var : pattern) {
        pattern_domain_sizes.push_back(variables[pattern_var].get_domain_size());
    }

    unique_ptr<pdbs::MatchTree> match_tree_forward = utils::make_unique_ptr<pdbs::MatchTree>(
        task_proxy, pattern, hash_multipliers);
    unique_ptr<pdbs::MatchTree> match_tree_backward = utils::make_unique_ptr<pdbs::MatchTree>(
        task_proxy, pattern, hash_multipliers);

    assert(hash_multipliers.size() > 0);

    // Compute abstract forward and backward operators.
    int total_num_transitions = 0;
    vector<AbstractForwardOperator> abstract_forward_operators;
    vector<AbstractBackwardOperator> abstract_backward_operators;
    vector<int> transition_id_offset;
    OperatorsProxy operators = task_proxy.get_operators();
    for (OperatorProxy op : operators) {
        //int cur_num_transitions = 0;
        //abstract_operator_id_offset[op.get_id()] = abstract_forward_operators.size();
        build_abstract_operators(
            op, -1, hash_multipliers, pattern, variable_to_pattern_index, variables,
            [&](
                const vector<FactPair> &prevail,
                const vector<FactPair> &preconditions,
                const vector<FactPair> &effects,
                int,
                const vector<size_t> &hash_multipliers,
                int concrete_operator_id) {                
                int abs_op_id = abstract_backward_operators.size();
                abstract_backward_operators.emplace_back(
                    concrete_operator_id,
                    compute_hash_effect(preconditions, effects, hash_multipliers, false));
                vector<FactPair> regression_preconditions = prevail;
                regression_preconditions.insert(
                    regression_preconditions.end(), effects.begin(), effects.end());
                sort(regression_preconditions.begin(), regression_preconditions.end());
                match_tree_backward->insert(abs_op_id, regression_preconditions);

                vector<int> abstract_preconditions = get_abstract_preconditions(
                    prevail, preconditions, hash_multipliers);
                int precondition_hash = 0;
                for (size_t pos = 0; pos < hash_multipliers.size(); ++pos) {
                    int pre_val = abstract_preconditions[pos];
                    if (pre_val != -1) {
                        precondition_hash += hash_multipliers[pos] * pre_val;
                    }
                }

                vector<FactPair> prevails_and_preconditions = prevail;
                prevails_and_preconditions.insert(
                    prevails_and_preconditions.end(),
                    preconditions.begin(),
                    preconditions.end());
                sort(prevails_and_preconditions.begin(), prevails_and_preconditions.end());

                abstract_forward_operators.emplace_back(
                    precondition_hash,
                    compute_hash_effect(
                        preconditions, effects, hash_multipliers, true));

                match_tree_forward->insert(abs_op_id, prevails_and_preconditions);

                // compute number of state-changing transitions that this abstract operator induces.
                transition_id_offset.emplace_back(total_num_transitions);
                total_num_transitions += task_info.get_num_transitions_from_abstract_operator(pattern, concrete_operator_id);
            });
    }
    abstract_forward_operators.shrink_to_fit();
    abstract_backward_operators.shrink_to_fit();

    // Initialize goal states.
    unordered_set<int> goal_states = compute_goal_states(num_states, task_info, hash_multipliers, pattern_domain_sizes, variable_to_pattern_index);
    // Initialize initial state.
    int init_state_id = abstraction_function->get_abstract_state_id(task_info.get_initial_state());

    return unique_ptr<Projection>(
        new Projection(
            task_info,
            bdd_builder,
            move(abstraction_function),
            total_num_transitions,
            num_states,
            init_state_id,
            move(goal_states),
            pattern,
            move(hash_multipliers),
            move(pattern_domain_sizes),
            move(abstract_forward_operators),
            move(match_tree_forward),
            move(abstract_backward_operators),
            move(match_tree_backward),
            move(transition_id_offset)));
}

}