#include "explicit_abstraction_cegar.h"

#include "bdd_builder.h"
#include "task_info.h"
#include "utils.h"

#include "../utils/logging.h"

namespace transition_cost_partitioning {

// ____________________________________________________________________________
ExplicitAbstractionCegar::ExplicitAbstractionCegar(
    const TaskInfo &task_info,
    const BddBuilder &bdd_builder,
    unique_ptr<AbstractionFunction> abstraction_function,
    int num_transitions,
    int num_states,
    int init_state_id,
    unordered_set<int> &&goal_states,
    vector<vector<Successor>> &&backward_graph,
    vector<int> &&num_transitions_by_operator,
    vector<bool> &&has_outgoing,
    vector<bool> &&has_loop,
    unique_ptr<cegar::SplitTree> cegar_split_tree) :
    ExplicitAbstraction(
        task_info,
        bdd_builder,
        move(abstraction_function),
        num_transitions,
        num_states,
        init_state_id,
        move(goal_states),
        move(backward_graph),
        move(num_transitions_by_operator),
        move(has_outgoing),
        move(has_loop)),
        split_tree(SplitTree(task_info, bdd_builder, move(cegar_split_tree))) {
}

// ____________________________________________________________________________
vector<int> ExplicitAbstractionCegar::get_split_variables() const {
    return split_variables;
}

// ____________________________________________________________________________
BDD ExplicitAbstractionCegar::make_state_bdd(int state_id) const {
    return split_tree.make_bdd(state_id);
}

// ____________________________________________________________________________
BDD ExplicitAbstractionCegar::make_transition_bdd_and_cache(const Transition &transition) const {
    if (transition_bdd_cache.is_uninitialized()) {
        transition_bdd_cache.initialize(Abstraction::get_num_transitions());
    }
    if (!transition_bdd_cache.exists(transition.transition_id)) {
        transition_bdd_cache.insert(
            transition.transition_id,
            split_tree.regress(transition));
    }
    return transition_bdd_cache.get(transition.transition_id);
}

// ____________________________________________________________________________
BDD ExplicitAbstractionCegar::make_transition_bdd(const Transition &transition) const {
    return split_tree.regress(transition);
}

}
