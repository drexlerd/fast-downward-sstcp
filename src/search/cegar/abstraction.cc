#include "abstraction.h"

#include "abstract_state.h"
#include "refinement_hierarchy.h"
#include "split_tree.h"
#include "transition.h"
#include "transition_system.h"
#include "utils.h"

#include "../tasks/domain_abstracted_task.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"
#include "../utils/math.h"
#include "../utils/memory.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <unordered_map>

using namespace std;

namespace cegar {

Abstraction::Abstraction(const shared_ptr<AbstractTask> &task, bool debug)
    : task(task),
      concrete_initial_state(TaskProxy(*task).get_initial_state()),
      goal_facts(task_properties::get_fact_pairs(TaskProxy(*task).get_goals())),
      refinement_hierarchy(utils::make_unique_ptr<RefinementHierarchy>(task)),
      split_tree(utils::make_unique_ptr<SplitTree>(task)),
      transition_system(utils::make_unique_ptr<TransitionSystem>(TaskProxy(*task).get_operators())),
      debug(debug) {
    initialize_trivial_abstraction(get_domain_sizes(TaskProxy(*task)));
}

Abstraction::~Abstraction() {
}

const AbstractState &Abstraction::get_initial_state() const {
    return *states[init_id];
}

int Abstraction::get_num_states() const {
    return states.size();
}

const Goals &Abstraction::get_goals() const {
    return goals;
}

const AbstractState &Abstraction::get_state(int state_id) const {
    return *states[state_id];
}

const TransitionSystem &Abstraction::get_transition_system() const {
    return *transition_system;
}

unique_ptr<RefinementHierarchy> Abstraction::extract_refinement_hierarchy() {
    assert(refinement_hierarchy);
    return move(refinement_hierarchy);
}

// ____________________________________________________________________________
vector<Transitions> Abstraction::extract_outgoing() {
    return move(transition_system->extract_outgoing());
}

// ____________________________________________________________________________
vector<Transitions> Abstraction::extract_incoming() {
    return move(transition_system->extract_incoming());
}

// ____________________________________________________________________________
vector<Loops> Abstraction::extract_loops() {
    return move(transition_system->extract_loops());
}

// ____________________________________________________________________________
unique_ptr<SplitTree> Abstraction::extract_split_tree() {
    assert(split_tree);
    return move(split_tree);
}

// ____________________________________________________________________________
std::unique_ptr<TransitionSystem> Abstraction::extract_transition_system() {
    assert(transition_system);
    return move(transition_system);
}

// ____________________________________________________________________________
std::vector<int> Abstraction::extract_goal_states() {
    std::vector<int> goal_states;
    for (const int goal_state : goals) {
        goal_states.push_back(goal_state);
    }
    goals.clear();
    return move(goal_states);
}

// ____________________________________________________________________________
std::vector<CartesianSet> Abstraction::get_concrete_abstract_states() {
    std::vector<int> concrete_domain_sizes = get_concrete_domain_sizes();
    std::vector<CartesianSet> result;
    result.reserve(states.size());
    extra_tasks::DomainAbstractedTask* dat = dynamic_cast<extra_tasks::DomainAbstractedTask*>(task.get());
    if (dat) {
        for (const unique_ptr<AbstractState> &state : states) {
            CartesianSet concrete_state = CartesianSet(concrete_domain_sizes);
            CartesianSet &abstract_state = state->get_cartesian_set();
            for (size_t var = 0; var < concrete_domain_sizes.size(); ++var) {
                cegar::Bitset &abstract_values = abstract_state.get_bitset_ref_from_var(var);
                cegar::Bitset &concrete_values = concrete_state.get_bitset_ref_from_var(var);
                dat->get_concrete_values(var, abstract_values, concrete_values);
            }
            assert(concrete_state.validate());
            result.push_back(concrete_state);
        }
        return result;
    } 
    for (const unique_ptr<AbstractState> &state : states) {
        result.push_back(state->get_cartesian_set());
    }
    return result;
    
}

// ____________________________________________________________________________
int Abstraction::get_num_loops() {
    return transition_system->get_num_loops();
}

// ____________________________________________________________________________
int Abstraction::get_num_non_loops() {
    return transition_system->get_num_non_loops();
}

// ____________________________________________________________________________
const std::shared_ptr<AbstractTask> Abstraction::get_task() const {
    return task;
}

// ____________________________________________________________________________
void Abstraction::mark_all_states_as_goals() {
    goals.clear();
    for (auto &state : states) {
        goals.insert(state->get_id());
    }
}

// ____________________________________________________________________________
void Abstraction::initialize_trivial_abstraction(const vector<int> &domain_sizes) {
    unique_ptr<AbstractState> init_state =
        AbstractState::get_trivial_abstract_state(domain_sizes);

    init_id = init_state->get_id();
    goals.insert(init_state->get_id());
    states.push_back(move(init_state));
}

// ____________________________________________________________________________
pair<int, int> Abstraction::refine(
    const AbstractState &state, int var, const vector<int> &wanted) {
    if (debug)
        cout << "Refine " << state << " for " << var << "=" << wanted << endl;

    int v_id = state.get_id();
    // Reuse state ID from obsolete parent to obtain consecutive IDs.
    int v1_id = v_id;
    int v2_id = get_num_states();

    // Update refinement hierarchy.
    pair<NodeID, NodeID> node_ids = refinement_hierarchy->split(
        state.get_node_id(), var, wanted, v1_id, v2_id);

    pair<CartesianSet, CartesianSet> cartesian_sets =
        state.split_domain(var, wanted);

    // Note: here we construct the context split tree
    pair<NodeID, NodeID> context_split_node_ids = split_tree->split(
        state.get_split_node_id(), v1_id, v2_id, var, 
        cartesian_sets.first.get_bitset_from_var(var), 
        cartesian_sets.second.get_bitset_from_var(var));

    unique_ptr<AbstractState> v1 = utils::make_unique_ptr<AbstractState>(
        v1_id, node_ids.first, context_split_node_ids.first, move(cartesian_sets.first));
    unique_ptr<AbstractState> v2 = utils::make_unique_ptr<AbstractState>(
        v2_id, node_ids.second, context_split_node_ids.second, move(cartesian_sets.second));
    assert(state.includes(*v1));
    assert(state.includes(*v2));

    /*
      Due to the way we split the state into v1 and v2, v2 is never the new
      initial state and v1 is never a goal state.
    */
    if (state.get_id() == init_id) {
        if (v1->includes(concrete_initial_state)) {
            assert(!v2->includes(concrete_initial_state));
            init_id = v1_id;
        } else {
            assert(v2->includes(concrete_initial_state));
            init_id = v2_id;
        }
        if (debug) {
            cout << "New init state #" << init_id << ": " << get_state(init_id)
                 << endl;
        }
    }
    if (goals.count(v_id)) {
        goals.erase(v_id);
        if (v1->includes(goal_facts)) {
            goals.insert(v1_id);
        }
        if (v2->includes(goal_facts)) {
            goals.insert(v2_id);
        }
        if (debug) {
            cout << "Goal states: " << goals.size() << endl;
        }
    }

    transition_system->rewire(states, v_id, *v1, *v2, var);

    states[v1_id] = move(v1);
    assert(static_cast<int>(states.size()) == v2_id);
    states.push_back(move(v2));

    return {
               v1_id, v2_id
    };
}

void Abstraction::print_statistics() const {
    cout << "States: " << get_num_states() << endl;
    cout << "Goal states: " << goals.size() << endl;
    transition_system->print_statistics();
}
}
