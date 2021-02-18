#include "optimal_transition_cost_partitioning_heuristic.h"

#include "abstraction.h"
#include "bdd_builder.h"
#include "abstraction_function.h"
#include "utils.h"
#include "task_info.h"
#include "task_info.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../utils/collections.h"
#include "../utils/logging.h"
#include "../utils/timer.h"
#include "../utils/memory.h"

#include <cassert>
#include <cmath>
#include <vector>
#include <memory>

using namespace std;

namespace transition_cost_partitioning {
class Abstraction;

// ____________________________________________________________________________
OptimalTransitionCostPartitioningHeuristic::OptimalTransitionCostPartitioningHeuristic(
    options::Options &opts)
    : Heuristic(opts),
    _lp_solver(lp::LPSolverType(opts.get_enum("lpsolver"))),
    _allow_negative_costs(opts.get<bool>("allow_negative_costs")),
    _found_initial_h_value(false) {
    utils::Timer timer;

    shared_ptr<AbstractTask> task = opts.get<shared_ptr<AbstractTask>>("transform");

    TaskProxy task_proxy(*task);
    TaskInfo task_info(task_proxy);
    BddBuilder bdd_builder(task_info);

    // 1. Generate cartesian abstractions
    vector<unique_ptr<Abstraction>> abstractions = generate_transition_cost_partitioning_abstractions(
        task,
        task_info,
        bdd_builder,
        opts.get_list<shared_ptr<AbstractionGenerator>>("abstraction_generators"));

    // 2. Initialize dead ends and transition bdds
    const vector<int> &ocf = task_info.get_operator_costs();
 
    _h_values.reserve(abstractions.size());
    for (const unique_ptr<Abstraction> &abstraction : abstractions) {
        // precompute h values for dead end detection. 
        _h_values.emplace_back(abstraction->compute_goal_distances_ocf(ocf));
    }

    // 3. Generate LP
    generate_lp(bdd_builder, abstractions, task_info);

    for (auto &abstraction : abstractions) {
        _abstraction_functions.push_back(abstraction->extract_abstraction_function());
    }

    _lp_solver.print_statistics();

    // Cache indices for the last evaluated state to speed up adapting the LP.
    State initial_state = task_proxy.get_initial_state();
    _current_abstract_state_vars.resize(_abstraction_functions.size());
    for (size_t i = 0; i < _abstraction_functions.size(); ++i) {
        int init_id = _abstraction_functions[i]->get_abstract_state_id(initial_state);
        _current_abstract_state_vars[i] = _distance_variables[i][init_id];
    }
    // free memory
    release_memory();
}

// ____________________________________________________________________________
void OptimalTransitionCostPartitioningHeuristic::print_statistics() {

}

// ____________________________________________________________________________
int OptimalTransitionCostPartitioningHeuristic::compute_heuristic(const GlobalState &global_state) {
    State concrete_state = convert_global_state(global_state);
    // Set upper bound for distance of current abstract states to 0 and for all other
    // abstract states to infinity.
    for (int id = 0; id < static_cast<int>(_abstraction_functions.size()); ++id) {
        int new_state_id = _abstraction_functions[id]->get_abstract_state_id(concrete_state);
        if (new_state_id == -1 || _h_values[id][new_state_id] == INF) {
            return DEAD_END;
        }

        int old_state_var = _current_abstract_state_vars[id];
        _lp_solver.set_variable_upper_bound(old_state_var, _lp_solver.get_infinity());
        if (_allow_negative_costs) {
            _lp_solver.set_variable_lower_bound(old_state_var, -_lp_solver.get_infinity());
        }

        int new_state_var = _distance_variables[id][new_state_id];
        _lp_solver.set_variable_upper_bound(new_state_var, 0);
        if (_allow_negative_costs) {
            _lp_solver.set_variable_lower_bound(new_state_var, 0);
        }
        _current_abstract_state_vars[id] = new_state_var;
    }

    _lp_solver.solve();
    if (!_lp_solver.has_optimal_solution()) {
        if (!_found_initial_h_value) {
            utils::exit_with(utils::ExitCode::SEARCH_OUT_OF_MEMORY);
        }
        return DEAD_END;
    }
    _found_initial_h_value = true;

    double h_val = _lp_solver.get_objective_value();
    double epsilon = 0.01;
    return static_cast<int>(ceil(h_val - epsilon));
}

// ____________________________________________________________________________
void OptimalTransitionCostPartitioningHeuristic::generate_lp(
  const BddBuilder &bdd_builder,
  const vector<unique_ptr<Abstraction>> &abstractions,
  TaskInfo &task_info) {    
    vector<lp::LPVariable> lp_variables;
    vector<lp::LPConstraint> lp_constraints;
    for (size_t abstraction_id = 0; abstraction_id < abstractions.size(); ++abstraction_id) {
        cout << "Add abstraction " << abstraction_id + 1 << " of " << abstractions.size()
             << " to LP." << endl;
        const Abstraction &abstraction = *abstractions[abstraction_id];

        add_abstraction_variables(abstraction, abstraction_id, lp_variables);
        add_abstraction_constraints(abstraction, abstraction_id, lp_constraints);
    }
    cout << "Add transition cost variable and constraints to LP." << endl;
    add_context_cost_constraints(bdd_builder, abstractions, task_info, lp_constraints);
    _lp_solver.load_problem(lp::LPObjectiveSense::MAXIMIZE, lp_variables, lp_constraints);
}

// ____________________________________________________________________________
void OptimalTransitionCostPartitioningHeuristic::add_abstraction_variables(
  const Abstraction &abstraction,
  size_t abstraction_id,
  std::vector<lp::LPVariable> &lp_variables) {
    // sanity checks
    assert(_abstraction_variables.size() == abstraction_id);
    assert(_distance_variables.size() == abstraction_id);
    assert(_transition_cost_variables.size() == abstraction_id);
    // bounds
    const double upper_bound = _lp_solver.get_infinity();
    const double default_lower_bound = _allow_negative_costs ? -_lp_solver.get_infinity() : 0.;

    /*
      Add shortest goal distance variable to objective function.
    */
    _abstraction_variables.push_back(lp_variables.size());
    lp_variables.emplace_back(default_lower_bound, upper_bound, 1.);

    const vector<bool> &reachability = abstraction.get_reachability_from_init();

    /*
      Add a distance variable for each abstract reachable/non-dead-end state.
    */
    int num_states = abstraction.get_num_states();
    _distance_variables.emplace_back(num_states, UNDEFINED);
    for (int state_id = 0; state_id < num_states; ++state_id) {
        if (!reachability[state_id])
            continue;

        _distance_variables[abstraction_id][state_id] = lp_variables.size();
        lp_variables.emplace_back(default_lower_bound, upper_bound, 0.);
    }

    /*
      Add a cost variable for each abstract transition.
    */
    _transition_cost_variables.emplace_back(abstraction.get_num_transitions(), UNDEFINED);
    abstraction.for_each_transition(
        [&](const Transition &transition) {
            if (!reachability[transition.source_id] || 
                !reachability[transition.target_id])
                return;

            _transition_cost_variables[abstraction_id][transition.transition_id] = lp_variables.size();
            lp_variables.emplace_back(default_lower_bound, upper_bound, 0.);
        }
    );
}


// ____________________________________________________________________________
void OptimalTransitionCostPartitioningHeuristic::add_abstraction_constraints(
  const Abstraction &abstraction,
  size_t abstraction_id,
  std::vector<lp::LPConstraint> &lp_constraints) {
    /*
      Add distance distance constraints for all transitions t = <s,l,s'> in abstraction A
      0 <= distance[A][s] + cost[A][t] - distance[A][s'] <= \infty
    */
    const vector<bool> &reachability = abstraction.get_reachability_from_init();
    abstraction.for_each_transition(
        [&](const Transition &transition) {
            if (!reachability[transition.source_id] || 
                !reachability[transition.target_id])
                return;

            int from_col = _distance_variables[abstraction_id][transition.source_id];
            int to_col = _distance_variables[abstraction_id][transition.target_id];
            int cost_col = _transition_cost_variables[abstraction_id][transition.transition_id];
            assert(from_col != UNDEFINED);
            assert(to_col != UNDEFINED);
            assert(cost_col != UNDEFINED);
            lp::LPConstraint constraint(0., _lp_solver.get_infinity());
            constraint.insert(from_col, 1);
            constraint.insert(cost_col, 1);
            constraint.insert(to_col, -1);
            lp_constraints.push_back(move(constraint));
        }
    );

    /*
      For each abstract goal state s' in abstraction A add constraint
      heuristic[A] <= distance[A][s'] which equals
      0 <= distance[A][s'] - heuristic[A] <= \infty
    */
    const int abstraction_col = _abstraction_variables[abstraction_id];
    const unordered_set<int> &goal_states = abstraction.get_goal_states();
    for (int goal_state_id : goal_states) {
        if (!reachability[goal_state_id])
            continue;

        int goal_col = _distance_variables[abstraction_id][goal_state_id];
        assert(goal_col != UNDEFINED);
        lp::LPConstraint constraint(0., _lp_solver.get_infinity());
        constraint.insert(goal_col, 1);
        constraint.insert(abstraction_col, -1);
        lp_constraints.push_back(move(constraint));
    }
}


// ____________________________________________________________________________
void OptimalTransitionCostPartitioningHeuristic::add_context_cost_constraints(
  const BddBuilder &bdd_builder,
  const vector<unique_ptr<Abstraction>> &abstractions,
  TaskInfo &task_info,
  std::vector<lp::LPConstraint> &lp_constraints) {

    const int num_operators = task_info.get_num_operators();
    const vector<int> costs = task_info.get_operator_costs();

    const vector<vector<BDD>> state_bdds = bdd_builder.build_state_bdds_by_abstraction(abstractions);
    const vector<vector<BDD>> transition_bdds = bdd_builder.build_transition_bdds_by_abstraction(abstractions);

    const double default_lower_bound = _allow_negative_costs ? -_lp_solver.get_infinity() : 0.;

    // we generate contexts per operator because this generates the coarsest contexts for each operator.
    for (int op_id = 0; op_id < num_operators; ++op_id) {
        lp::LPConstraint trivial_constraint(default_lower_bound, costs[op_id]);
        // the context is the set of all states where op is applicable.
        const BDD &trivial_context = bdd_builder.get_precondition_bdd(op_id);
        generate_contexts_recursively(
            bdd_builder, 
            state_bdds,
            transition_bdds,
            move(trivial_constraint),
            abstractions,
            lp_constraints,
            trivial_context,            
            op_id, 
            0);
    }
}


// ____________________________________________________________________________
void OptimalTransitionCostPartitioningHeuristic::generate_contexts_recursively(
  const BddBuilder &bdd_builder,
    const vector<vector<BDD>> state_bdds,
    const vector<vector<BDD>> transition_bdds,
    lp::LPConstraint &&current_constraint,
    const vector<unique_ptr<Abstraction>> &abstractions,
    std::vector<lp::LPConstraint> &lp_constraints,
    const BDD cur_context,
    const int cur_op_id,
    int cur_abs_id) {
    // Base case: no abstraction for further context refinement available
    if (cur_abs_id == (int)abstractions.size()) {
        if (!current_constraint.get_variables().empty()) {
            lp_constraints.push_back(move(current_constraint));
        }
        return;
    }

    // Inductive case:

    // set of states that are reachable and non-dead-end and induce a loop.
    BDD looping_states = bdd_builder.make_zero();
    // set of states that are unreachable or has transitions/loops to dead-end states.
    // we call them infinity states because we insert a -infty term by setting the upper bound to infinity.
    BDD infinity_states = bdd_builder.make_zero();

    const Abstraction &abstraction = *abstractions[cur_abs_id];
    const vector<bool> &reachability = abstraction.get_reachability_from_init();
    for (int source_id = 0; source_id < abstraction.get_num_states(); ++source_id) {         
        const BDD &state_bdd = state_bdds[cur_abs_id][source_id];
        const BDD state_intersection = state_bdd * cur_context;
        if (state_intersection == bdd_builder.make_zero()) {
            // operator is not applicable in the abstract state => no loop and no transition
            continue;
        }
        // The current state is either a dead-end or unreachable state.
        // No matter for transition(s) or loop, add -infinity term.
        if (!reachability[source_id]) {
            infinity_states += state_intersection;
            continue;
        }

        // operator is applicable in the abstract state. (1) transition (2) transitions or (3) loop        
        // (1) transition (2) transitions
        bool has_transition = false;
        abstraction.for_each_transition(
            [&](const Transition &transition) {
                if ((source_id != transition.source_id) ||
                    (cur_op_id != transition.op_id)) {
                    // Transition does not have cur_op_id label and does not start from source_id.
                    return;
                }
                // There exists at least one transition with cur_op_id. Hence, there can not be any loop.
                has_transition = true;
                const BDD &transition_bdd = transition_bdds[cur_abs_id][transition.transition_id];
                const BDD transition_intersection = transition_bdd * state_intersection;
                // The transition preimage lies outside the current context.
                if (transition_intersection == bdd_builder.make_zero()) {
                    return;
                }

                // The target of this transition is a dead-end
                if (!reachability[transition.target_id]) {
                    infinity_states += transition_intersection;
                    return;   
                }
                
                // General transition handling (as abstract transition cost term)
                lp::LPConstraint next_constraint(current_constraint);
                const int transition_cost_variable = _transition_cost_variables[cur_abs_id][transition.transition_id];
                assert(transition_cost_variable != UNDEFINED);
                next_constraint.insert(transition_cost_variable, 1);
                generate_contexts_recursively(
                    bdd_builder, 
                    state_bdds,
                    transition_bdds,
                    move(next_constraint),
                    abstractions,
                    lp_constraints,
                    transition_intersection, 
                    cur_op_id, 
                    cur_abs_id + 1);
            }
        );
        // (3) loop          
        if (!has_transition) {
            looping_states += state_intersection;
        }
    }
    if (looping_states != bdd_builder.make_zero()) {
        lp::LPConstraint next_constraint(current_constraint);
        // simply forward the call
        generate_contexts_recursively(
            bdd_builder, 
            state_bdds,
            transition_bdds,
            move(next_constraint),
            abstractions,
            lp_constraints,
            looping_states, 
            cur_op_id, 
            cur_abs_id + 1);
    }

    if (infinity_states != bdd_builder.make_zero()) {
        lp::LPConstraint next_constraint(current_constraint);
        // add termin -infinity by setting upper bound to infinity
        next_constraint.set_upper_bound(_lp_solver.get_infinity());
        generate_contexts_recursively(
            bdd_builder, 
            state_bdds,
            transition_bdds,
            move(next_constraint),
            abstractions,
            lp_constraints,
            infinity_states, 
            cur_op_id, 
            cur_abs_id + 1);
    }
}


// ____________________________________________________________________________
void OptimalTransitionCostPartitioningHeuristic::release_memory() {
    utils::release_vector_memory(_abstraction_variables);
    utils::release_vector_memory(_transition_cost_variables);
}

// ____________________________________________________________________________
static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Optimal transition cost partitioning heuristic",
        "");

    lp::add_lp_solver_option_to_parser(parser);

    Heuristic::add_options_to_parser(parser);

    parser.add_list_option<shared_ptr<AbstractionGenerator>>(
        "abstraction_generators",
        "available generators are cartesian() and projections()",
        "[projections(hillclimbing(max_time=60, random_seed=0)),"
        " projections(systematic(2)), cartesian()]");
    
    parser.add_option<bool>(
        "allow_negative_costs",
        "use general instead of non-negative cost partitioning",
        "true");

    Options opts = parser.parse();
    if (parser.help_mode())
        return nullptr;

    if (parser.dry_run())
        return nullptr;

    return make_shared<OptimalTransitionCostPartitioningHeuristic>(opts);
}

// ____________________________________________________________________________
static Plugin<Evaluator> _plugin("optimal_transition_cost_partitioning", _parse);
}
