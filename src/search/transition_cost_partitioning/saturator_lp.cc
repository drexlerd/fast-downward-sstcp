#include "saturator_lp.h"

#include "abstract_transition_cost_function.h"
#include "cost_function_state_dependent.h"
#include "saturator.h"
#include "abstraction.h"
#include "utils.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../lp/lp_solver.h"
#include "../utils/logging.h"
#include "../utils/memory.h"

#include <cmath>
#include <unordered_set>

using namespace std;

namespace transition_cost_partitioning {

/*
  To avoid having to use real-valued numbers outside of the LP solver, we
  multiply all operator costs of the input task by 1000. This keeps the
  rounding error when converting from doubles to ints below 0.001. For the
  conversion, we use the round() function which returns the nearest integral
  value.

  Note that we cannot round the doubles up or down unconditionally: rounding up
  unconditionally could yield saturated cost functions that exceed the original
  cost function or make the heuristic inadmissible. Rounding down
  unconditionally preserves admissibility, but it often needlessly decreases
  heuristic values and turns very small negative cost values to -1, i.e., it
  introduces negative costs instead of 0 costs, which is undesirable.
*/ 
static int convert_to_int(double d) {
    d = round(d);
    // https://stackoverflow.com/a/30424410/1277298
    // we explicitely use our INF here, because this are our chosen upper and lower bounds.
    if (!(d > -INF && d < INF)) {
        cout << d << endl;
        ABORT("Overflow while converting double to int.");
    }
    return d;
}

static vector<lp::LPVariable> get_variables(
    const Abstraction &abstraction,
    const vector<int> &costs,
    const vector<int> &goal_distances, 
    const vector<bool> &reachability,
    bool use_general_costs,
    int state_id,
    int state_h_value,
    double lp_infty,
    vector<int> &distance_variables,
    vector<int> &operator_cost_variables) {
    int num_states = abstraction.get_num_states();
    int num_operators = costs.size();

    vector<lp::LPVariable> lp_variables;
    lp_variables.reserve(num_states + num_operators);

    // Add LP variables for abstract goal distances.
    unordered_set<int> goal_states(
        abstraction.get_goal_states().begin(), abstraction.get_goal_states().end());
    for (int source_id = 0; source_id < num_states; ++source_id) {
        // Check if the state is irrelevant for the LP
        if (goal_distances[source_id] == INF ||
            goal_distances[source_id] == -INF ||
            !reachability[source_id]) {
            continue;
        }

        double lower = -lp_infty;
        double upper = goal_states.count(source_id) ? 0. : lp_infty;
        /*
          For the abstract state s with goal distance h for which we saturate:
            h[s] = h which equals h <= h[s] <= h
        */
        if (source_id == state_id) {
            lower = state_h_value;
            upper = state_h_value;
        }
        distance_variables[source_id] = lp_variables.size();
        lp_variables.emplace_back(lower, upper, 0.);
    }

    // Add LP variables for the cost function.
    // we iterate transitions instead of operators to create a lp variables only if there is a relevant transition.
    abstraction.for_each_transition(
        [&](const Transition &transition) 
        {
            // Check if the transition is irrelevant for the LP
            if (goal_distances[transition.source_id] == INF ||
                goal_distances[transition.source_id] == -INF ||
                goal_distances[transition.target_id] == INF ||
                goal_distances[transition.target_id] == -INF ||
                !reachability[transition.source_id] ||
                costs[transition.op_id] == INF ||
                costs[transition.op_id] == -INF) {
                return;
            }
            
            // operator cost variable was already created.
            if (operator_cost_variables[transition.op_id] != UNDEFINED) {
                return;
            }

            // create operator cost variable.
            bool operator_loops = abstraction.operator_induces_self_loop(transition.op_id);
            double lower = (!use_general_costs || operator_loops) ? 0. : -lp_infty;
            double upper = costs[transition.op_id];
            operator_cost_variables[transition.op_id] = lp_variables.size();
            lp_variables.emplace_back(lower, upper, 0.);
        }
    );
    return lp_variables;
}

static vector<lp::LPConstraint> get_constraints(
    const Abstraction &abstraction, 
    const vector<int> &costs,
    const vector<int> &goal_distances, 
    const vector<bool> &reachable_from_state,
    double lp_infty, 
    vector<lp::LPVariable> &lp_variables,
    const vector<int> &distance_variables,
    const vector<int> &operator_cost_variables) {
    /*
      For <s, o, s'> in abstract transitions
        h[s] <= c[o] + h[s'] which equals
        0 <= c[o] + h[s'] - h[s] <= \infty
    */
    vector<lp::LPConstraint> lp_constraints;
    abstraction.for_each_transition(
        [&](const Transition &transition) {
            // Check if the transition is irrelevant for the LP
            if (goal_distances[transition.source_id] == INF ||
                goal_distances[transition.source_id] == -INF ||
                goal_distances[transition.target_id] == INF ||
                goal_distances[transition.target_id] == -INF ||
                !reachable_from_state[transition.source_id] ||
                costs[transition.op_id] == INF ||
                costs[transition.op_id] == -INF) {
                return;
            }
            // The transition starts and ends at a state with finite heuristic values            
            int from_col = distance_variables[transition.source_id];
            int op_col = operator_cost_variables[transition.op_id];
            int to_col = distance_variables[transition.target_id];
            assert(from_col != UNDEFINED);
            assert(op_col != UNDEFINED);
            assert(to_col != UNDEFINED);
            lp::LPConstraint constraint(0., lp_infty);
            constraint.insert(op_col, 1);
            constraint.insert(to_col, 1);
            constraint.insert(from_col, -1);
            lp_constraints.push_back(move(constraint));
            lp_variables[op_col].objective_coefficient = 1.;
        });
    lp_constraints.shrink_to_fit();
    return lp_constraints;
}


static vector<lp::LPVariable> get_variables_transition(
    const Abstraction &abstraction,
    const vector<int> &tcf,
    const vector<int> &goal_distances, 
    const vector<bool> &reachability,
    bool use_general_costs,
    int state_id,
    int state_h_value,
    double lp_infty,
    ObjectiveType objective_type,
    vector<int> &distance_variables,
    vector<int> &operator_cost_variables, 
    vector<int> &transition_cost_variables) {
    int num_states = abstraction.get_num_states();
    int num_transitions = tcf.size();

    vector<lp::LPVariable> lp_variables;
    lp_variables.reserve(num_states + num_transitions);

    // Add LP variables for abstract goal distances.
    unordered_set<int> goal_states(
        abstraction.get_goal_states().begin(), abstraction.get_goal_states().end());
    for (int source_id = 0; source_id < num_states; ++source_id) {
        // Check if the state is irrelevant for the LP
        if (goal_distances[source_id] == INF ||
            goal_distances[source_id] == -INF ||
            !reachability[source_id]) {
            continue;
        }

        double lower = -lp_infty;
        double upper = goal_states.count(source_id) ? 0. : lp_infty;
        /*
          For the abstract state s with goal distance h for which we saturate:
            h[s] = h which equals h <= h[s] <= h
        */
        if (source_id == state_id) {
            lower = state_h_value;
            upper = state_h_value;
        } 
        distance_variables[source_id] = lp_variables.size();
        lp_variables.emplace_back(lower, upper, 0.);
    }

    // Add LP variables for the cost function.
    abstraction.for_each_transition(
        [&](const Transition &transition) {
            int op_id = transition.op_id;
            int transition_id = transition.transition_id;
            int source_id = transition.source_id;
            int target_id = transition.target_id;
            // Check if the transition is irrelevant for the LP
            if (goal_distances[source_id] == INF ||
                goal_distances[source_id] == -INF ||
                goal_distances[target_id] == INF ||
                goal_distances[target_id] == -INF ||
                !reachability[source_id] ||
                tcf[transition_id] == INF ||
                tcf[transition_id] == -INF) {
                return;
            }
            /*
              For every transition t add variable
                0/-\infty <= cost[t] <= tcf[t]
            */
            transition_cost_variables[transition_id] = lp_variables.size();
            double lower = !use_general_costs ? 0. : -lp_infty;
            double upper = tcf[transition_id];
            lp_variables.emplace_back(lower, upper, 0.);

            if (objective_type == ObjectiveType::OPERATORS) {
                if (operator_cost_variables[op_id] != UNDEFINED) {
                    return;
                }
                /*
                  For every operator o add variable
                    0/-\infty <= cost[o] <= \infty
                */
                operator_cost_variables[op_id] = lp_variables.size();
                bool operator_loops = abstraction.operator_induces_self_loop(op_id);
                double lower = (!use_general_costs || operator_loops) ? 0. : -lp_infty;
                double upper = lp_infty;
                lp_variables.emplace_back(lower, upper, 0.);
            }
        }
    );
    lp_variables.shrink_to_fit();
    return lp_variables;
}

static vector<lp::LPConstraint> get_constraints_transition(
    const Abstraction &abstraction, 
    const vector<int> &tcf,
    const vector<int> &goal_distances, 
    const vector<bool> &reachable_from_state,
    double lp_infty,
    ObjectiveType objective_type,  
    vector<lp::LPVariable> &lp_variables,
    const vector<int> &distance_variables,
    const vector<int> &operator_cost_variables,
    const vector<int> &transition_cost_variables) {
    vector<lp::LPConstraint> lp_constraints;
    abstraction.for_each_transition(
        [&](const Transition &transition) {
            int op_id = transition.op_id;
            int transition_id = transition.transition_id;
            int source_id = transition.source_id;
            int target_id = transition.target_id;
            // Check if the transition is irrelevant for the LP
            if (goal_distances[source_id] == INF ||
                goal_distances[source_id] == -INF ||
                goal_distances[target_id] == INF ||
                goal_distances[target_id] == -INF ||
                !reachable_from_state[source_id] ||
                !reachable_from_state[target_id] ||
                tcf[transition_id] == INF ||
                tcf[transition_id] == -INF) {
                return;
            }

            /*
              For t = <s, o, s'> in abstract transitions
                * h[s] <= cost[t] + h[s'] which equals
                0 <= cost[t] + h[s'] - h[s] <= \infty
            */
            assert(reachable_from_state[target_id]);
            int from_col = distance_variables[source_id];
            int transition_col = transition_cost_variables[transition_id];
            int to_col = distance_variables[target_id];
            assert(from_col != UNDEFINED);
            assert(transition_col != UNDEFINED);
            assert(to_col != UNDEFINED);
            lp::LPConstraint constraint(0., lp_infty);
            constraint.insert(transition_col, 1);
            constraint.insert(to_col, 1);
            constraint.insert(from_col, -1);
            lp_constraints.push_back(move(constraint));
            if (objective_type == ObjectiveType::TRANSITIONS) {
                lp_variables[transition_col].objective_coefficient = 1.0;
            } else if (objective_type == ObjectiveType::OPERATORS) {
                /*
                  For t = <s, o, s'> in abstract transitions
                    cost[t] <= cost[o] which equals
                    0 <= cost[o] - cost[t] <= \infty
                */
                int op_col = operator_cost_variables[op_id];
                assert(op_col != UNDEFINED);
                lp::LPConstraint constraint2(0., lp_infty);
                constraint2.insert(op_col, 1);
                constraint2.insert(transition_col, -1);
                lp_constraints.push_back(move(constraint2));
                lp_variables[op_col].objective_coefficient = 1.0;
            }
        }
    );

    lp_constraints.shrink_to_fit();
    return lp_constraints;
}



// ____________________________________________________________________________
SaturatorLP::SaturatorLP(const options::Options &opts)
    : Saturator(opts),
    objective_type(static_cast<ObjectiveType>(opts.get_enum("objective_type"))),
    spd(opts.get<bool>("spd")),
    saturate_negative_infinity(opts.get<bool>("saturate_negative_infinity")) {
    // If cost saturation sees this saturator, it will initialize the abstraction accordingly.
    reachable = Reachable::FROM_STATE;
}

// ____________________________________________________________________________
SaturatorResultOcf SaturatorLP::saturate_ocf(
    const Abstraction &abstraction,
    const vector<int> &ocf,
    vector<int> &&h_values,
    int state_id) const {
    bool verbose = false;
    int num_states = abstraction.get_num_states();
    int num_operators = ocf.size();
    int h = h_values[state_id];


    /*
      Initialize reachability from the given state.
    */
    vector<bool> reachability = abstraction.compute_reachability_from_state_ocf(
        compute_reachability_cost_function(ocf), state_id);

    if (verbose) {
        cout << "Solve LP for" << endl;
        cout << "h-values: " << h_values << endl;
        cout << "reachable: " << reachability << endl;
        cout << "ocf: " << ocf << endl;
        cout << "state: " << state_id << endl;
        cout << "goal states: " << abstraction.get_goal_states() << endl;
    }

    /*
      When we saturate for state s, we preserve h(s) in all abstractions. That
      means h(s) can only be INF if there are no paths from s to a goal, not
      because there are only paths from s to a goal that use transitions with
      cost INF. Therefore, if h(s) = INF, there are no paths from s to a goal
      and consequently any cost function preserves h(s)=INF, including the cost
      function that assigns -INF to all operators. Although we could set
      h(s')=0 for goal states s' without solvable successors, for simplicity we
      use h(s')=-INF for all states s'!=s.
    */
    if (h == INF) {
        vector<int> saturated_costs(num_operators, use_general_costs ? -INF : 0);
        vector<int> h_values(num_states, use_general_costs ? -INF : 0);
        h_values[state_id] = INF;
        ABORT("initial h is infinity");
        return SaturatorResultOcf(move(saturated_costs), move(h_values), false);
    } else if (h == -INF) {
        ABORT("Saturating for h(s) = -INF not supported.");
    }

    /* 
      Build and solve the lp.
      To speed up the computation, we only construct variables for transitions between
      states with finite heuristic values.
    */
    lp::LPSolver lp_solver(lp::LPSolverType::SOPLEX);
    double lp_infty = lp_solver.get_infinity();
    vector<int> distance_variables(num_states, UNDEFINED);
    vector<int> operator_cost_variables(ocf.size(), UNDEFINED);
    vector<lp::LPVariable> lp_variables = get_variables(
        abstraction, ocf, h_values, reachability, use_general_costs, state_id, h, lp_infty, distance_variables, operator_cost_variables);
    vector<lp::LPConstraint> lp_constraints = get_constraints(
        abstraction, ocf, h_values, reachability, lp_infty, lp_variables, distance_variables, operator_cost_variables);
    lp_solver.load_problem(lp::LPObjectiveSense::MINIMIZE, lp_variables, lp_constraints);
    lp_solver.solve();
    vector<double> solution = lp_solver.extract_solution();

    /*
      Extract the h values.
    */
    vector<int> new_h_values(num_states);
    for (int source_id = 0; source_id < num_states; ++source_id) {
        int distance_col = distance_variables[source_id];
        // case 1: infinite h_value
        if (distance_col == UNDEFINED) {
            if (!reachability[source_id] && h_values[source_id] != INF) {
                new_h_values[source_id] = -INF;
            } else {
                new_h_values[source_id] = INF;
            }
        // case 2: finite h_value
        } else {
            // Attention: new_h_values values might be negative. 
            // This happens in states that are close to a goal and have many transitions.
            new_h_values[source_id] = convert_to_int(solution[distance_col]);
        }
    }

    /* 
      Extract the saturated costs.
    */
    vector<int> socf = abstraction.compute_saturated_costs_ocf(new_h_values);

    if (verbose) {
        cout << "solution: " << solution << endl;
        cout << "new h-values: " << new_h_values << endl;
        cout << "socf: " << socf << endl;
    }

    return get_saturator_result_ocf(
        abstraction, ocf, move(socf), move(new_h_values), saturate_negative_infinity);
}

// ____________________________________________________________________________
SaturatorResultTcf SaturatorLP::saturate_tcf(
    const Abstraction &abstraction,
    AbstractTransitionCostFunction &&tcf,
    const CostFunctionStateDependent &sdac,
    vector<int> &&h_values,
    int state_id) const {

    /*
      Apply the spd saturator to retrieve the transition cost function.
      It already considers newly detected useless operators.
    */
    if (h_values.empty()) {
        if (spd) {     
            h_values = abstraction.compute_goal_distances_tcf(sdac, tcf);
        } else {
            sdac.determine_remaining_abstract_transition_cost_function(abstraction, tcf);
            h_values = abstraction.compute_goal_distances_tcf(tcf);
        }
    }

    bool verbose = false;
    int num_states = abstraction.get_num_states();
    int num_transitions = abstraction.get_num_transitions();
    int num_operators = abstraction.get_num_operators();
    int state_h = h_values[state_id];

    /*
      Initialize reachability from state.
    */
    vector<bool> reachability = abstraction.compute_reachability_from_state_tcf(tcf, state_id);

    if (verbose) {
        cout << "Solve LP for" << endl;
        cout << "h-values: " << h_values << endl;
        cout << "reachable: " << reachability << endl;
        cout << "state: " << state_id << endl;
        cout << "goal states: " << abstraction.get_goal_states() << endl;
    }

    /*
      When we saturate for state s, we preserve h(s) in all abstractions. That
      means h(s) can only be INF if there are no paths from s to a goal, not
      because there are only paths from s to a goal that use transitions with
      cost INF. Therefore, if h(s) = INF, there are no paths from s to a goal
      and consequently any cost function preserves h(s)=INF, including the cost
      function that assigns -INF to all operators. Although we could set
      h(s')=0 for goal states s' without solvable successors, for simplicity we
      use h(s')=-INF for all states s'!=s.
    */
    if (state_h == INF) {
        vector<int> h_values(num_states, use_general_costs ? -INF : 0);
        h_values[state_id] = INF;
        return SaturatorResultTcf(
            move(tcf), 
            move(h_values), 
            saturate_negative_infinity);
    } else if (state_h == -INF) {
        ABORT("Saturating for h(s) = -INF not supported.");
    }

    /* 
      Build and solve the lp.
      To speed up the computation, we only construct variables for transitions between
      states with finite heuristic values.
    */
    lp::LPSolver lp_solver(lp::LPSolverType::SOPLEX);
    double lp_infty = lp_solver.get_infinity();
    // store variables related to parts of the transition system.
    vector<int> distance_variables(num_states, UNDEFINED);
    vector<int> operator_cost_variables(num_operators, UNDEFINED);
    vector<int> transition_cost_variables(num_transitions, UNDEFINED);
    vector<lp::LPVariable> lp_variables = get_variables_transition(
        abstraction, tcf.get_sd_costs(), h_values, reachability, use_general_costs, state_id, state_h, lp_infty, objective_type, distance_variables, operator_cost_variables, transition_cost_variables);
    vector<lp::LPConstraint> lp_constraints = get_constraints_transition(
        abstraction, tcf.get_sd_costs(), h_values, reachability, lp_infty, objective_type, lp_variables, distance_variables, operator_cost_variables, transition_cost_variables);
    lp_solver.load_problem(lp::LPObjectiveSense::MINIMIZE, lp_variables, lp_constraints);
    lp_solver.solve();
    vector<double> solution = lp_solver.extract_solution();

    /*
      Extract the h values.
    */
    vector<int> new_h_values(abstraction.get_num_states());
    for (int source_id = 0; source_id < num_states; ++source_id) {
        int distance_col = distance_variables[source_id];
        // case 1: infinite h_value
        if (distance_col == UNDEFINED) {
            if (h_values[source_id] == INF) {
                new_h_values[source_id] = INF;
            } else {
                new_h_values[source_id] = -INF;
            }
        // case 2: finite h_value
        } else {
            // Attention: new_h_values values might be negative. 
            // This happens in states that are close to a goal and have many transitions.
            new_h_values[source_id] = convert_to_int(solution[distance_col]);
            // Ensure that the new_h_value is indeed finite.
            if (new_h_values[source_id] == -INF || new_h_values[source_id] == INF) {
                cout << new_h_values[source_id] << endl;
                ABORT("LP Transition Saturator: new_heuristic_value is infinity in the finite case. Reachability function incorrect.");
            }
        }
    }

    /*
      Instead of extracting the saturated costs from the lp,
      we recompute the saturated transition cost function
      with the given new_h_values.
    */
    abstraction.compute_saturated_costs_tcf(h_values, tcf);

    /*
      Return results including the reachability function, 
      if we want to saturate -INF for these states.
    */
    return SaturatorResultTcf(move(tcf), move(new_h_values), saturate_negative_infinity);
}


// ____________________________________________________________________________
static shared_ptr<Saturator> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "LP-based saturator",
        "");
    parser.add_option<bool>(
        "spd",
        "use shortest path discovery saturator",
        "true");
    parser.add_option<bool>(
        "saturate_negative_infinity",
        "saturate -INF for uninteresting states",
        "false");
    vector<std::string> objective_types;
    objective_types.push_back("TRANSITIONS");
    objective_types.push_back("OPERATORS");
    parser.add_enum_option(
        "objective_type",
        objective_types,
        "the objective",
        "TRANSITIONS");
    add_saturator_options(parser);

    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;

    return make_shared<SaturatorLP>(opts);
}

// ____________________________________________________________________________
static Plugin<Saturator> _plugin("cp_lp", _parse);

}
