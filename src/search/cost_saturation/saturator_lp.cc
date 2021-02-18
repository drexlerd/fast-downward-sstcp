#include "saturator_lp.h"

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

namespace cost_saturation {

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
    bool use_general_costs,
    int state_id,
    int state_h_value,
    double lp_infty) {
    int num_states = abstraction.get_num_states();
    int num_operators = costs.size();

    vector<lp::LPVariable> lp_variables;
    lp_variables.reserve(num_states + num_operators);

    // Add LP variables for abstract goal distances.
    unordered_set<int> goal_states(
        abstraction.get_goal_states().begin(), abstraction.get_goal_states().end());
    for (int i = 0; i < num_states; ++i) {
        double lower = -lp_infty;
        double upper = goal_states.count(i) ? 0. : lp_infty;
        /*
          For the abstract state s with goal distance h for which we saturate:
            h[s] = h which equals h <= h[s] <= h
        */
        if (i == state_id) {
            lower = state_h_value;
            upper = state_h_value;
        }
        lp_variables.emplace_back(lower, upper, 0.);
    }

    // Add LP variables for the cost function.
    for (int op_id = 0; op_id < num_operators; ++op_id) {
        bool operator_loops = abstraction.operator_induces_self_loop(op_id);
        double lower = (!use_general_costs || operator_loops) ? 0. : -lp_infty;
        double upper = costs[op_id];
        double coefficient = operator_loops ? 1. : 0.;

        // Ignore operators with infinite costs.
        if (costs[op_id] == -INF || costs[op_id] == INF) {
            coefficient = 0.;
        }

        lp_variables.emplace_back(lower, upper, coefficient);
    }

    return lp_variables;
}

static vector<lp::LPConstraint> get_constraints(
    const Abstraction &abstraction, const vector<int> &costs,
    const vector<int> &goal_distances, const vector<bool> &reachable_from_state,
    double lp_infty, vector<lp::LPVariable> &lp_variables) {
    /*
      For <s, o, s'> in abstract transitions
        h[s] <= c[o] + h[s'] which equals
        0 <= c[o] + h[s'] - h[s] <= \infty
    */
    vector<lp::LPConstraint> lp_constraints;
    abstraction.for_each_transition(
        [&](const Transition &transition) {
            if (goal_distances[transition.src] == INF ||
                goal_distances[transition.src] == -INF ||
                goal_distances[transition.target] == INF ||
                goal_distances[transition.target] == -INF ||
                !reachable_from_state[transition.src] ||
                costs[transition.op] == INF ||
                costs[transition.op] == -INF) {
                return;
            }
            assert(reachable_from_state[transition.target]);
            int from_col = transition.src;
            int op_col = abstraction.get_num_states() + transition.op;
            int to_col = transition.target;
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


SaturatorLP::SaturatorLP(const options::Options &opts)
    : Saturator(opts) {
}

SaturatorResult SaturatorLP::saturate(
    const Abstraction &abstraction,
    int,
    const vector<int> &costs,
    vector<int> &&h_values,
    int state_id) const {
    bool verbose = false;
    int num_states = abstraction.get_num_states();
    int num_operators = costs.size();
    int h = h_values[state_id];
    vector<bool> reachable_from_state = abstraction.compute_reachability_from_state(
        compute_reachability_cost_function(costs), state_id);

    if (verbose) {
        cout << "Solve LP for" << endl;
        cout << "h-values: " << h_values << endl;
        cout << "reachable: " << reachable_from_state << endl;
        cout << "costs: " << costs << endl;
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
        return SaturatorResult(move(saturated_costs), move(h_values));
    } else if (h == -INF) {
        ABORT("Saturating for h(s) = -INF not supported.");
    }

    lp::LPSolver lp_solver(lp::LPSolverType::SOPLEX);
    double lp_infty = lp_solver.get_infinity();
    vector<lp::LPVariable> lp_variables = get_variables(
        abstraction, costs, use_general_costs, state_id, h, lp_infty);
    vector<lp::LPConstraint> lp_constraints = get_constraints(
        abstraction, costs, h_values, reachable_from_state, lp_infty, lp_variables);
    lp_solver.load_problem(lp::LPObjectiveSense::MINIMIZE, lp_variables, lp_constraints);
    lp_solver.solve();
    vector<double> solution = lp_solver.extract_solution();

    vector<int> saturated_costs(num_operators, use_general_costs ? -INF : 0);
    for (int i = 0; i < num_operators; ++i) {
        if (lp_variables[num_states + i].objective_coefficient != 0.) {
            saturated_costs[i] = convert_to_int(solution[num_states + i]);
        }
    }

    vector<int> new_h_values;
    new_h_values.reserve(num_states);
    for (int i = 0; i < num_states; ++i) {
        int h;
        if (h_values[i] == INF) {
            h = INF;
        } else if (!reachable_from_state[i]) {
            h = -INF;
        } else {
            h = convert_to_int(solution[i]);
        }
        new_h_values.push_back(h);
    }

    if (verbose) {
        cout << "solution: " << solution << endl;
        cout << "new h-values: " << new_h_values << endl;
        cout << "saturated costs: " << saturated_costs << endl;
    }

    return get_saturator_result(
        abstraction, costs, move(saturated_costs), move(new_h_values), h);
}

static shared_ptr<Saturator> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "LP-based saturator",
        "");
    add_saturator_options(parser);

    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;

    return make_shared<SaturatorLP>(opts);
}

static Plugin<Saturator> _plugin("lp", _parse);
}
