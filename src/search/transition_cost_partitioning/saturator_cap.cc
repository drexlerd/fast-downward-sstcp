#include "saturator_cap.h"

#include "abstraction.h"
#include "utils.h"
#include "cost_function_state_dependent.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../utils/collections.h"
#include "../utils/logging.h"

using namespace std;

namespace transition_cost_partitioning {
// ____________________________________________________________________________
static void cap_h_values(int max_value, vector<int> &h_values) {
    assert(max_value != -INF);
    for (int &h : h_values) {
        if (h != INF) {
            h = min(h, max_value);
        }
    }
}

// ____________________________________________________________________________
SaturatorCap::SaturatorCap(const options::Options &opts)
    : Saturator(opts),
      cap(opts.get<bool>("cap")),
      spd(opts.get<bool>("spd")),
      saturate_negative_infinity(opts.get<bool>("saturate_negative_infinity")) {
    reachable = static_cast<Reachable>(opts.get_enum("reachable"));
}

// ____________________________________________________________________________
SaturatorResultOcf SaturatorCap::saturate_ocf(
    const Abstraction &abstraction,
    const vector<int> &ocf,
    vector<int> &&h_values,
    int state_id) const {

    /*
      perim saturator
    */
    int state_h = h_values[state_id];
    if (cap) {
        if (!all_of(ocf.begin(), ocf.end(), [](int c) { return c >= 0 || c == -INF; })) {
            ABORT("perim saturator only accepts costs >= 0 or -\\infty");
        }
        cap_h_values(state_h, h_values);
    }

    /*
      Update reachability if FROM_STATE is used.
    */
    if (reachable == Reachable::FROM_STATE) {
        vector<bool> reachability = abstraction.compute_reachability_from_state_ocf(
            compute_reachability_cost_function(ocf), state_id);
        for (int i = 0; i < abstraction.get_num_states(); ++i) {
            if (!reachability[i] && h_values[i] != INF) {
                h_values[i] = -INF;
            }
        }
    } else if (reachable == Reachable::FROM_INIT) {
        const vector<bool> &reachability = abstraction.get_reachability_from_init();
        for (int i = 0; i < abstraction.get_num_states(); ++i) {
            if (!reachability[i] && h_values[i] != INF) {
                h_values[i] = -INF;
            }
        }
    }

    vector<int> socf = abstraction.compute_saturated_costs_ocf(h_values);

    /*
      Note: we do not explicitly keep negative infinities here, in order to apply Dijkstra's algorithm
      on the nonnegative saturated operator cost function.
      But we keep heuristic estimates of negative infinities.
      Hence, applying a general saturator afterward, allows to refetch negative infinities again.
    */
    if (!use_general_costs) {
        if (!all_of(ocf.begin(), ocf.end(), [](int c) {return c >= 0;})) {
            ABORT("Cap Operator Saturator: Nonnegative saturator only accepts costs >= 0");
        }
        for (size_t op_id = 0; op_id < socf.size(); ++op_id) {
            int &saturated = socf[op_id];
            int remaining = ocf[op_id];
            assert(saturated <= remaining);
            saturated = max(0, saturated);
            if (saturated > remaining) {
                cout << saturated << " > " << remaining << endl;
                ABORT("Cap Operator Saturator: output has to dominate its input");
            }
        }
    }

    return get_saturator_result_ocf(
        abstraction, ocf, move(socf), move(h_values), saturate_negative_infinity);
}

// ____________________________________________________________________________
SaturatorResultTcf SaturatorCap::saturate_tcf(
    const Abstraction &abstraction,
    AbstractTransitionCostFunction &&tcf,
    const CostFunctionStateDependent &sdac,
    vector<int> &&h_values,
    int state_id) const {
    // useful for debugging.
    bool verbose = false;
    if (verbose) {
        // cout << h_values << endl;
    }

    /*
      Apply the spd saturator to retrieve the transition cost function.
    */
    bool is_first_saturator_in_chain = false;
    if (h_values.empty()) {
        is_first_saturator_in_chain = true;
        if (spd) {
            h_values = abstraction.compute_goal_distances_tcf(sdac, tcf);
        } else {
            sdac.determine_remaining_abstract_transition_cost_function(abstraction, tcf);
            h_values = abstraction.compute_goal_distances_tcf(tcf);
        }
    }

    /*
      Apply the perim saturator.
    */
    int state_h = h_values[state_id];
    if (cap) {
        if (!is_first_saturator_in_chain && !tcf.is_nonnegative()) {
            ABORT("Perim Transition Saturator: Only accepts costs >= 0");
        }
        cap_h_values(state_h, h_values);
    }

    /*
      Update heuristic values depending on reachability.
    */
    if (reachable == Reachable::FROM_INIT) {
        const vector<bool> &reachability = abstraction.get_reachability_from_init();
        assert(reachability[state_id]);
        for (int i = 0; i < abstraction.get_num_states(); ++i) {
            if (!reachability[i] && h_values[i] != INF) {
                h_values[i] = -INF;
            }
        }
    } else if (reachable == Reachable::FROM_STATE) {
        // Dynamically update the reachability.
        vector<bool> reachability = abstraction.compute_reachability_from_state_tcf(
            tcf, state_id);
        assert(reachability[state_id]);
        for (int i = 0; i < abstraction.get_num_states(); ++i) {
            if (!reachability[i] && h_values[i] != INF) {
                h_values[i] = -INF;
            }
        }
    }

    // Note: stcf(t) = -INF for transitions <s,l,s'> with h(s)=INF and h(s')<INF.
    // This is a problem only if we reevaluate the heuristic.
    // For simplicity we keep -INF because INF - x = INF, so it doesnt matter if we chose stcf(t)=-INF or stcf(t)=INF.
    abstraction.compute_saturated_costs_tcf(h_values, tcf);

    if (verbose) {
        cout << h_values << endl;
        cout << tcf.get_si() << endl;
        cout << endl;
    }

    /*
      Return results including the reachability function,
      if we want to saturate -INF for these states.
    */
    return SaturatorResultTcf(move(tcf), move(h_values), saturate_negative_infinity);
}


// ____________________________________________________________________________
static void add_saturator_cap_options(OptionParser &parser) {
    vector<string> reachable_opts;
    reachable_opts.push_back("ALL");
    reachable_opts.push_back("FROM_INIT");
    reachable_opts.push_back("FROM_STATE");
    parser.add_enum_option(
        "reachable",
        reachable_opts,
        "only consider reachable states",
        "ALL");
    parser.add_option<bool>(
        "spd",
        "use shortest path discovery saturator",
        "true");
    parser.add_option<bool>(
        "cap",
        "cap h values at the goal distance of the given state",
        "false");
    parser.add_option<bool>(
        "saturate_negative_infinity",
        "saturate -INF for uninteresting states",
        "false");
}

// ____________________________________________________________________________
static shared_ptr<Saturator> _parse_all(OptionParser &parser) {
    parser.document_synopsis(
        "Saturator all",
        "");
    add_saturator_cap_options(parser);
    add_saturator_options(parser);

    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;

    return make_shared<SaturatorCap>(opts);
}

// ____________________________________________________________________________
static Plugin<Saturator> _plugin("cp_all", _parse_all);

}
