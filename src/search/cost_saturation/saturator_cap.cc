#include "saturator_cap.h"

#include "abstraction.h"
#include "utils.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../utils/collections.h"
#include "../utils/logging.h"

using namespace std;

namespace cost_saturation {
static void cap_h_values(int max_value, vector<int> &h_values) {
    assert(max_value != -INF);
    for (int &h : h_values) {
        if (h != INF) {
            h = min(h, max_value);
        }
    }
}


SaturatorCap::SaturatorCap(const options::Options &opts)
    : Saturator(opts),
      reachable(static_cast<Reachable>(opts.get_enum("reachable"))),
      cap(opts.get<bool>("cap")) {
}

void SaturatorCap::initialize(
    const Abstractions &abstractions, const vector<int> &costs, const State &initial_state) {
    if (reachable == Reachable::FROM_INIT) {
        unreachable_from_init.reserve(abstractions.size());
        for (auto &abstraction : abstractions) {
            vector<bool> reachable_from_init = abstraction->compute_reachability_from_state(
                costs, abstraction->get_abstract_state_id(initial_state));
            vector<int> unreachable;
            for (int state_id = 0; state_id < abstraction->get_num_states(); ++state_id) {
                if (!reachable_from_init[state_id]) {
                    unreachable.push_back(state_id);
                }
            }
            unreachable.shrink_to_fit();
            unreachable_from_init.push_back(move(unreachable));
        }
    }
}

SaturatorResult SaturatorCap::saturate(
    const Abstraction &abstraction,
    int abstraction_id,
    const vector<int> &costs,
    vector<int> &&h_values,
    int state_id) const {
    int state_h = h_values[state_id];

    if (cap) {
        if (!all_of(costs.begin(), costs.end(), [](int c) {return c >= 0 || c == -INF;})) {
            ABORT("perim saturator only accepts costs >= 0 or -\\infty");
        }
        cap_h_values(state_h, h_values);
    }

    if (reachable == Reachable::FROM_INIT) {
        for (int unreachable_state : unreachable_from_init[abstraction_id]) {
            // Ensure we never reach a state that was determined to be unreachable.
            assert(unreachable_state != state_id);
            if (h_values[unreachable_state] != INF) {
                h_values[unreachable_state] = -INF;
            }
        }
    } else if (reachable == Reachable::FROM_STATE) {
        vector<bool> reachable_from_state = abstraction.compute_reachability_from_state(
            compute_reachability_cost_function(costs), state_id);
        assert(reachable_from_state[state_id]);
        for (int i = 0; i < abstraction.get_num_states(); ++i) {
            if (!reachable_from_state[i] && h_values[i] != INF) {
                h_values[i] = -INF;
            }
        }
    }

    vector<int> saturated_costs = abstraction.compute_saturated_costs(h_values);
    if (!use_general_costs) {
        if (!all_of(costs.begin(), costs.end(), [](int c) {return c >= 0;})) {
            ABORT("Cap Operator Saturator: Nonnegative saturator only accepts costs >= 0");
        }
        transform(saturated_costs.begin(), saturated_costs.end(), saturated_costs.begin(),
            [this](int c) {
                return max(0, c);
            });
    }

    return get_saturator_result(
        abstraction, costs, move(saturated_costs), move(h_values), state_h);
}

static void add_saturator_cap_options(OptionParser &parser) {
    vector<string> reachable_opts;
    reachable_opts.push_back("FROM_INIT");
    reachable_opts.push_back("FROM_STATE");
    reachable_opts.push_back("ALL");
    parser.add_enum_option(
        "reachable",
        reachable_opts,
        "only consider reachable states",
        "ALL");
    parser.add_option<bool>(
        "cap",
        "cap h values at the goal distance of the given state",
        "false");
}

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

static Plugin<Saturator> _plugin("all", _parse_all);
}
