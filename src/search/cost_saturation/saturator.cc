#include "saturator.h"

#include "abstraction.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace cost_saturation {
Saturator::Saturator(const options::Options &opts)
    : use_general_costs(opts.get<bool>("use_general_costs")),
      recompute_h_values(opts.get<bool>("recompute_h_values")) {
}

SaturatorResult Saturator::get_saturator_result(
    const Abstraction &abstraction,
    const vector<int> &costs,
    vector<int> &&saturated_costs,
    vector<int> &&h_values,
    int state_h) const {
    /*
      Apply preprocessing step 1: if ocf(o) = INF then socf(o) = INF
    */    
    for (size_t i = 0; i < costs.size(); ++i) {
        if (costs[i] == INF) {
            saturated_costs[i] = INF;
        }
    }
    /*
      Recomputing goal distances might make sense because of slack in transitions 
      that are outside the subset of states used in the operator saturator.

      Note: we have to speed up dijkstra computation based on current knowledge about heuristic values.
      E.g. for states inside the subset, we know the goal distances. They wont become cheaper.
      For now, reevaluation only makes sense in combination with perim.
      In this case we can check if state_h > 0 before we reevaluate.
    */
    if (recompute_h_values && state_h > 0) {
        // TODO: use some incremental Dijkstra search to speed up reevaluation
        vector<int> new_h_values = abstraction.compute_goal_distances(saturated_costs);
        /*
          Apply preprocessing step 2 and 3: heuristic estimates of negative infinity should remain,
          such that they can be pruned by any later operator saturator in the composition.
        */
        for (int source_id = 0; source_id < abstraction.get_num_states(); ++source_id) {
            if (h_values[source_id] == -INF) {
                new_h_values[source_id] = -INF;
            }
        }
        return SaturatorResult(move(saturated_costs), move(new_h_values));
    }
    return SaturatorResult(move(saturated_costs), move(h_values));
}

void Saturator::initialize(
    const Abstractions &, const vector<int> &, const State &) {
}


void add_saturator_options(options::OptionParser &parser) {
    parser.add_option<bool>(
        "use_general_costs",
        "use general costs",
        "true");
    parser.add_option<bool>(
        "recompute_h_values",
        "recompute h values after computing the saturated cost function",
        "false");
}

static PluginTypePlugin<Saturator> _type_plugin(
    "Saturator",
    "");
}
