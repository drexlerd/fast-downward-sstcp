#include "saturator.h"

#include "abstraction.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace transition_cost_partitioning {

// ____________________________________________________________________________
Saturator::Saturator(const options::Options &opts)
    : use_general_costs(opts.get<bool>("use_general_costs")),
      recompute_h_values(opts.get<bool>("recompute_h_values")) {
}

// ____________________________________________________________________________
SaturatorResultOcf Saturator::get_saturator_result_ocf(
    const Abstraction &abstraction,
    const vector<int> &ocf,
    vector<int> &&socf,
    vector<int> &&h_values,
    bool saturate_negative_infinity) const {  
    /*
      Apply preprocessing step 1: if ocf(o) = INF then socf(o) = INF
    */
    assert(ocf.size() == socf.size());
    for (int op_id = 0; op_id < (int)ocf.size(); ++op_id) {
        if (ocf[op_id] == INF) {
            socf[op_id] = INF;
        }
    }    
    /*
      Recomputing goal distances might make sense because of slack in transitions 
      that are outside the subset of states used in the operator saturator.
    */
    if (recompute_h_values) {     
        // TODO: use some incremental Dijkstra search to speed up reevaluation
        vector<int> new_h_values = abstraction.compute_goal_distances_ocf(socf);
        /*
          Apply preprocessing step 2 and 3: heuristic estimates of negative infinity should remain,
          such that they can be pruned by any later operator saturator in the composition.
        */
        if (reachable != Reachable::ALL) {
            for (int source_id = 0; source_id < abstraction.get_num_states(); ++source_id) {
                if (h_values[source_id] == -INF) {
                    new_h_values[source_id] = -INF;
                }
            }
        }
        return SaturatorResultOcf(move(socf), move(new_h_values), saturate_negative_infinity);
    }
    return SaturatorResultOcf(move(socf), move(h_values), saturate_negative_infinity);
}

// ____________________________________________________________________________
void add_saturator_options(options::OptionParser &parser) {
    parser.add_option<bool>(
        "use_general_costs",
        "use general costs",
        "true");
    //parser.add_option<bool>(
    //    "use_general_costs_sd",
    //    "use general costs state-dependent",
    //    "true");
    parser.add_option<bool>(
        "recompute_h_values",
        "recompute h values after computing the saturated cost function",
        "false");
}

// ____________________________________________________________________________
static PluginTypePlugin<Saturator> _type_plugin(
    "cp_saturator",
    "");
}
