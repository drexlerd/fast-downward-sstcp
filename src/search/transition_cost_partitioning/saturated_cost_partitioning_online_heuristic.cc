#include "saturated_cost_partitioning_online_heuristic.h"

#include "abstract_transition_cost_function.h"
#include "abstraction_mask_generator.h"
#include "operator_mask_generator.h"
#include "cost_function_state_dependent.h"
#include "abstraction.h"
#include "bdd_builder.h"
#include "cost_partitioning_heuristic.h"
#include "max_cost_partitioning_heuristic.h"
#include "order_generator.h"
#include "saturator.h"
#include "utils.h"
#include "task_info.h"


#include "../option_parser.h"
#include "../plugin.h"

#include "../task_utils/task_properties.h"
#include "../utils/logging.h"

#include <unordered_map>
#include <iostream>

using namespace std;

namespace transition_cost_partitioning {


// ____________________________________________________________________________
CostPartitioningHeuristic compute_saturated_cost_partitioning_with_saturators(
    const TaskInfo &task_info,
    const Abstractions &abstractions,
    const OperatorMaskGenerator &,
    const AbstractionMaskGenerator &abstraction_mask_generator,
    const std::vector<int> &order,
    const Saturators &saturators,  
    const std::vector<int> &abstract_state_ids,
    CostFunctionStateDependent &sdac,
    Stats &stats) {    
    assert(abstractions.size() == order.size());

    CostPartitioningHeuristic cp_heuristic;
    for (int abstraction_id : order) {
        // cout << "Abstraction " << abstraction_id << endl;
        Abstraction &abstraction = *abstractions[abstraction_id];
        int state_id = abstract_state_ids[abstraction_id];

        bool abstraction_sd = abstraction_mask_generator.generate_mask(abstraction, task_info);
        if (abstraction_sd) {
            // abstraction is handled state-dependent.
            stats.saturator_timer_saturate.resume();

            AbstractTransitionCostFunction stcf(abstraction);
            vector<int> h_values;
            bool saturate_negative_infinity = false;        
            for (auto &saturator : saturators) {
                SaturatorResultTcf result = saturator->saturate_tcf(
                    abstraction, move(stcf), sdac, move(h_values), state_id);
                stcf = move(result.stcf);
                h_values = move(result.h_values);
                saturate_negative_infinity = result.saturate_negative_infinity;
            }
            stats.saturator_timer_saturate.stop();
            
            stats.saturator_timer_reduce.resume();
            sdac.reduce_operator_costs(stcf);
            sdac.reduce_transition_costs_finite(abstraction, stcf);
            if (saturate_negative_infinity) {
                sdac.reduce_transition_costs_negative_infinity(abstraction, h_values);
            }
            stats.saturator_timer_reduce.stop();

            cp_heuristic.add_h_values(abstraction_id, move(h_values));
        } else {
            // abstraction is handled state-independent.
            stats.saturator_timer_saturate.resume();
            vector<int> socf = sdac.determine_remaining_costs_operator();
            vector<int> h_values = abstraction.compute_goal_distances_ocf(socf);
            for (auto &saturator : saturators) {
                SaturatorResultOcf result = saturator->saturate_ocf(
                    abstraction, socf, move(h_values), state_id);
                socf = move(result.socf);
                h_values = move(result.h_values);
            }
            stats.saturator_timer_saturate.stop();

            stats.saturator_timer_reduce.resume();
            sdac.reduce_operator_costs(socf);
            stats.saturator_timer_reduce.stop();

            cp_heuristic.add_h_values(abstraction_id, move(h_values));
        }
    }

    return cp_heuristic;
}

SaturatedCostPartitioningOnlineHeuristic::SaturatedCostPartitioningOnlineHeuristic(
    const options::Options &opts,
    Abstractions &&abstractions,
    vector<int> &&costs)
    : Heuristic(opts),
      cp_generator(opts.get<shared_ptr<OrderGenerator>>("orders")),
      abstractions(move(abstractions)),
      costs(move(costs)),
      saturators(opts.get_list<shared_ptr<Saturator>>("saturators")),
      num_scps_computed(0) {

    cp_generator->initialize(this->abstractions, costs);
    utils::Log() << "Done initializing SCP online heuristic." << endl;
}

SaturatedCostPartitioningOnlineHeuristic::~SaturatedCostPartitioningOnlineHeuristic() {
    print_statistics();
}

int SaturatedCostPartitioningOnlineHeuristic::compute_heuristic(
    const GlobalState &global_state) {
    State state = convert_global_state(global_state);
    vector<int> abstract_state_ids = get_abstract_state_ids(abstractions, state);
    Order order = cp_generator->compute_order_for_state(
        abstractions, costs, abstract_state_ids, num_scps_computed == 0);
    vector<int> remaining_costs = costs;
    Stats stats("saturators");
    pair<CostPartitioningHeuristic, vector<SaturatorResultOcf>> result; // = compute_saturated_cost_partitioning_with_operator_saturators(
    //    abstractions, order, saturators, saturator_info, remaining_costs, abstract_state_ids, stats);
    CostPartitioningHeuristic &cost_partitioning = result.first;
    ++num_scps_computed;
    int h = cost_partitioning.compute_heuristic(abstract_state_ids);
    if (h == INF) {
        return DEAD_END;
    }
    double epsilon = 0.01;
    return static_cast<int>(ceil((h / static_cast<double>(COST_FACTOR)) - epsilon));
}

void SaturatedCostPartitioningOnlineHeuristic::print_statistics() const {
    cout << "Computed SCPs: " << num_scps_computed << endl;
}


static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Saturated cost partitioning online heuristic",
        "");

    prepare_parser_for_cost_partitioning_heuristic(parser);
    add_scp_options_to_parser(parser);

    parser.add_option<shared_ptr<OrderGenerator>>(
        "orders",
        "order generator",
        "cp_greedy_orders()");

    Options opts = parser.parse();
    if (parser.help_mode())
        return nullptr;

    if (parser.dry_run())
        return nullptr;

    shared_ptr<AbstractTask> task = get_scaled_costs_task(
         opts.get<shared_ptr<AbstractTask>>("transform"), COST_FACTOR);
    opts.set<shared_ptr<AbstractTask>>("transform", task);

    TaskProxy task_proxy(*task);
    TaskInfo task_info(task_proxy);
    BddBuilder bdd_builder(task_info);

    // 1. Generate cartesian abstractions
    vector<unique_ptr<Abstraction>> abstractions = generate_transition_cost_partitioning_abstractions(
        task,
        task_info,
        bdd_builder,
        opts.get_list<shared_ptr<AbstractionGenerator>>("abstraction_generators"));

    vector<int> costs = task_properties::get_operator_costs(TaskProxy(*task));

    return make_shared<SaturatedCostPartitioningOnlineHeuristic>(
        opts,
        move(abstractions),
        move(costs));
}

static Plugin<Evaluator> _plugin("saturated_transition_cost_partitioning_online", _parse);
}
