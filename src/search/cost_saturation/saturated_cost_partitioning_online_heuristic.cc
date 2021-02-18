#include "saturated_cost_partitioning_online_heuristic.h"

#include "abstraction.h"
#include "cost_partitioning_heuristic.h"
#include "max_cost_partitioning_heuristic.h"
#include "order_generator.h"
#include "saturator.h"
#include "utils.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../task_utils/task_properties.h"
#include "../utils/logging.h"

using namespace std;

namespace cost_saturation {
CostPartitioningHeuristic compute_saturated_cost_partitioning_with_saturators(
    const Abstractions &abstractions,
    const vector<int> &order,
    const Saturators &saturators,
    vector<int> &remaining_costs,
    const vector<int> &abstract_state_ids,
    Stats &stats) {
    assert(abstractions.size() == order.size());
    CostPartitioningHeuristic cp_heuristic;
    for (int abstraction_id : order) {
        const Abstraction &abstraction = *abstractions[abstraction_id];
        int state_id = abstract_state_ids[abstraction_id];

        stats.saturator_timer_saturate.resume();
        vector<int> saturated_costs = remaining_costs;
        vector<int> h_values = abstraction.compute_goal_distances(saturated_costs);
        int state_h = h_values[state_id];
        utils::unused_variable(state_h);
        for (auto &saturator : saturators) {
            SaturatorResult result = saturator->saturate(
                abstraction, abstraction_id, saturated_costs, move(h_values), state_id);
            saturated_costs = move(result.saturated_costs);
            h_values = move(result.h_values);
            assert(h_values[state_id] == state_h);
        }
        cp_heuristic.add_h_values(abstraction_id, move(h_values));
        stats.saturator_timer_saturate.stop();

        stats.saturator_timer_reduce.resume();
        reduce_costs(remaining_costs, saturated_costs);
        stats.saturator_timer_reduce.stop();
    }
    return cp_heuristic;
}

SaturatedCostPartitioningOnlineHeuristic::SaturatedCostPartitioningOnlineHeuristic(
    const options::Options &opts,
    Abstractions &&abstractions)
    : Heuristic(opts),
      cp_generator(opts.get<shared_ptr<OrderGenerator>>("orders")),
      abstractions(move(abstractions)),
      costs(task_properties::get_operator_costs(task_proxy)),
      saturators(opts.get_list<shared_ptr<Saturator>>("saturators")),
      num_scps_computed(0) {
    State initial_state = task_proxy.get_initial_state();
    for (auto &saturator : saturators) {
        saturator->initialize(this->abstractions, costs, initial_state);
    }
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
    CostPartitioningHeuristic cost_partitioning = compute_saturated_cost_partitioning_with_saturators(
        abstractions, order, saturators, remaining_costs, abstract_state_ids, stats);
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
        "greedy_orders()");

    Options opts = parser.parse();
    if (parser.help_mode())
        return nullptr;

    if (parser.dry_run())
        return nullptr;

    shared_ptr<AbstractTask> task = get_scaled_costs_task(
        opts.get<shared_ptr<AbstractTask>>("transform"), COST_FACTOR);
    opts.set<shared_ptr<AbstractTask>>("transform", task);
    Abstractions abstractions = generate_abstractions(
        task, opts.get_list<shared_ptr<AbstractionGenerator>>("abstraction_generators"));

    return make_shared<SaturatedCostPartitioningOnlineHeuristic>(
        opts,
        move(abstractions));
}

static Plugin<Evaluator> _plugin("saturated_cost_partitioning_online", _parse);
}
