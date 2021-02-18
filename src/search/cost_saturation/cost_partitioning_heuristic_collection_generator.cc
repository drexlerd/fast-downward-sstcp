#include "cost_partitioning_heuristic_collection_generator.h"

#include "cost_partitioning_heuristic.h"
#include "diversifier.h"
#include "order_generator.h"
#include "order_optimizer.h"
#include "saturated_cost_partitioning_heuristic.h"
#include "saturated_cost_partitioning_online_heuristic.h"
#include "saturator.h"
#include "utils.h"

#include "../task_proxy.h"

#include "../task_utils/sampling.h"
#include "../task_utils/task_properties.h"
#include "../utils/collections.h"
#include "../utils/countdown_timer.h"
#include "../utils/logging.h"
#include "../utils/memory.h"

#include <cassert>

using namespace std;

namespace cost_saturation {
static vector<vector<int>> sample_states_and_return_abstract_state_ids(
    const TaskProxy &task_proxy,
    const Abstractions &abstractions,
    sampling::RandomWalkSampler &sampler,
    int num_samples,
    int init_h,
    const DeadEndDetector &is_dead_end,
    double max_sampling_time) {
    assert(num_samples >= 1);
    utils::CountdownTimer sampling_timer(max_sampling_time);
    utils::Log() << "Start sampling" << endl;
    vector<vector<int>> abstract_state_ids_by_sample;
    abstract_state_ids_by_sample.push_back(
        get_abstract_state_ids(abstractions, task_proxy.get_initial_state()));
    while (static_cast<int>(abstract_state_ids_by_sample.size()) < num_samples
           && !sampling_timer.is_expired()) {
        abstract_state_ids_by_sample.push_back(
            get_abstract_state_ids(abstractions, sampler.sample_state(init_h, is_dead_end)));
    }
    utils::Log() << "Samples: " << abstract_state_ids_by_sample.size() << endl;
    utils::Log() << "Sampling time: " << sampling_timer.get_elapsed_time() << endl;
    return abstract_state_ids_by_sample;
}


CostPartitioningHeuristicCollectionGenerator::CostPartitioningHeuristicCollectionGenerator(
    const shared_ptr<OrderGenerator> &order_generator,
    int max_orders,
    double max_time,
    bool diversify,
    int num_samples,
    double max_optimization_time,
    const shared_ptr<utils::RandomNumberGenerator> &rng)
    : order_generator(order_generator),
      max_orders(max_orders),
      max_time(max_time),
      diversify(diversify),
      num_samples(num_samples),
      max_optimization_time(max_optimization_time),
      rng(rng) {
}

vector<CostPartitioningHeuristic>
CostPartitioningHeuristicCollectionGenerator::generate_cost_partitionings(
    const TaskProxy &task_proxy,
    const Abstractions &abstractions,
    const vector<int> &costs,
    const Saturators &saturators,
    const shared_ptr<Saturator> &extra_saturator) const {
    utils::Log log;
    utils::CountdownTimer timer(max_time);

    // Collect stats
    Stats saturators_stats("saturators");
    Stats extra_saturator_stats("extra_saturator");

    State initial_state = task_proxy.get_initial_state();
    vector<int> abstract_state_ids_for_init = get_abstract_state_ids(
        abstractions, initial_state);

    // If any abstraction detects unsolvability in the initial state, we only
    // need a single order (any order suffices).
    CostPartitioningHeuristic default_order_cp = compute_saturated_cost_partitioning(
        abstractions, get_default_order(abstractions.size()), costs);
    if (default_order_cp.compute_heuristic(abstract_state_ids_for_init) == INF) {
        return {
                   default_order_cp
        };
    }

    for (auto &saturator : saturators) {
        saturator->initialize(abstractions, costs, initial_state);
    }
    if (extra_saturator) {
        extra_saturator->initialize(abstractions, costs, initial_state);
    }
    order_generator->initialize(abstractions, costs);

    // Compute h(s_0) using a greedy order for s_0.
    Order order_for_init = order_generator->compute_order_for_state(
        abstractions, costs, abstract_state_ids_for_init, false);
    vector<int> remaining_costs_for_init = costs;
    CostPartitioningHeuristic cp_for_init = compute_saturated_cost_partitioning_with_saturators(
        abstractions, order_for_init, saturators, remaining_costs_for_init, abstract_state_ids_for_init, saturators_stats);
    function<int (const State &state)> sampling_heuristic =
        [&abstractions, &cp_for_init](const State &state) {
            return cp_for_init.compute_heuristic(
                get_abstract_state_ids(abstractions, state));
        };
    int init_h = sampling_heuristic(initial_state);

    // Compute dead end detector which uses the sampling heuristic.
    DeadEndDetector is_dead_end = [&sampling_heuristic](const State &state) {
            return sampling_heuristic(state) == INF;
        };
    sampling::RandomWalkSampler sampler(task_proxy, *rng);

    unique_ptr<Diversifier> diversifier;
    if (diversify) {
        double max_sampling_time = timer.get_remaining_time();
        diversifier = utils::make_unique_ptr<Diversifier>(
            sample_states_and_return_abstract_state_ids(
                task_proxy, abstractions, sampler, num_samples, init_h, is_dead_end, max_sampling_time));
    }

    vector<int> remaining_costs;
    vector<CostPartitioningHeuristic> cp_heuristics;
    int evaluated_orders = 0;
    log << "Start computing cost partitionings" << endl;
    while (static_cast<int>(cp_heuristics.size()) < max_orders &&
           (!timer.is_expired() || cp_heuristics.empty())) {
        bool first_order = (evaluated_orders == 0);

        vector<int> abstract_state_ids;
        Order order;
        CostPartitioningHeuristic cp_heuristic;
        if (first_order) {
            // Use initial state as first sample.
            abstract_state_ids = abstract_state_ids_for_init;
            order = order_for_init;
            cp_heuristic = cp_for_init;
            remaining_costs = remaining_costs_for_init;
            ++saturators_stats.evaluations;
        } else {
            abstract_state_ids = get_abstract_state_ids(
                abstractions, sampler.sample_state(init_h, is_dead_end));
            order = order_generator->compute_order_for_state(
                abstractions, costs, abstract_state_ids, false);
            remaining_costs = costs;
            cp_heuristic = compute_saturated_cost_partitioning_with_saturators(
                abstractions, order, saturators, remaining_costs, abstract_state_ids, saturators_stats);
            ++saturators_stats.evaluations;
        }

        if (extra_saturator) {
            cp_heuristic.add(
                compute_saturated_cost_partitioning_with_saturators(
                    abstractions, order, {extra_saturator}, remaining_costs, abstract_state_ids, extra_saturator_stats));
            ++extra_saturator_stats.evaluations;
        }

        // Optimize order.
        if (max_optimization_time > 0) {
            ABORT("Not implemented.");
            /*
            utils::CountdownTimer timer(max_optimization_time);
            int incumbent_h_value = cp_heuristic.compute_heuristic(abstract_state_ids);
            optimize_order_with_hill_climbing(
                cp_function, timer, abstractions, costs, abstract_state_ids, order,
                cp_heuristic, incumbent_h_value, first_order);
            if (first_order) {
                log << "Time for optimizing order: " << timer.get_elapsed_time()
                    << endl;
            }
            */
        }

        // If diversify=true, only add order if it improves upon previously
        // added orders.
        if (!diversifier || diversifier->is_diverse(cp_heuristic)) {
            cp_heuristics.push_back(move(cp_heuristic));
            if (diversify) {
                log << "Sum over max h values for " << num_samples
                    << " samples after " << timer.get_elapsed_time()
                    << " of diversification: "
                    << diversifier->compute_sum_portfolio_h_value_for_samples()
                    << endl;
            }
        }

        ++evaluated_orders;
    }
    saturators_stats.print_statistics();
    extra_saturator_stats.print_statistics();

    cout << "Peak memory to compute cost partitionings: " << utils::get_peak_memory_in_kb() << " KB\n";
    cout << "Cost partitionings: " << cp_heuristics.size() << "\n";
    cout << "Evaluated orders: " << evaluated_orders << "\n";
    cout << "Total number of abstractions used: " << abstractions.size() << "\n";
    cout << "Num useless operators: " << count(remaining_costs.begin(), remaining_costs.end(), INF) << "\n";
    return cp_heuristics;
}
}
