#include "utils.h"

#include "abstraction.h"
#include "abstraction_generator.h"
#include "cost_partitioning_heuristic.h"

#include "../tasks/root_task.h"
#include "../task_utils/task_properties.h"
#include "../tasks/modified_operator_costs_task.h"
#include "../utils/collections.h"
#include "../utils/logging.h"
#include "../utils/math.h"

#include <cassert>
#include <numeric>

using namespace std;

namespace transition_cost_partitioning {

shared_ptr<AbstractTask> get_scaled_costs_task(
    const shared_ptr<AbstractTask> &task, int factor) {
    vector<int> costs = task_properties::get_operator_costs(TaskProxy(*task));
    for (int &cost : costs) {
        if (!utils::is_product_within_limit(cost, factor, INF)) {
            cerr << "Overflowing cost : " << cost << endl;
            utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
        }
        cost *= factor;
    }
    return make_shared<extra_tasks::ModifiedOperatorCostsTask>(task, move(costs));
}

vector<unique_ptr<Abstraction>> generate_transition_cost_partitioning_abstractions(
    const std::shared_ptr<AbstractTask> &task,
    const TaskInfo &task_info,
    const BddBuilder &bdd_builder,
    const std::vector<std::shared_ptr<AbstractionGenerator>> &abstraction_generators) {
    utils::Timer timer;
    vector<unique_ptr<Abstraction>> abstractions;
    vector<int> abstractions_per_generator;
    for (const shared_ptr<AbstractionGenerator> &generator : abstraction_generators) {
        int abstractions_before = abstractions.size();
        for (auto &abstraction : generator->generate_abstractions(task, task_info, bdd_builder)) {
            abstractions.push_back(move(abstraction));
        }
        abstractions_per_generator.push_back(abstractions.size() - abstractions_before);
    }
    utils::g_log << "Abstractions: " << abstractions.size() << "\n";
    utils::g_log << "Abstractions per generator: " << abstractions_per_generator << "\n";
    cout << "Total time to compute abstractions: " << timer() << "\n" << endl;
    return abstractions;
}

Order get_default_order(int num_abstractions) {
    vector<int> indices(num_abstractions);
    iota(indices.begin(), indices.end(), 0);
    return indices;
}

bool is_sum_within_range(int a, int b) {
    return (b >= 0 && a <= numeric_limits<int>::max() - b) ||
           (b < 0 && a >= numeric_limits<int>::min() - b);
}

int left_addition(int a, int b) {
    if (a == -INF || a == INF) {
        return a;
    } else if (b == -INF || b == INF) {
        return b;
    } else {
        if (!is_sum_within_range(a, b)) {
            cout << "left_addition: " << a << " " << b << endl;
        }
        assert(is_sum_within_range(a, b));
        return a + b;
    }
}

int left_subtraction(int a, int b) {
    if (a == INF || a == -INF) {
        return a;
    } else if (b == INF || b == -INF) {
        return INF;
    } else {
        if (!is_sum_within_range(a, b)) {
            cout << "left_subtraction: " << a << " " << b << endl;
        }
        assert(is_sum_within_range(a, b));
        return a - b;
    }
}

int path_addition(int a, int b) {
    if (a == INF || b == INF) {
        return INF;
    } else if (a == -INF || b == -INF) {
        return -INF;
    } else {
        if (!is_sum_within_range(a, b)) {
            cout << "path_addition: " << a << " " << b << endl;
        }
        assert(is_sum_within_range(a, b));
        return a + b;
    }
}

vector<int> compute_reachability_cost_function(const vector<int> &costs) {
    vector<int> reachability_cost_function = costs;
    for (size_t op_id = 0; op_id < reachability_cost_function.size(); ++op_id) {
        int &c = reachability_cost_function[op_id];
        if (c == -INF || c == INF) {
            c = INF;
        } else {
            c = 1;
        }
    }
    return reachability_cost_function;
}


bool is_infimum_stcf(const Abstraction &abstraction, const vector<int> &tcf, const vector<int> &stcf, const vector<int> &h_values) {
    bool result = true;
    abstraction.for_each_transition([&](const Transition &transition) {
        int source_id = transition.source_id;
        int target_id = transition.target_id;
        int source_h = h_values[source_id];
        int target_h = h_values[target_id];
        if (tcf[transition.transition_id] == INF) {
            return;
        }
        if ((source_h == INF || source_h == -INF || 
            target_h == INF || target_h == -INF ) &&
            stcf[transition.transition_id] != -INF) {
            result = false;
        }
    });
    return result;
}

int compute_max_h_with_statistics(
    const CPHeuristics &cp_heuristics,
    const vector<int> &abstract_state_ids,
    vector<int> &num_best_order) {
    int max_h = 0;
    int best_id = -1;
    int current_id = 0;
    for (const CostPartitioningHeuristic &cp_heuristic : cp_heuristics) {
        int sum_h = cp_heuristic.compute_heuristic(abstract_state_ids);
        if (sum_h > max_h) {
            max_h = sum_h;
            best_id = current_id;
        }
        if (sum_h == INF) {
            break;
        }
        ++current_id;
    }
    assert(max_h >= 0);

    num_best_order.resize(cp_heuristics.size(), 0);
    if (best_id != -1) {
        ++num_best_order[best_id];
    }

    return max_h;
}

void reduce_costs(std::vector<int> &remaining_costs, const std::vector<int> &saturated_costs) {
    assert(remaining_costs.size() == saturated_costs.size());
    for (size_t op_id = 0; op_id < remaining_costs.size(); ++op_id) {
        int &remaining = remaining_costs[op_id];
        int saturated = saturated_costs[op_id];
        if (saturated > remaining) {
            cerr << "op " << op_id << ": " << saturated << " > " << remaining << endl;
            ABORT("invalid saturated cost function");
        }
        if (remaining == INF || remaining == -INF) {
            // Left addition: x - y = x for all values y if x is infinite.
        } else if (saturated == -INF) {
            remaining = INF;
        } else {
            assert(saturated != INF);
            remaining -= saturated;
        }
        assert(remaining >= 0);
    }
}

bool verify_saturated_costs_transition(
    const Abstraction &abstraction, 
    const vector<int> saturated_costs_operator, 
    const vector<int> saturated_costs_transition) {
    bool is_valid = true;
    abstraction.for_each_transition(
        [&saturated_costs_operator, &saturated_costs_transition, &is_valid](const Transition &transition) {
            int op_id = transition.op_id;
            int transition_id = transition.transition_id;
            // saturated costs operator has to upper bound the saturated costs transition
            if (saturated_costs_operator[op_id] < saturated_costs_transition[transition_id]) {
                cout << "sat_I: " << saturated_costs_operator[op_id] << " sat_D: " << saturated_costs_transition[transition_id] << endl;
                is_valid = false;
            }
        }
    );
    return is_valid;
}


}
