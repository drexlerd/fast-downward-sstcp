#include "utils.h"

#include "abstraction.h"
#include "abstraction_generator.h"
#include "cost_partitioning_heuristic.h"

#include "../task_utils/task_properties.h"
#include "../tasks/modified_operator_costs_task.h"
#include "../utils/collections.h"
#include "../utils/logging.h"
#include "../utils/math.h"

#include <cassert>
#include <numeric>

using namespace std;

namespace cost_saturation {
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

Abstractions generate_abstractions(
    const shared_ptr<AbstractTask> &task,
    const vector<shared_ptr<AbstractionGenerator>> &abstraction_generators) {
    utils::Timer timer;
    Abstractions abstractions;
    vector<int> abstractions_per_generator;
    for (const shared_ptr<AbstractionGenerator> &generator : abstraction_generators) {
        int abstractions_before = abstractions.size();
        for (auto &abstraction : generator->generate_abstractions(task)) {
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
        assert(is_sum_within_range(a, b));
        return a + b;
    }
}

int path_addition(int a, int b) {
    if (a == INF || b == INF) {
        return INF;
    } else if (a == -INF || b == -INF) {
        return -INF;
    } else {
        assert(is_sum_within_range(a, b));
        return a + b;
    }
}

vector<int> compute_reachability_cost_function(const vector<int> &costs) {
    vector<int> reachability_cost_function = costs;
    for (int &c : reachability_cost_function) {
        if (c == -INF || c == INF) {
            c = INF;
        } else {
            c = 1;
        }
    }
    return reachability_cost_function;
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

void reduce_costs(vector<int> &remaining_costs, const vector<int> &saturated_costs) {
    assert(remaining_costs.size() == saturated_costs.size());
    for (size_t i = 0; i < remaining_costs.size(); ++i) {
        int &remaining = remaining_costs[i];
        int saturated = saturated_costs[i];
        if (saturated > remaining) {
            cerr << "op " << i << ": " << saturated << " > " << remaining << endl;
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
}
