#ifndef COST_SATURATION_UTILS_H
#define COST_SATURATION_UTILS_H

#include "abstraction.h"
#include "types.h"

#include <iostream>
#include <vector>

class AbstractTask;
class State;

namespace cost_saturation {
class AbstractionGenerator;

// Multiply all costs by this factor to avoid using real-valued costs.
static const int COST_FACTOR = 1000;

extern std::shared_ptr<AbstractTask> get_scaled_costs_task(
    const std::shared_ptr<AbstractTask> &task, int factor);

extern Abstractions generate_abstractions(
    const std::shared_ptr<AbstractTask> &task,
    const std::vector<std::shared_ptr<AbstractionGenerator>> &abstraction_generators);

extern Order get_default_order(int num_abstractions);

extern bool is_sum_within_range(int a, int b);

// The sum of mixed infinities evaluates to the left infinite value.
extern int left_addition(int a, int b);

// The sum of mixed infinities evaluates to INF.
extern int path_addition(int a, int b);

extern std::vector<int> compute_reachability_cost_function(const std::vector<int> &costs);

extern int compute_max_h_with_statistics(
    const CPHeuristics &cp_heuristics,
    const std::vector<int> &abstract_state_ids,
    std::vector<int> &num_best_order);

template<typename AbstractionsOrFunctions>
std::vector<int> get_abstract_state_ids(
    const AbstractionsOrFunctions &abstractions, const State &state) {
    std::vector<int> abstract_state_ids;
    abstract_state_ids.reserve(abstractions.size());
    for (auto &abstraction : abstractions) {
        if (abstraction) {
            // Only add local state IDs for useful abstractions.
            abstract_state_ids.push_back(abstraction->get_abstract_state_id(state));
        } else {
            // Add dummy value if abstraction will never be used.
            abstract_state_ids.push_back(-1);
        }
    }
    return abstract_state_ids;
}

extern void reduce_costs(
    std::vector<int> &remaining_costs, const std::vector<int> &saturated_costs);

template<typename T>
void print_indexed_vector(const std::vector<T> &vec) {
    for (size_t i = 0; i < vec.size(); ++i) {
        std::cout << i << ":";
        T value = vec[i];
        if (value == INF) {
            std::cout << "inf";
        } else if (value == -INF) {
            std::cout << "-inf";
        } else {
            std::cout << value;
        }
        if (i < vec.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << std::endl;
}
}

#endif
