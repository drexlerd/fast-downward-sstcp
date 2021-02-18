#include "abstraction.h"

#include "explicit_abstraction.h"
#include "types.h"
#include "utils.h"

#include "../algorithms/priority_queues.h"
#include "../utils/collections.h"
#include "../utils/logging.h"
#include "../utils/memory.h"

#include <cassert>

using namespace std;

namespace cost_saturation {
Graph get_forward_graph(const Abstraction &abstraction) {
    Graph forward_graph(abstraction.get_num_states());
    abstraction.for_each_transition(
        [&forward_graph](const Transition &t) {
            forward_graph[t.src].emplace_back(t.op, t.target);
        });
    for (auto &transitions : forward_graph) {
        transitions.shrink_to_fit();
    }
    return forward_graph;
}

vector<int> compute_forward_distances(
    const Graph &forward_graph, const vector<int> &costs, int state_id) {
    vector<int> state_distances(forward_graph.size(), INF);
    state_distances[state_id] = 0;
    priority_queues::AdaptiveQueue<int> queue;
    queue.push(0, state_id);
    dijkstra_search(forward_graph, costs, queue, state_distances);
    return state_distances;
}

Abstraction::Abstraction(unique_ptr<AbstractionFunction> abstraction_function)
    : abstraction_function(move(abstraction_function)) {
}

int Abstraction::get_abstract_state_id(const State &concrete_state) const {
    assert(abstraction_function);
    return abstraction_function->get_abstract_state_id(concrete_state);
}

unique_ptr<AbstractionFunction> Abstraction::extract_abstraction_function() {
    return move(abstraction_function);
}

vector<int> Abstraction::compute_goal_distances_for_general_costs(
    const vector<int> &costs) const {
    int num_states = get_num_states();
    vector<int> distances(num_states, INF);

    bool negative_weight_cycle_found = false;
    do {
        negative_weight_cycle_found = false;
        for (int &d : distances) {
            // Reset distances but keep distances of -\infty.
            if (d != -INF) {
                d = INF;
            }
        }
        for (int goal : get_goal_states()) {
            distances[goal] = 0;
        }
        for (int i = 0; i < num_states; ++i) {
            bool last_round = (i == num_states - 1);
            // Optimization: break if no distances change.
            bool distances_changed = false;
            for_each_transition(
                [this, last_round, &negative_weight_cycle_found, &distances_changed,
                 &costs, &distances](const Transition &t) {
                    // Convert forward to backward transition.
                    int src = t.target;
                    int target = t.src;
                    int cost = costs[t.op];
                    int new_distance = path_addition(distances[src], cost);
                    if (new_distance < distances[target]) {
                        if (last_round) {
                            negative_weight_cycle_found = true;
                            // For all states s that can reach target, set h(s) = -\infty.
                            vector<bool> can_reach_target =
                                compute_reachability_to_state(
                                    compute_reachability_cost_function(costs), target);
                            for (size_t i = 0; i < can_reach_target.size(); ++i) {
                                if (can_reach_target[i]) {
                                    distances[target] = -INF;
                                }
                            }
                            assert(distances[target] == -INF);
                            return;
                        } else {
                            distances[target] = new_distance;
                            distances_changed = true;
                        }
                    }
                });
            if (!distances_changed) {
                break;
            }
        }
    } while (negative_weight_cycle_found);

    return distances;
}

vector<int> Abstraction::compute_goal_distances(const vector<int> &costs) const {
    if (all_of(costs.begin(), costs.end(), [](int c) {return c >= 0;})) {
#ifndef NDEBUG
        vector<int> distances_dijkstra = compute_goal_distances_for_non_negative_costs(costs);
        vector<int> distances_bellman_ford = compute_goal_distances_for_general_costs(costs);
        assert(distances_dijkstra == distances_bellman_ford);
#endif
        return compute_goal_distances_for_non_negative_costs(costs);
    } else {
        return compute_goal_distances_for_general_costs(costs);
    }
}
}
