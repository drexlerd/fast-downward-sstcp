#include "explicit_abstraction.h"

#include "types.h"
#include "utils.h"

#include "../utils/collections.h"
#include "../utils/logging.h"

#include <unordered_set>

using namespace std;

namespace cost_saturation {
void dijkstra_search(
    const vector<vector<Successor>> &graph,
    const vector<int> &costs,
    priority_queues::AdaptiveQueue<int> &queue,
    vector<int> &distances) {
    assert(all_of(costs.begin(), costs.end(), [](int c) {return c >= 0;}));
    while (!queue.empty()) {
        pair<int, int> top_pair = queue.pop();
        int distance = top_pair.first;
        int state = top_pair.second;
        int state_distance = distances[state];
        assert(state_distance <= distance);
        if (state_distance < distance) {
            continue;
        }
        for (const Successor &transition : graph[state]) {
            int successor = transition.state;
            int op = transition.op;
            assert(utils::in_bounds(op, costs));
            int cost = costs[op];
            assert(cost >= 0);
            int successor_distance = (cost == INF) ? INF : state_distance + cost;
            assert(successor_distance >= 0);
            if (distances[successor] > successor_distance) {
                distances[successor] = successor_distance;
                queue.push(successor_distance, successor);
            }
        }
    }
}

ostream &operator<<(ostream &os, const Successor &successor) {
    os << "(" << successor.op << ", " << successor.state << ")";
    return os;
}

static vector<bool> get_active_operators_from_graph(
    const vector<vector<Successor>> &backward_graph, int num_ops) {
    vector<bool> active_operators(num_ops, false);
    int num_states = backward_graph.size();
    for (int target = 0; target < num_states; ++target) {
        for (const Successor &transition : backward_graph[target]) {
            int op_id = transition.op;
            active_operators[op_id] = true;
        }
    }
    return active_operators;
}

ExplicitAbstraction::ExplicitAbstraction(
    unique_ptr<AbstractionFunction> abstraction_function,
    vector<vector<Successor>> &&backward_graph_,
    vector<bool> &&looping_operators,
    vector<int> &&goal_states)
    : Abstraction(move(abstraction_function)),
      backward_graph(move(backward_graph_)),
      active_operators(get_active_operators_from_graph(
                           backward_graph, looping_operators.size())),
      looping_operators(move(looping_operators)),
      goal_states(move(goal_states)) {
#ifndef NDEBUG
    for (int target = 0; target < get_num_states(); ++target) {
        // Check that no transition is stored multiple times.
        vector<Successor> copied_transitions = this->backward_graph[target];
        sort(copied_transitions.begin(), copied_transitions.end());
        assert(utils::is_sorted_unique(copied_transitions));
        // Check that we don't store self-loops.
        assert(all_of(copied_transitions.begin(), copied_transitions.end(),
                      [target](const Successor &succ) {return succ.state != target;}));
    }
#endif
}

vector<bool> ExplicitAbstraction::compute_reachability_from_state(
    const vector<int> &costs, int state_id) const {
    vector<bool> reachable(get_num_states(), false);
    Graph forward_graph = get_forward_graph(*this);
    vector<int> open = {state_id};
    reachable[state_id] = true;
    while (!open.empty()) {
        int current_state = open.back();
        open.pop_back();
        for (const Successor &successor : forward_graph[current_state]) {
            if (!reachable[successor.state] && costs[successor.op] != INF) {
                reachable[successor.state] = true;
                open.push_back(successor.state);
            }
        }
    }

#ifndef NDEBUG
    vector<int> distances = compute_forward_distances(forward_graph, costs, state_id);
    vector<bool> reachable_old(get_num_states(), false);
    for (int i = 0; i < get_num_states(); ++i) {
        reachable_old[i] = distances[i] != INF;
    }
    assert(reachable == reachable_old);
#endif

    return reachable;
}

vector<bool> ExplicitAbstraction::compute_reachability_to_state(
    const vector<int> &costs, int state_id) const {
    vector<bool> reachable(get_num_states(), false);
    vector<int> open = {state_id};
    reachable[state_id] = true;
    while (!open.empty()) {
        int current_state = open.back();
        open.pop_back();
        for (const Successor &predecessor : backward_graph[current_state]) {
            if (!reachable[predecessor.state] && costs[predecessor.op] != INF) {
                reachable[predecessor.state] = true;
                open.push_back(predecessor.state);
            }
        }
    }
    return reachable;
}

vector<int> ExplicitAbstraction::compute_goal_distances_for_non_negative_costs(
    const vector<int> &costs) const {
    assert(all_of(costs.begin(), costs.end(), [](int c) {return c >= 0;}));
    vector<int> goal_distances(get_num_states(), INF);
    queue.clear();
    for (int goal_state : goal_states) {
        goal_distances[goal_state] = 0;
        queue.push(0, goal_state);
    }
    dijkstra_search(backward_graph, costs, queue, goal_distances);
    return goal_distances;
}

vector<int> ExplicitAbstraction::compute_saturated_costs(
    const vector<int> &h_values) const {
    int num_operators = get_num_operators();
    vector<int> saturated_costs(num_operators, -INF);

    /* To prevent negative cost cycles we ensure that all operators
       inducing self-loops have non-negative costs. */
    for (int op_id = 0; op_id < num_operators; ++op_id) {
        if (looping_operators[op_id]) {
            saturated_costs[op_id] = 0;
        }
    }

    int num_states = backward_graph.size();
    for (int target = 0; target < num_states; ++target) {
        assert(utils::in_bounds(target, h_values));
        int target_h = h_values[target];
        if (target_h == INF || target_h == -INF) {
            continue;
        }

        for (const Successor &transition : backward_graph[target]) {
            int op_id = transition.op;
            int src = transition.state;
            assert(utils::in_bounds(src, h_values));
            int src_h = h_values[src];
            if (src_h == INF || src_h == -INF) {
                continue;
            }

            const int needed = src_h - target_h;
            saturated_costs[op_id] = max(saturated_costs[op_id], needed);
        }
    }
    return saturated_costs;
}

int ExplicitAbstraction::get_num_operators() const {
    return looping_operators.size();
}

int ExplicitAbstraction::get_num_states() const {
    return backward_graph.size();
}

bool ExplicitAbstraction::operator_is_active(int op_id) const {
    return active_operators[op_id];
}

bool ExplicitAbstraction::operator_induces_self_loop(int op_id) const {
    return looping_operators[op_id];
}

void ExplicitAbstraction::for_each_transition(const TransitionCallback &callback) const {
    int num_states = get_num_states();
    for (int target = 0; target < num_states; ++target) {
        for (const Successor &transition : backward_graph[target]) {
            int op_id = transition.op;
            int src = transition.state;
            callback(Transition(src, op_id, target));
        }
    }
}

const vector<int> &ExplicitAbstraction::get_goal_states() const {
    return goal_states;
}

void ExplicitAbstraction::dump() const {
    cout << "State-changing transitions:" << endl;
    for (int state = 0; state < get_num_states(); ++state) {
        if (!backward_graph[state].empty()) {
            cout << "  " << state << " <- " << backward_graph[state] << endl;
        }
    }
    cout << "Looping operators: " << looping_operators << endl;
    cout << "Goal states: " << goal_states << endl;
}
}
