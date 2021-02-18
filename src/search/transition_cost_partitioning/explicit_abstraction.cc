#include "explicit_abstraction.h"

#include "abstract_transition_cost_function.h"
#include "cost_function_state_dependent.h"
#include "abstraction.h"
#include "bdd_builder.h"
#include "utils.h"
#include "task_info.h"

#include "../task_proxy.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"
#include "../utils/collections.h"
#include "../cegar/transition_system.h"

#include <algorithm>

namespace transition_cost_partitioning {

// ____________________________________________________________________________
static vector<vector<Successor>> generate_forward_graph(
    const vector<vector<Successor>> &backward_graph) {
    vector<vector<Successor>> forward_graph(backward_graph.size());
    for (size_t target_id = 0; target_id < backward_graph.size(); ++target_id) {
        for (const Successor &transition : backward_graph[target_id]) {
            int source_id = transition.target_id;
            forward_graph[source_id].emplace_back(Successor(transition.transition_id, transition.op_id, target_id));
        }
    }
    return forward_graph;
}

// ____________________________________________________________________________
static void dijkstra_search_ocf(
  const vector<vector<Successor>> &graph,
  const vector<int> &ocf, 
  priority_queues::AdaptiveQueue<int> &queue, 
  vector<int> &distances) {
    assert(all_of(ocf.begin(), ocf.end(), [](int c) {return c >= 0;}));

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
            int successor = transition.target_id;
            int op = transition.op_id;
            assert(utils::in_bounds(op, ocf));
            int cost = ocf[op];
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

// ____________________________________________________________________________
static void dijkstra_search_tcf(
  const Abstraction &abstraction,
  const vector<vector<Successor>> &graph,
  const CostFunctionStateDependent &sdac,
  AbstractTransitionCostFunction &tcf,
  priority_queues::AdaptiveQueue<int> &queue, 
  vector<int> &distances) {
    vector<int> &sd_costs = tcf.get_sd_costs();
    fill(sd_costs.begin(), sd_costs.end(), 0);
    while (!queue.empty()) {
        pair<int, int> top_pair = queue.pop();
        int distance = top_pair.first;
        int state = top_pair.second;
        int state_distance = distances[state];        
        assert(state_distance <= distance);
        if (distance > state_distance) {
            continue;
        }
        // mark the state after popping it from the queue
        for (const Successor &transition : graph[state]) {
            int successor = transition.target_id;
            int required = distances[successor] - distances[state];
            int cost = 0;
            int op_id = transition.op_id;
            if (required > 0) {
                cost = sdac.determine_remaining_costs_operator(op_id);
                if (cost < required) {
                    cost = sdac.determine_remaining_costs_transition(abstraction, Transition(transition.transition_id, op_id, successor, state), required);
                }
            }            
            assert(cost >= 0);
            sd_costs[transition.transition_id] = cost;
            int successor_distance = (cost == INF) ? INF : state_distance + cost;
            assert(successor_distance >= 0);
            if (distances[successor] > successor_distance) {
                distances[successor] = successor_distance;
                queue.push(successor_distance, successor);
            }
        }
    }
}

// ____________________________________________________________________________
static void dijkstra_search_tcf(
  const vector<vector<Successor>> &graph,
  AbstractTransitionCostFunction &tcf,
  priority_queues::AdaptiveQueue<int> &queue, 
  vector<int> &distances) {
    const vector<int> &sd_costs = tcf.get_sd_costs();
    while (!queue.empty()) {
        pair<int, int> top_pair = queue.pop();
        int distance = top_pair.first;
        int state = top_pair.second;
        int state_distance = distances[state];        
        assert(state_distance <= distance);
        if (distance > state_distance) {
            continue;
        }
        for (const Successor &transition : graph[state]) {
            int successor = transition.target_id;
            int cost = sd_costs[transition.transition_id];
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

// ____________________________________________________________________________
ExplicitAbstraction::ExplicitAbstraction(
    const TaskInfo &task_info,
    const BddBuilder &bdd_builder,
    unique_ptr<AbstractionFunction> abstraction_function,
    int num_transitions,
    int num_states,
    int init_state_id,
    unordered_set<int> &&goal_states,
    vector<vector<Successor>> &&backward_graph,
    vector<int> &&num_transitions_by_operator,
    vector<bool> &&has_outgoing,
    vector<bool> &&has_loop) : 
    Abstraction(task_info, bdd_builder, move(abstraction_function), num_transitions, num_states, init_state_id, move(goal_states)),
    backward_graph(move(backward_graph)),
    num_transitions_by_operator(move(num_transitions_by_operator)),
    has_loop(move(has_loop)),
    has_outgoing(move(has_outgoing)) {
}

// ____________________________________________________________________________
ExplicitAbstraction::~ExplicitAbstraction() {
}

// ____________________________________________________________________________
void ExplicitAbstraction::for_each_transition(
    const TransitionCallback &callback) const {
    for (int target_id = 0; target_id < get_num_states(); ++target_id) {
        for (const Successor &transition : backward_graph[target_id]) {
            callback(Transition(transition.transition_id, transition.op_id, transition.target_id, target_id));
        }
    }
}

// ____________________________________________________________________________
vector<int> 
ExplicitAbstraction::compute_goal_distances_for_non_negative_costs_ocf(
  const vector<int> &ocf) const {
    assert(all_of(ocf.begin(), ocf.end(), [](int c) {return c >= 0;}));
    vector<int> goal_distances = vector<int>(get_num_states(), INF);
    queue.clear();
    for (int goal_state : goal_states) {
        goal_distances[goal_state] = 0;
        queue.push(0, goal_state);
    }
    dijkstra_search_ocf(backward_graph, ocf, queue, goal_distances);
    return goal_distances;
}

// ____________________________________________________________________________
vector<int> 
ExplicitAbstraction::compute_goal_distances_for_non_negative_costs_tcf(
    const CostFunctionStateDependent &sdac,
    AbstractTransitionCostFunction &tcf) const {
    vector<int> goal_distances(get_num_states(), INF);
    queue.clear();
    for (int goal_state : goal_states) {
        goal_distances[goal_state] = 0;
        queue.push(0, goal_state);
    }
    dijkstra_search_tcf(*this, backward_graph, sdac, tcf, queue, goal_distances);
    return goal_distances;  
}

// ____________________________________________________________________________
vector<int> 
ExplicitAbstraction::compute_goal_distances_for_non_negative_costs_tcf(
    AbstractTransitionCostFunction &tcf) const {
    vector<int> goal_distances(get_num_states(), INF);
    queue.clear();
    for (int goal_state : goal_states) {
        goal_distances[goal_state] = 0;
        queue.push(0, goal_state);
    }
    dijkstra_search_tcf(backward_graph, tcf, queue, goal_distances);
    return goal_distances;   
}

// ____________________________________________________________________________
vector<bool> ExplicitAbstraction::compute_reachability_from_state_ocf(
    const vector<int> &ocf, int state_id) const {
    vector<int> state_distances = vector<int>(get_num_states(), INF);
    queue.clear();
    state_distances[state_id] = 0;
    queue.push(0, state_id);
    if (forward_graph.empty()) {
        forward_graph = generate_forward_graph(backward_graph);
    }
    dijkstra_search_ocf(forward_graph, ocf, queue, state_distances);
    vector<bool> reachable_from_state(get_num_states(), false);
    for (int source_id = 0; source_id < get_num_states(); ++source_id) {
        if (state_distances[source_id] != INF) {
            reachable_from_state[source_id] = true;
        }
    }
    return reachable_from_state;
}

// ____________________________________________________________________________
vector<bool> ExplicitAbstraction::compute_reachability_from_state_tcf(
    AbstractTransitionCostFunction &tcf, 
    int state_id) const {
    vector<int> state_distances = vector<int>(get_num_states(), INF);
    queue.clear();
    state_distances[state_id] = 0;
    queue.push(0, state_id);
    if (forward_graph.empty()) {
        forward_graph = generate_forward_graph(backward_graph);
    }
    dijkstra_search_tcf(forward_graph, tcf, queue, state_distances);
    vector<bool> reachable_from_state(get_num_states(), false);
    for (int source_id = 0; source_id < get_num_states(); ++source_id) {
        if (state_distances[source_id] != INF) {
            reachable_from_state[source_id] = true;
        }
    }
    return reachable_from_state;
}
  
// ____________________________________________________________________________
vector<bool> ExplicitAbstraction::compute_reachability_to_state_ocf(
    const vector<int> &ocf, int state_id) const {
    vector<int> state_distances = vector<int>(get_num_states(), INF);
    queue.clear();
    state_distances[state_id] = 0;
    queue.push(0, state_id);
    dijkstra_search_ocf(backward_graph, ocf, queue, state_distances);
    vector<bool> reachable_to_state(get_num_states(), false);
    for (int source_id = 0; source_id < get_num_states(); ++source_id) {
        if (state_distances[source_id] != INF) {
            reachable_to_state[source_id] = true;
        }
    }
    return reachable_to_state;
}

// ____________________________________________________________________________
vector<bool> ExplicitAbstraction::compute_reachability_to_state_tcf(
    AbstractTransitionCostFunction &tcf, 
    int state_id) const {
    vector<int> state_distances = vector<int>(get_num_states(), INF);
    queue.clear();
    state_distances[state_id] = 0;
    queue.push(0, state_id);
    dijkstra_search_tcf(backward_graph, tcf, queue, state_distances);
    vector<bool> reachable_to_state(get_num_states(), false);
    for (int source_id = 0; source_id < get_num_states(); ++source_id) {
        if (state_distances[source_id] != INF) {
            reachable_to_state[source_id] = true;
        }
    }
    return reachable_to_state;
}

// ____________________________________________________________________________
vector<int> 
ExplicitAbstraction::compute_saturated_costs_ocf(
    const vector<int> &h_values) const {
    vector<int> socf(task_info.get_num_operators(), -INF);
    /* To prevent negative cost cycles we ensure that all operators
       inducing self-loops have non-negative costs. */
    for (int op_id = 0; op_id < task_info.get_num_operators(); ++op_id) {
        if (operator_induces_self_loop(op_id)) {
            socf[op_id] = 0;
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
            int op_id = transition.op_id;
            int src = transition.target_id;
            assert(utils::in_bounds(src, h_values));
            int src_h = h_values[src];
            if (src_h == INF || src_h == -INF) {
                continue;
            }
            const int needed = src_h - target_h;
            socf[op_id] = max(socf[op_id], needed);
        }
    }
    return socf;
}

// ____________________________________________________________________________
void ExplicitAbstraction::compute_saturated_costs_tcf(
    const vector<int> &h_values, 
    AbstractTransitionCostFunction &stcf) const {
    vector<int> &sd_costs = stcf.get_sd_costs();
    vector<bool> &si = stcf.get_si();
    vector<int> &si_costs = stcf.get_si_costs();
    fill(sd_costs.begin(), sd_costs.end(), -INF);
    // Initially: For each operator holds that stcf does not deviate from socf
    fill(si.begin(), si.end(), true);
    fill(si_costs.begin(), si_costs.end(), -INF);
    for (int target = 0; target < static_cast<int>(backward_graph.size()); ++target) {
        assert(utils::in_bounds(target, h_values));
        int target_h = h_values[target];
        if (target_h == INF || target_h == -INF) {
            continue;
        }
        for (const Successor &transition : backward_graph[target]) {            
            int src = transition.target_id;
            assert(utils::in_bounds(src, h_values));
            int src_h = h_values[src];
            if (src_h == INF || src_h == -INF) {
                continue;
            }
            int op_id = transition.op_id;
            int needed = src_h - target_h;
            // stcf deviates from socf
            if (si[op_id] && 
                needed != si_costs[op_id] && 
                si_costs[op_id] != -INF) {
                si[op_id] = false;
            }
            sd_costs[transition.transition_id] = needed;
            si_costs[op_id] = max(needed, si_costs[op_id]);            
        }
    }
    for (int op_id = 0; op_id < get_num_operators(); ++op_id) {
        if (operator_induces_self_loop(op_id)) {
            // stcf deviates from socf: 
            // We require si_costs[op_id] != -INF because only in this specific case 
            // we have to iterate over all transitions with label op_id 
            if (si_costs[op_id] != 0 && si_costs[op_id] != -INF) {
                si[op_id] = false;
            }
            /* To prevent negative cost cycles we ensure that all operators
               inducing self-loops have non-negative costs. */
            si_costs[op_id] = max(0, si_costs[op_id]);
        }
    }
}

// ____________________________________________________________________________
int ExplicitAbstraction::get_num_transitions(int op_id) const {
    return num_transitions_by_operator[op_id];
}

// ____________________________________________________________________________
bool ExplicitAbstraction::operator_induces_self_loop(int op_id) const {
    return has_loop[op_id];
}

// ____________________________________________________________________________
bool ExplicitAbstraction::operator_is_active(int op_id) const {
    return has_outgoing[op_id];
}

}
