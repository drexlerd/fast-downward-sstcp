#include "split_tree.h"

#include "utils.h"
#include "gmpxx.h"

#include <iostream>
#include <cassert>
#include <queue>
#include <unordered_set>


using namespace std;

namespace cegar {

// ____________________________________________________________________________
SplitTreeNode::SplitTreeNode(int state_id)
  : id(state_id), var(-1), 
    left_vals(Bitset(0)), left_child(-1), 
    right_vals(Bitset(0)), right_child(-1) { 
}

// ____________________________________________________________________________
bool SplitTreeNode::is_leaf() const {
    return (var == -1) ? true : false;
}


// ____________________________________________________________________________
SplitTree::SplitTree(const shared_ptr<AbstractTask> &task)
  : dat(dynamic_cast<extra_tasks::DomainAbstractedTask*>(task.get())),    
    concrete_domain_sizes(get_concrete_domain_sizes()),
    next_node_id(0) {
    // add root node for trivial abstract state
    nodes.emplace_back(0);
}

// ____________________________________________________________________________
void SplitTree::initialize() {
    int num_states = static_cast<int>(size());
    // 1. Initialize split tree states
    vector<vector<bool>> split_tree_states_temp(num_states);
    vector<bool> current_state;
    const SplitTreeNode &root = nodes[0];
    compute_split_tree_states(
      root,
      current_state,
      split_tree_states_temp);
    // flatten split tree states
    split_tree_states_offset = vector<int>(num_states);
    for (int state_id = 0; state_id < num_states; ++state_id) {
        split_tree_states_offset[state_id] = static_cast<int>(split_tree_states.size());
        vector<bool> &split_state = split_tree_states_temp[state_id];
        split_tree_states.insert(split_tree_states.end(), split_state.begin(), split_state.end());
    }
    // 2. Initialize split variables in descending order (= consistent with bdd topdown variable order)
    split_variables = vector<int>(split_variables_ordered.begin(), split_variables_ordered.end());
    reverse(split_variables.begin(), split_variables.end());
}


// ____________________________________________________________________________
void SplitTree::compute_split_tree_states(
    const SplitTreeNode &node,
    vector<bool> &current_state,
    vector<vector<bool>> &split_tree_states) const {
    if (node.is_leaf()) {
        int state_id = node.id;
        split_tree_states[state_id] = current_state;
        return;
    }
    assert(!node.is_leaf());

    vector<bool> left_state = current_state;
    left_state.emplace_back(true);
    vector<bool> right_state = move(current_state); // we can move the right side.
    right_state.emplace_back(false);

    compute_split_tree_states(nodes[node.left_child], left_state, split_tree_states);
    compute_split_tree_states(nodes[node.right_child], right_state, split_tree_states);
}

// ____________________________________________________________________________
pair<NodeID, NodeID> SplitTree::split(
  NodeID split_node_id, int left_state_id, int right_state_id, int var, 
  Bitset left_vals, Bitset right_vals) {
    split_variables_ordered.insert(var);
    // we add nodes before accessing the node that we are about to split.
    NodeID left_node_id = nodes.size();
    nodes.emplace_back(left_state_id);
    NodeID right_node_id = nodes.size();  
    nodes.emplace_back(right_state_id);

    SplitTreeNode &split_node = nodes[split_node_id];
    split_node.var = var;
    split_node.left_child = left_node_id;
    split_node.right_child = right_node_id;

    // transform into concrete domain if necessary
    if (dat) {
        Bitset concrete_left_vals = Bitset(concrete_domain_sizes[var]);
        Bitset concrete_right_vals = Bitset(concrete_domain_sizes[var]);
        dat->get_concrete_values(var, left_vals, concrete_left_vals);
        dat->get_concrete_values(var, right_vals, concrete_right_vals);
        split_node.left_vals = concrete_left_vals;  
        split_node.right_vals = concrete_right_vals;
    } else {
        split_node.left_vals = left_vals;  
        split_node.right_vals = right_vals;
    }
    assert(split_node.left_vals.is_disjunct(split_node.right_vals));
    assert(static_cast<int>(split_node.left_vals.size()) == concrete_domain_sizes[var]);
    assert(static_cast<int>(split_node.right_vals.size()) == concrete_domain_sizes[var]);

    return make_pair(left_node_id, right_node_id);
}

// ____________________________________________________________________________
unsigned int SplitTree::size() const {
    return (nodes.size() + 1) / 2;
}

// ____________________________________________________________________________
vector<SplitTreeNode> SplitTree::extract_nodes() {
    return move(nodes);
}

// ____________________________________________________________________________
vector<int> SplitTree::extract_split_tree_states_offset() {
    return move(split_tree_states_offset);
}

// ____________________________________________________________________________
vector<bool> SplitTree::extract_split_tree_states() {
    return move(split_tree_states);
}

// ____________________________________________________________________________
vector<int> SplitTree::extract_split_variables() {
    return move(split_variables);
}


}
