#include "split_tree.h"

#include "abstraction.h"
#include "task_info.h"

namespace transition_cost_partitioning {


// ____________________________________________________________________________
SplitTreeNode::SplitTreeNode(NodeID id)
  : id(id), var(-1), left_child(-1), right_child(-1) { 
}

SplitTreeNode::SplitTreeNode(NodeID id, int var, BDD left_vals, NodeID left_child, BDD right_vals, NodeID right_child) 
  : id(id), var(var), left_vals(left_vals), left_child(left_child), right_vals(right_vals), right_child(right_child) {
}

// ____________________________________________________________________________
bool SplitTreeNode::is_leaf() const {
    return (var == -1) ? true : false;
}

// ____________________________________________________________________________
SplitTree::SplitTree(
    const TaskInfo &task_info,
    const BddBuilder &bdd_builder,
    unique_ptr<cegar::SplitTree> split_tree) :
    task_info(task_info),
    bdd_builder(bdd_builder),
    split_tree_states_offset(move(split_tree->extract_split_tree_states_offset())),
    split_tree_states(move(split_tree->extract_split_tree_states())),
    split_variables(move(split_tree->extract_split_variables())) {
    // create new version of nodes
    vector<cegar::SplitTreeNode> cegar_nodes = split_tree->extract_nodes();
    nodes.reserve(cegar_nodes.size());
    for (const cegar::SplitTreeNode &cegar_node : cegar_nodes) {
        if (cegar_node.is_leaf()) {
            nodes.emplace_back(cegar_node.id);
        } else {
            nodes.emplace_back(cegar_node.id, cegar_node.var, 
                bdd_builder.make_bdd(cegar_node.var, cegar_node.left_vals),
                cegar_node.left_child,
                bdd_builder.make_bdd(cegar_node.var, cegar_node.right_vals),
                cegar_node.right_child);
        }
    }
    source_state.resize(task_info.get_num_variables());
    source_vars.resize(task_info.get_num_variables());
    target_state.resize(task_info.get_num_variables());
    target_vars.resize(task_info.get_num_variables());
}

// ____________________________________________________________________________
void SplitTree::fill_state(int state_id, vector<const BDD*> &state_bdds, vector<bool> &state_vars) const {
    for (int var : split_variables) {
        state_vars[var] = false;
    }
    int state_offset = split_tree_states_offset[state_id];
    int i = 0;
    NodeID cur_node_id = 0;
    while (true) {
        const SplitTreeNode &cur_node = nodes[cur_node_id];
        if (cur_node.is_leaf()) {
            break;
        }
        bool left = split_tree_states[state_offset + i];
        ++i;
        int var = cur_node.var;
        state_vars[var] = true;
        if (left) {
            state_bdds[var] = &cur_node.left_vals;
            cur_node_id = cur_node.left_child;
        } else {
            state_bdds[var] = &cur_node.right_vals;
            cur_node_id = cur_node.right_child;
        }        
    }
}

// ____________________________________________________________________________
BDD SplitTree::make_bdd(int state_id) const {
    fill_state(state_id, source_state, source_vars);
    BDD result = bdd_builder.make_one();
    for (int var : split_variables) {
        if (source_vars[var]) {
            result *= *source_state[var];
        }
    }
    return result;
}

// ____________________________________________________________________________
BDD SplitTree::regress(const Transition &transition) const {
    fill_state(transition.source_id, source_state, source_vars);
    fill_state(transition.target_id, target_state, target_vars);
    BDD result = bdd_builder.make_one();
    for (int var : split_variables) {
        if (task_info.operator_has_precondition(transition.op_id, var)) {
            continue;
        }
        // fill with from values if the operator has no precondition.
        BDD var_bdd = bdd_builder.make_one();
        if (source_vars[var]) {
            var_bdd *= *source_state[var];
        }
        if (target_vars[var] && !task_info.operator_mentions_variable(transition.op_id, var)) {
            var_bdd *= *target_state[var];
        }
        result *= var_bdd;
    }
    return result;
}

}