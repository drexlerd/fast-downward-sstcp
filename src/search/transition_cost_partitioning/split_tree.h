#ifndef TRANSITION_COST_PARTITIONING_SATURATOR_H
#define TRANSITION_COST_PARTITIONING_SATURATOR_H

#include "bdd_builder.h"

#include "types.h"

namespace transition_cost_partitioning {
struct Transition;


struct SplitTreeNode {
    NodeID id;
    int var;
    BDD left_vals;
    NodeID left_child;
    BDD right_vals;
    NodeID right_child;

    bool is_leaf() const;
    
    // Leaf
    SplitTreeNode(NodeID id);
    // Inner node
    SplitTreeNode(NodeID id, int var, BDD left_vals, NodeID left_child, BDD right_vals, NodeID right_child);
};

class SplitTree {
  private:
    const TaskInfo &task_info;
    const BddBuilder &bdd_builder;

    vector<SplitTreeNode> nodes;
    vector<int> split_tree_states_offset;
    vector<bool> split_tree_states;
    vector<int> split_variables;
    /**
     * Preallocated memory for reuse.
     */
    mutable vector<const BDD*> source_state;
    mutable vector<bool> source_vars;
    mutable vector<const BDD*> target_state;
    mutable vector<bool> target_vars;

  private:
    void fill_state(int state_id, vector<const BDD*> &state_bdds, vector<bool> &state_vars) const;

  public:
    /**
     * Construct SplitTree from cegar::SplitTree.
     */
    SplitTree(
      const TaskInfo &task_info,
      const BddBuilder &bdd_builder,
      unique_ptr<cegar::SplitTree> split_tree);

    /**
     * Compute bdd for state.
     */
    BDD make_bdd(int state_id) const;

    /**
     * Compute the CartesianSet for each abstract state.
     * We use references to Bitsets that are stored at the edges of the SplitTree.
     * The lower the split in the tree, the more refined it is.
     */
    BDD regress(const Transition &transition) const;
};

}

#endif
