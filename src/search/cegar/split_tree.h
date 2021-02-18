#ifndef COST_SATURATION_SD_CONTEXT_SPLIT_TREE_H
#define COST_SATURATION_SD_CONTEXT_SPLIT_TREE_H


#include "types.h"
#include "cartesian_set.h"

#include "../tasks/domain_abstracted_task.h"

#include <memory>
#include <unordered_map>
#include <set>

using namespace std;

namespace cegar {

struct SplitTreeNode {
    // The identifier of the node.
    NodeID id;
    // The index of the variable in this node, -1 for leaf node
    int var;
    // The values included in the cartesian set of the left child
    Bitset left_vals;
    NodeID left_child;
    // The values included in
    Bitset right_vals;
    NodeID right_child;

    /**
     * Constructor.
     */
    SplitTreeNode(NodeID id);

    /**
     * Returns true if SplitTreeNode is an uninitialized leaf.
     */
    bool is_leaf() const;
};


/**
 * A context split tree is essentially the same structure as a refinement hierarchy.
 */
class SplitTree {
  private:
    /**
     * For landmark task we used value abstraction.
     * The context split tree splits over the concrete domain.
     * Hence, the domain abstraction is inverted.
     */
    extra_tasks::DomainAbstractedTask* dat;

    const vector<int> concrete_domain_sizes;

    int next_node_id;
    vector<SplitTreeNode> nodes;

    /**
     * Initialized after construction using initialize method.
     */
    vector<int> split_tree_states_offset;
    vector<bool> split_tree_states;
    vector<int> split_variables;
    set<int> split_variables_ordered;  // only for construction

  private:
    /**
     * Traverse the SplitTree in depth-first manner and collect information
     * about which edge to take to get to the respective leaf node of each abstract state.
     */
    void compute_split_tree_states(
      const SplitTreeNode &node,
      vector<bool> &current_state,
      vector<vector<bool>> &split_tree_states) const;

  public:
    SplitTree(const shared_ptr<AbstractTask> &task);

    /**
     * initialize additional attributes after construction.
     */
    void initialize();

    /**
     * Split the SplitTreeNode identified with node_id
     * This is being called in the CEGAR refinement loop
     */
    pair<NodeID, NodeID> split(
      NodeID split_node_id, int left_state_id, int right_state_id, 
      int var, Bitset left_vals, Bitset right_vals);

    /**
     * Get the number of contexts (= number of leaf nodes)
     */
    unsigned int size() const;

    /**
     * Extract attributes.
     */
    vector<SplitTreeNode> extract_nodes();
    vector<int> extract_split_tree_states_offset();
    vector<bool> extract_split_tree_states();
    vector<int> extract_split_variables();
};

}

#endif
