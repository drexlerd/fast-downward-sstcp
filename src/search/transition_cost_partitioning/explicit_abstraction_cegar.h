#ifndef TRANSITION_COST_PARTITIONING_EXPLICIT_ABSTRACTION_CEGAR_H
#define TRANSITION_COST_PARTITIONING_EXPLICIT_ABSTRACTION_CEGAR_H

#include "explicit_abstraction.h"
#include "types.h"
#include "abstraction_function.h"
#include "split_tree.h"

#include "../algorithms/priority_queues.h"
#include "../cegar/cartesian_set.h"

#include <memory>
#include <vector>
#include <functional>

using namespace std;

namespace transition_cost_partitioning {

/*
  An ExplicitAbstractionCegar is an instantiation of an ExplicitAbstraction
  from a given abstraction generated with CEGAR.
*/
class ExplicitAbstractionCegar : public ExplicitAbstraction {
  private:
    /**
     * We store a treelike version of the refinement hierarchy
     * without domain abstraction to efficiently store cartesian abstract states.
     */
    SplitTree split_tree;
    vector<int> split_variables;
  public:
    /**
     * R6: Moveable and not copyable.
     */
    ExplicitAbstractionCegar() = delete;
    ExplicitAbstractionCegar(
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
      vector<bool> &&has_loop,
      unique_ptr<cegar::SplitTree> split_tree);
    ExplicitAbstractionCegar(const ExplicitAbstractionCegar &other) = delete;
    ExplicitAbstractionCegar& operator=(const ExplicitAbstractionCegar &other) = delete;
    ExplicitAbstractionCegar(ExplicitAbstractionCegar &&other) = default;
    ExplicitAbstractionCegar& operator=(ExplicitAbstractionCegar &&other) = default;
    virtual ~ExplicitAbstractionCegar() = default;

    virtual vector<int> get_split_variables() const override;

    virtual BDD make_state_bdd(int state_id) const override;
    virtual BDD make_transition_bdd_and_cache(const Transition &transition) const override;
    virtual BDD make_transition_bdd(const Transition &transition) const override;
};

}

#endif
