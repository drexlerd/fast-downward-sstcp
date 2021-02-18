#ifndef TRANSITION_COST_PARTITIONING_OPTIMAL_TRANSITION_COST_PARTITIONING_HEURISTIC_H
#define TRANSITION_COST_PARTITIONING_OPTIMAL_TRANSITION_COST_PARTITIONING_HEURISTIC_H

#include "types.h"

#include "../heuristic.h"

#include "../lp/lp_solver.h"

namespace transition_cost_partitioning {
class BddBuilder;
class Abstraction;
class TaskInfo;

/*
  This transition cost partitioning heuristics has a cost variable for each
  abstract transition instead of each concrete transition.
  This saves a lot of variables and makes the linear program easier to solve
  if the number of distinguished contexts is large.
*/
class OptimalTransitionCostPartitioningHeuristic : public Heuristic {
    AbstractionFunctions _abstraction_functions;
    lp::LPSolver _lp_solver;
    const bool _allow_negative_costs;
    bool _found_initial_h_value;

    /*
      Column indices for abstraction variables indexed by abstraction id.
      Variable abstraction_variables[A] encodes the shortest distance of the
      current abstract state to its nearest abstract goal state in abstraction
      A using the cost partitioning.
    */
    std::vector<int> _abstraction_variables;

    /*
      Column indices for distance variables indexed by abstraction id and
      abstract state id. Variable distance_variables[A][s] encodes the distance
      of abstract state s in abstraction A from the current abstract state
      using the cost partitioning.
    */
    std::vector<std::vector<int>> _distance_variables;

    /*
      Column indices for context cost variables indexed by abstraction id,
      context id and operator id. 
      Variable transition_cost_variables[A][s][t] encodes the cost of
      of transition t in state s in abstraction A
    */
    std::vector<std::vector<int>> _transition_cost_variables;

    /*
      Precompute dead end states to avoid solving an lp for dead ends
    */
    std::vector<std::vector<int>> _h_values;

    /*
      Cache the variables corresponding to the current state in all
      abstractions. This speeds up resetting the bounds for each evaluation.
    */
    std::vector<int> _current_abstract_state_vars;

  private:
    void generate_lp(
      const BddBuilder &bdd_builder,
      const vector<unique_ptr<Abstraction>> &abstractions,
      TaskInfo &task_info);

    void add_abstraction_variables(
        const Abstraction &abstraction,
        size_t abstraction_id,
        std::vector<lp::LPVariable> &lp_variables);
    void add_abstraction_constraints(
        const Abstraction &abstraction,
        size_t abstraction_id,
        std::vector<lp::LPConstraint> &lp_constraints);

    void add_context_cost_constraints(
        const BddBuilder &bdd_builder,
        const vector<unique_ptr<Abstraction>> &abstractions,
        TaskInfo &task_info,
        std::vector<lp::LPConstraint> &lp_constraints);

    void generate_contexts_recursively(
      const BddBuilder &bdd_builder,
      const vector<vector<BDD>> state_bdds,
      const vector<vector<BDD>> state_transition_bdds,
      lp::LPConstraint &&current_constraint,
      const vector<unique_ptr<Abstraction>> &abstractions,
      std::vector<lp::LPConstraint> &lp_constraints,
      const BDD cur_context,
      const int cur_op_id,
      int cur_abs_id);

    void release_memory();

protected:
    virtual int compute_heuristic(const GlobalState &global_state) override;

public:
    explicit OptimalTransitionCostPartitioningHeuristic(options::Options &opts);

    void print_statistics();
};
}

#endif
