#ifndef TRANSITION_COST_PARTITIONING_SATURATOR_LP_H
#define TRANSITION_COST_PARTITIONING_SATURATOR_LP_H

#include "saturator.h"
#include "types.h"

using namespace std;

namespace transition_cost_partitioning {

/**
 * For the transition saturator we allow two different types of objectives.
 * The OPERATORS objective is similar to the one used in the operator saturator.
 * The TRANSITIONS objective minimizes the sum of costs assigned to abstract transitions
 * needed to preserve the h-value of the state.
 */
enum class ObjectiveType {
    TRANSITIONS,
    OPERATORS,
};


class SaturatorLP : public Saturator {
    const ObjectiveType objective_type;
    const bool spd;
    const bool saturate_negative_infinity;

public:
    explicit SaturatorLP(const options::Options &opts);


    /*
    For an abstraction A and operator cost function ocf, build the following LP:
    
    Variables:
    * h[s] for each abstract state s in A
    * c[o] for each operator o

    Objective function: MIN sum_{o in operators} c[o]

    * For <s, o, s'> in abstract transitions
        h[s] <= c[o] + h[s']
        Note that self-loops reduce to a special case that can
        be encoded in the variable bounds:
        c[o] >= 0
    * For each abstract goal state s:
        h[s] <= 0 (encoded in variable bounds)
    * For all abstract states s with required heuristic value h:
        h[s] = h
    * For all other states s:
        -INF <= h[s] <= INF
    * For the given operator cost function and all operators o:
        -INf <= c[o] <= ocf(o)
    */
    virtual SaturatorResultOcf saturate_ocf(
        const Abstraction &abstraction,
        const std::vector<int> &ocf,
        std::vector<int> &&h_values,
        int state_id) const override;


    /*
    For an abstraction A and transition cost function tcf, build the following LP:

    Variables:
    * h[s] for each abstract state s in A
    * c[t] for each transition t

    Objective function: MIN sum_{t in transitions} c[t]

    * For t = <s, o, s'> in abstract transitions with s != s'
        h[s] <= c[t] + h[s']
    * For each abstract goal state s:
        h[s] <= 0 (encoded in variable bounds)
    * For all abstract states s with required heuristic value h:
        h[s] = h
    * For all other states s:
        -INF <= h[s] <= INF
    * For the given transition cost function and all transitions t:
        -INF <= c[t] <= tcf(t)
    */
    virtual SaturatorResultTcf saturate_tcf(
        const Abstraction &abstraction,
        AbstractTransitionCostFunction &&tcf,
        const CostFunctionStateDependent &sdac,
        vector<int> &&h_values,
        int state_id) const override;
};
}

#endif
