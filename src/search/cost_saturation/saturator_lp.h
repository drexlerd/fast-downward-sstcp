#ifndef COST_SATURATION_SATURATOR_LP_H
#define COST_SATURATION_SATURATOR_LP_H

#include "saturator.h"
#include "types.h"

namespace cost_saturation {
/*
  TODO: Describe how we handle infinities.

  For an abstraction A and cost function cost, build the following LP:

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
   * For the given cost function and all operators o:
     c[o] <= cost(o)

  Bounds:
    TODO
*/
class SaturatorLP : public Saturator {
public:
    explicit SaturatorLP(const options::Options &opts);

    virtual SaturatorResult saturate(
        const Abstraction &abstraction,
        int abstraction_id,
        const std::vector<int> &costs,
        std::vector<int> &&h_values,
        int state_id) const override;
};
}

#endif
