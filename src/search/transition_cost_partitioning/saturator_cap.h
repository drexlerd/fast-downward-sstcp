#ifndef TRANSITION_COST_PARTITIONING_SATURATOR_CAP_H
#define TRANSITION_COST_PARTITIONING_SATURATOR_CAP_H

#include "saturator.h"
#include "types.h"

namespace transition_cost_partitioning {

class SaturatorCap : public Saturator {
    const bool cap;
    const bool spd;
    const bool saturate_negative_infinity;
public:
    explicit SaturatorCap(const options::Options &opts);

    virtual SaturatorResultOcf saturate_ocf(
        const Abstraction &abstraction,
        const vector<int> &ocf,
        vector<int> &&h_values,
        int state_id) const override;

    virtual SaturatorResultTcf saturate_tcf(
        const Abstraction &abstraction,
        AbstractTransitionCostFunction &&tcf,
        const CostFunctionStateDependent &sdac,
        vector<int> &&h_values,
        int state_id) const override;
};
}

#endif
