#ifndef TRANSITION_COST_PARTITIONING_SATURATOR_H
#define TRANSITION_COST_PARTITIONING_SATURATOR_H

#include "types.h"
#include "abstract_transition_cost_function.h"

class State;

namespace options {
class OptionParser;
class Options;
}

namespace transition_cost_partitioning {
class TransitionWeights;
class CostFunctionStateDependent;
class Stats;
struct SaturatorInfo;

enum class Reachable {
    ALL,
    FROM_INIT,
    FROM_STATE
};

/*
  The saturator result using operator cost functions.
*/
struct SaturatorResultOcf {
    std::vector<int> socf;
    std::vector<int> h_values;
    bool saturate_negative_infinity;

    SaturatorResultOcf(std::vector<int> &&socf, std::vector<int> &&h_values, bool saturate_negative_infinity)
        : socf(std::move(socf)),
          h_values(std::move(h_values)),
          saturate_negative_infinity(saturate_negative_infinity) {
    }
};

/*
  The saturator result using transition cost functions.
*/
struct SaturatorResultTcf {
    AbstractTransitionCostFunction stcf;
    std::vector<int> h_values;
    bool saturate_negative_infinity;

    SaturatorResultTcf(AbstractTransitionCostFunction &&stcf, std::vector<int> &&h_values, bool saturate_negative_infinity) :
          stcf(std::move(stcf)),
          h_values(std::move(h_values)),
          saturate_negative_infinity(saturate_negative_infinity) {
    }
};


class Saturator {
protected:
    // we only evaluate general saturators.
    const bool use_general_costs;
    // Recomputation is only considered in the ocf case.
    // In the tcf case we expect saturators to tighten the consistency constraint.
    const bool recompute_h_values;
    // The type of reachability function used.
    Reachable reachable;

    // Get saturator result from resulting operator cost functions.
    virtual SaturatorResultOcf get_saturator_result_ocf(
        const Abstraction &abstraction,
        const std::vector<int> &ocf,
        std::vector<int> &&socf,
        std::vector<int> &&h_values,
        bool saturate_negative_infinity) const;

public:
    explicit Saturator(const options::Options &opts);
    virtual ~Saturator() = default;

    // saturate using operator cost functions
    virtual SaturatorResultOcf saturate_ocf(
        const Abstraction &abstraction,
        const std::vector<int> &ocf,
        std::vector<int> &&h_values,
        int state_id) const = 0;

    // saturate using state transition cost functions
    virtual SaturatorResultTcf saturate_tcf(
        const Abstraction &abstraction,
        AbstractTransitionCostFunction &&tcf,
        const CostFunctionStateDependent &sdac,        
        vector<int> &&h_values,
        int state_id) const = 0;

    Reachable get_reachable() const;
};

extern void add_saturator_options(options::OptionParser &parser);
}

#endif
